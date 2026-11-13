//! MeshCore Bluetooth Low Energy (BLE) Companion Subsystem
//!
//! Exposes Nordic UART Service (NUS) over BLE GATT, enabling official MeshCore
//! mobile apps (iOS & Android) to discover, pair, and communicate with ThinkNode M1.

use core::cell::UnsafeCell;
use core::future::poll_fn;
use core::sync::atomic::{AtomicU32, AtomicUsize, Ordering};
use core::task::Poll;

use defmt::{debug, error, info};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::waitqueue::AtomicWaker;

/// Shared raw mutex type — see usb.rs for documentation.
type SharedRawMutex = CriticalSectionRawMutex;
use heapless::{String, Vec};
use nrf_softdevice::ble::gatt_server;
use nrf_softdevice::ble::peripheral;
use nrf_softdevice::ble::security::{IoCapabilities, SecurityHandler};
use nrf_softdevice::ble::{Connection, EncryptionInfo, IdentityKey, MasterId, PasskeyReply};
use nrf_softdevice::Softdevice;

use crate::boards::{Board, CurrentBoard};
use crate::companion::protocol::CompanionNode;

#[nrf_softdevice::gatt_service(uuid = "180a")]
pub struct DeviceInformationService {
    #[characteristic(uuid = "2a24", read)]
    pub model_number: Vec<u8, 32>,
    #[characteristic(uuid = "2a26", read)]
    pub firmware_revision: Vec<u8, 32>,
    #[characteristic(uuid = "2a29", read)]
    pub manufacturer_name: Vec<u8, 32>,
}

#[nrf_softdevice::gatt_service(uuid = "6e400001-b5a3-f393-e0a9-e50e24dcca9e")]
pub struct NordicUartService {
    #[characteristic(uuid = "6e400002-b5a3-f393-e0a9-e50e24dcca9e", write, write_without_response)]
    pub rx: Vec<u8, 256>,

    #[characteristic(uuid = "6e400003-b5a3-f393-e0a9-e50e24dcca9e", notify)]
    pub tx: Vec<u8, 256>,
}

#[nrf_softdevice::gatt_server]
pub struct MeshCoreGattServer {
    pub dis: DeviceInformationService,
    pub nus: NordicUartService,
}

/// Responses are queued here and drained by `push_fut`, never notified from
/// inside the GATT callback. Sized for a full contact sync burst
/// (`GET_CONTACTS` emits at most: start + 32 contacts + end).
pub static COMPANION_PUSH_CHANNEL: Channel<SharedRawMutex, Vec<u8, 256>, 40> = Channel::new();

/// Largest companion command accepted over NUS.
const CMD_MAX_LEN: usize = 247;
/// Number of queued commands. Commands arrive one at a time from the app, so
/// a small ring is plenty.
const CMD_RING_SLOTS: usize = 8;

#[derive(Clone, Copy)]
struct CmdSlot {
    len: u8,
    data: [u8; CMD_MAX_LEN],
}

/// Single-producer/single-consumer ring between the GATT callback and
/// `command_fut`.
///
/// The GATT callback must never run companion protocol code: `handle_command`
/// takes locks, reads shared state and can emit many responses. Doing that
/// inside a SoftDevice event callback can stall the SoftDevice and freeze the
/// device. The callback now only copies the bytes here.
struct CmdRing {
    slots: UnsafeCell<[CmdSlot; CMD_RING_SLOTS]>,
    head: AtomicUsize,
    tail: AtomicUsize,
}

// SAFETY: the producer (GATT callback) and consumer (`command_fut`) are
// distinct. The producer writes a slot only after observing it free and
// publishes it with a Release store to `head`; the consumer only reads slots
// below `head` (Acquire) and advances `tail` with Release before reuse.
unsafe impl Sync for CmdRing {}

impl CmdRing {
    const fn new() -> Self {
        Self {
            slots: UnsafeCell::new(
                [CmdSlot {
                    len: 0,
                    data: [0u8; CMD_MAX_LEN],
                }; CMD_RING_SLOTS],
            ),
            head: AtomicUsize::new(0),
            tail: AtomicUsize::new(0),
        }
    }

    /// Enqueue one command. Never blocks, never takes a lock. Returns false if
    /// the command is too long or the ring is full.
    fn push(&self, data: &[u8]) -> bool {
        if data.is_empty() || data.len() > CMD_MAX_LEN {
            return false;
        }

        let head = self.head.load(Ordering::Relaxed);
        let tail = self.tail.load(Ordering::Acquire);
        if head.wrapping_sub(tail) >= CMD_RING_SLOTS {
            return false;
        }

        let idx = head % CMD_RING_SLOTS;
        // SAFETY: `idx` is free (head - tail < SLOTS) and only the producer
        // writes slots.
        unsafe {
            let slot = &mut (*self.slots.get())[idx];
            slot.len = data.len() as u8;
            slot.data[..data.len()].copy_from_slice(data);
        }
        self.head.store(head.wrapping_add(1), Ordering::Release);
        true
    }

    /// Dequeue one command into `out`, returning its length.
    fn pop(&self, out: &mut [u8; CMD_MAX_LEN]) -> Option<usize> {
        let tail = self.tail.load(Ordering::Relaxed);
        let head = self.head.load(Ordering::Acquire);
        if head == tail {
            return None;
        }

        let idx = tail % CMD_RING_SLOTS;
        // SAFETY: the slot was published by the producer (Acquire on head) and
        // only the consumer reads it until it advances `tail`.
        let len = unsafe {
            let slot = &(*self.slots.get())[idx];
            let len = slot.len as usize;
            out[..len].copy_from_slice(&slot.data[..len]);
            len
        };
        self.tail.store(tail.wrapping_add(1), Ordering::Release);
        Some(len)
    }

    fn is_empty(&self) -> bool {
        self.head.load(Ordering::Acquire) == self.tail.load(Ordering::Relaxed)
    }

    /// Discard queued commands. Called by the consumer after a disconnect,
    /// when the producer is no longer running.
    fn clear(&self) {
        let head = self.head.load(Ordering::Acquire);
        self.tail.store(head, Ordering::Release);
    }
}

static BLE_CMD_RING: CmdRing = CmdRing::new();
static BLE_CMD_WAKER: AtomicWaker = AtomicWaker::new();

/// Contact operation deferred out of the command handler so contact storage
/// never runs inside the BLE/SoftDevice path.
pub enum ContactOperation {
    AddOrUpdate {
        pub_key: [u8; 32],
        callsign: String<32>,
        node_id: [u8; 4],
        last_seen_epoch: u32,
        latitude: i32,
        longitude: i32,
        flags: u8,
        contact_type: u8,
        out_path: [u8; 64],
        out_path_len: u8,
        last_advert_ts: u32,
    },
    /// Clear a contact's learned path.
    ResetPath {
        pub_key: [u8; 32],
    },
    Remove {
        pub_key: [u8; 32],
    },
}

/// Channel for deferring contact mutations outside the GATT callback's
/// SoftDevice critical section. Capacity 4 is sufficient since contact
/// operations are infrequent and the channel is drained promptly.
pub static CONTACT_OP_CHANNEL: Channel<SharedRawMutex, ContactOperation, 4> = Channel::new();

/// Applies deferred contact mutations away from the GATT callback and the
/// main routing loop, in its own task/future state.
#[embassy_executor::task]
pub async fn contact_task() -> ! {
    loop {
        match CONTACT_OP_CHANNEL.receive().await {
            ContactOperation::AddOrUpdate {
                pub_key,
                callsign,
                node_id,
                last_seen_epoch,
                latitude,
                longitude,
                flags,
                contact_type,
                out_path,
                out_path_len,
                last_advert_ts,
            } => {
                crate::companion::usb::DISCOVERED_CONTACTS.lock(|c| {
                    let mut list = c.borrow_mut();
                    if let Some(ct) = list.iter_mut().find(|ct| ct.pub_key == pub_key) {
                        ct.callsign = callsign;
                        ct.node_id = node_id;
                        ct.last_seen_epoch = last_seen_epoch;
                        ct.latitude = latitude;
                        ct.longitude = longitude;
                        ct.flags = flags;
                        ct.contact_type = contact_type;
                        ct.out_path = out_path;
                        ct.out_path_len = out_path_len;
                        ct.last_advert_ts = last_advert_ts;
                    } else if list.len() < list.capacity() {
                        let _ = list.push(crate::meshcore::mesh::DiscoveredContact {
                            pub_key,
                            callsign,
                            node_id,
                            last_seen_epoch,
                            rssi: 0,
                            snr: 0,
                            latitude,
                            longitude,
                            flags,
                            contact_type,
                            out_path,
                            out_path_len,
                            last_advert_ts,
                        });
                    }
                });
            }
            ContactOperation::ResetPath { pub_key } => {
                crate::companion::usb::DISCOVERED_CONTACTS.lock(|c| {
                    if let Some(ct) = c.borrow_mut().iter_mut().find(|ct| ct.pub_key == pub_key) {
                        ct.out_path = [0u8; 64];
                        ct.out_path_len = 0xFF;
                    }
                });
            }
            ContactOperation::Remove { pub_key } => {
                crate::companion::usb::DISCOVERED_CONTACTS.lock(|c| {
                    let mut list = c.borrow_mut();
                    if let Some(pos) = list.iter().position(|ct| ct.pub_key == pub_key) {
                        list.remove(pos);
                    }
                });
            }
        }
        // Persist a structural change. The storage task's hash check means
        // volatile-only updates do not write flash.
        crate::storage::request_contacts();
    }
}

/// Stream `GET_CONTACTS` responses with backpressure so a large contact list
/// (up to `MAX_CONTACTS`) never overflows `COMPANION_PUSH_CHANNEL`.
async fn stream_contacts() {
    use crate::companion::protocol;

    let count = protocol::contacts_count();

    let mut start = Vec::<u8, 256>::new();
    let _ = start.extend_from_slice(&protocol::encode_contacts_start(count));
    let _ = COMPANION_PUSH_CHANNEL.send(start).await;

    let mut max_lastmod = 0u32;
    for idx in 0..count as usize {
        let Some(c) = protocol::contact_at(idx) else {
            continue;
        };
        if c.last_seen_epoch > max_lastmod {
            max_lastmod = c.last_seen_epoch;
        }
        let resp = protocol::serialize_contact(&c);
        let mut vec = Vec::<u8, 256>::new();
        let _ = vec.extend_from_slice(&resp);
        let _ = COMPANION_PUSH_CHANNEL.send(vec).await;
    }

    let mut end = Vec::<u8, 256>::new();
    let _ = end.extend_from_slice(&protocol::encode_contacts_end(max_lastmod));
    let _ = COMPANION_PUSH_CHANNEL.send(end).await;
}

pub struct MeshCoreSecurityHandler;

impl SecurityHandler for MeshCoreSecurityHandler {
    fn io_capabilities(&self) -> IoCapabilities {
        IoCapabilities::DisplayOnly
    }

    fn can_bond(&self, _conn: &Connection) -> bool {
        // Bond storage is not implemented: nrf-softdevice has no built-in bond
        // store, so `on_bonded` would have to persist the keys itself.
        //
        // Enabling bonding without implementing `on_bonded` is fatal: when the
        // SoftDevice reports a completed bond (BLE_GAP_EVT_AUTH_STATUS with
        // bonded=1), the trait's default `on_bonded` panics, taking down USB,
        // BLE and the whole executor. That was the contact-add freeze.
        false
    }

    fn on_bonded(
        &self,
        _conn: &Connection,
        _master_id: MasterId,
        _key: EncryptionInfo,
        _peer_id: IdentityKey,
    ) {
        // Defensive: never let a bonding event panic. If bonding is enabled in
        // the future, persist the key here instead.
    }

    fn request_mitm_protection(&self, _conn: &Connection) -> bool {
        true
    }

    fn display_passkey(&self, _passkey: &[u8; 6]) {
        // No logging: this runs from a SoftDevice callback.
    }

    fn enter_passkey(&self, reply: PasskeyReply) {
        let pin = BLE_PIN.load(Ordering::Relaxed);
        let mut buf = [b'0'; 6];
        let mut value = pin;
        for i in (0..6).rev() {
            buf[i] = b'0' + (value % 10) as u8;
            value /= 10;
        }
        let _ = reply.reply(Some(&buf));
    }
}

static SECURITY_HANDLER: MeshCoreSecurityHandler = MeshCoreSecurityHandler;

#[embassy_executor::task]
pub async fn softdevice_task(sd: &'static Softdevice) -> ! {
    sd.run().await
}

#[embassy_executor::task]
pub async fn ble_companion_task(
    sd: &'static Softdevice,
    server: MeshCoreGattServer,
    public_key: [u8; 32],
    private_key: [u8; 64],
) {
    info!("Starting MeshCore BLE companion advertising...");

    let callsign = crate::storage::get_callsign();
    let mut node = CompanionNode::new(
        callsign.as_str(),
        CurrentBoard::NAME,
        CurrentBoard::CAPABILITIES,
        public_key,
        private_key,
    );

    // Use fast advertising initially (20ms interval) for faster discovery,
    // then switch to slow (211ms) after the first connection to save power.
    // Interval is in 625µs units: 20ms = 32, 211ms = 338.
    let mut adv_config = peripheral::Config {
        interval: 32, // 20ms fast advertising
        ..peripheral::Config::default()
    };

    // Nordic UART Service 128-bit UUID for advertisement
    // 6e400001-b5a3-f393-e0a9-e50e24dcca9e (reversed for little-endian wire format)
    let nus_uuid_128: [u8; 16] = [
        0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0, 0x93, 0xf3, 0xa3, 0xb5, 0x01, 0x00, 0x40, 0x6e,
    ];

    let mut adv_data = [0u8; 31];
    let mut scan_data = [0u8; 31];

    // Flags: LE General Discoverable Mode, BR/EDR Not Supported
    adv_data[0] = 2; // length
    adv_data[1] = 0x01; // Flags
    adv_data[2] = 0x06;

    // Complete 128-bit Service UUID list
    adv_data[3] = 17; // 1 (type) + 16 (uuid)
    adv_data[4] = 0x07; // Complete list of 128-bit Service UUIDs
    adv_data[5..21].copy_from_slice(&nus_uuid_128);

    // Scan response data: Complete Local Name (MeshCore-ThinkNode)
    let name = b"Ferrix-M1";
    scan_data[0] = (name.len() + 1) as u8;
    scan_data[1] = 0x09; // Complete Local Name
    scan_data[2..2 + name.len()].copy_from_slice(name);

    let adv = peripheral::ConnectableAdvertisement::ScannableUndirected {
        adv_data: &adv_data[..21],
        scan_data: &scan_data[..2 + name.len()],
    };

    let mut model = Vec::<u8, 32>::new();
    let _ = model.extend_from_slice(b"Ferrix Companion");
    let _ = server.dis.model_number_set(&model);

    let mut fw = Vec::<u8, 32>::new();
    let _ = fw.extend_from_slice(b"Ferrix 0.1.0");
    let _ = server.dis.firmware_revision_set(&fw);

    let mut mfg = Vec::<u8, 32>::new();
    let _ = mfg.extend_from_slice(b"Elecrow");
    let _ = server.dis.manufacturer_name_set(&mfg);

    loop {
        info!("BLE: Waiting for pairable connection from MeshCore companion app (PIN 123456)...");

        let conn = match peripheral::advertise_pairable(sd, adv, &adv_config, &SECURITY_HANDLER).await {
            Ok(mut c) => {
                info!("BLE: Phone connected & pairing!");
                // Request maximum BLE data length (251 bytes) instead of the default 27 bytes.
                // This increases throughput for channel sync and other large responses.
                let _ = c.data_length_update(None);
                // Disable slave latency during sync so the device wakes at every connection event.
                let _ = c.ignore_slave_latency(true);
                c
            }
            Err(e) => {
                error!("BLE: Advertising error: {:?}", e);
                embassy_time::Timer::after_secs(1).await;
                continue;
            }
        };

        // The GATT callback only copies the command into the lock-free ring and
        // wakes `command_fut`. It must not parse commands, take locks, allocate
        // or log: `handle_command` can emit dozens of responses (contact sync)
        // and doing that inside a SoftDevice callback can stall the SoftDevice.
        let gatt_fut = gatt_server::run(&conn, &server, |event| match event {
            MeshCoreGattServerEvent::Nus(NordicUartServiceEvent::RxWrite(data)) => {
                if !BLE_CMD_RING.push(data.as_slice()) {
                    // Ring full: drop rather than spin inside the callback.
                    // Command drops are counted and logged by `command_fut`.
                    BLE_CMD_DROPS.fetch_add(1, Ordering::Relaxed);
                }
                BLE_CMD_WAKER.wake();
            }
            MeshCoreGattServerEvent::Nus(NordicUartServiceEvent::TxCccdWrite {
                notifications: _,
            }) => {
                // Never log from a SoftDevice callback.
            }
            _ => {}
        });

        // Companion protocol processing, running in normal task context. One
        // command at a time, then yield so `push_fut` can drain the responses
        // before the next command is handled.
        let mut cmd_buf = [0u8; CMD_MAX_LEN];
        let command_fut = async {
            loop {
                if let Some(len) = BLE_CMD_RING.pop(&mut cmd_buf) {
                    let drops = BLE_CMD_DROPS.swap(0, Ordering::Relaxed);
                    if drops != 0 {
                        debug!("BLE: dropped {} command(s) because the ring was full", drops);
                    }
                    if cmd_buf[..len].first() == Some(&crate::companion::protocol::CMD_GET_CONTACTS) {
                        stream_contacts().await;
                    } else {
                        node.handle_command(&cmd_buf[..len], |resp| {
                            let mut vec = Vec::<u8, 256>::new();
                            let copy_len = resp.len().min(247);
                            let _ = vec.extend_from_slice(&resp[..copy_len]);
                            if COMPANION_PUSH_CHANNEL.try_send(vec).is_err() {
                                debug!("BLE: response queue full, dropping response");
                            }
                        });
                    }
                    embassy_futures::yield_now().await;
                } else {
                    // Register before the emptiness check to avoid a lost wake.
                    poll_fn(|cx| {
                        BLE_CMD_WAKER.register(cx.waker());
                        if BLE_CMD_RING.is_empty() {
                            Poll::Pending
                        } else {
                            Poll::Ready(())
                        }
                    })
                    .await;
                }
            }
        };

        let push_fut = async {
            loop {
                let msg = COMPANION_PUSH_CHANNEL.receive().await;
                let mut vec = Vec::<u8, 256>::new();
                let copy_len = msg.len().min(247);
                let _ = vec.extend_from_slice(&msg[..copy_len]);
                // The SoftDevice HVX queue can be momentarily full during a
                // contact sync burst. Retry briefly instead of dropping.
                for attempt in 0..8 {
                    match server.nus.tx_notify(&conn, &vec) {
                        Ok(()) => break,
                        Err(e) => {
                            if attempt == 7 {
                                debug!("BLE NUS tx_notify push failed: {:?}", e);
                            } else {
                                embassy_time::Timer::after_millis(5).await;
                            }
                        }
                    }
                }
            }
        };

        match embassy_futures::select::select3(gatt_fut, command_fut, push_fut).await {
            embassy_futures::select::Either3::First(e) => {
                info!("BLE: Phone disconnected ({:?}), resuming advertising...", e);
                // Drop anything left over from the closed connection so the
                // next app session never receives stale responses.
                BLE_CMD_RING.clear();
                while COMPANION_PUSH_CHANNEL.try_receive().is_ok() {}
                // Switch to slow advertising (211ms) after the first connection to save power.
                if adv_config.interval == 32 {
                    adv_config.interval = 338; // 211ms slow advertising
                    info!("BLE: Switched to slow advertising (211ms)");
                }
            }
            embassy_futures::select::Either3::Second(_) => {}
            embassy_futures::select::Either3::Third(_) => {}
        }
    }
}

pub static BLE_STATUS: AtomicU32 = AtomicU32::new(0);
/// Counts commands dropped because the GATT command ring was full.
static BLE_CMD_DROPS: AtomicU32 = AtomicU32::new(0);
/// Pairing PIN reported to the app and used by the security handler.
pub static BLE_PIN: AtomicU32 = AtomicU32::new(123456);

unsafe extern "C" fn sd_fault_handler(id: u32, pc: u32, info: u32) {
    defmt::error!("Softdevice fault: id={=u32:#x} pc={=u32:#x} info={=u32:#x}", id, pc, info);
    // A SoftDevice assert is fatal. Signal it with 10 blue blinks so the
    // failure can be identified without a debugger.
    crate::fatal_blink_forever(10)
}

#[embassy_executor::task]
pub async fn ble_companion_supervisor_task(
    spawner: embassy_executor::Spawner,
    public_key: [u8; 32],
    private_key: [u8; 64],
) {
    // Wait 500ms so USB CDC-ACM and SX1262 LoRa tasks are completely booted
    embassy_time::Timer::after_millis(500).await;
    BLE_STATUS.store(1, Ordering::Relaxed);
    info!("BLE: Initializing SoftDevice S140 subsystem...");

    let sd_config = nrf_softdevice::Config {
        clock: Some(nrf_softdevice::raw::nrf_clock_lf_cfg_t {
            source: nrf_softdevice::raw::NRF_CLOCK_LF_SRC_RC as u8,
            rc_ctiv: 16,
            rc_temp_ctiv: 2,
            accuracy: nrf_softdevice::raw::NRF_CLOCK_LF_ACCURACY_500_PPM as u8,
        }),
        common_vs_uuid: Some(nrf_softdevice::raw::ble_common_cfg_vs_uuid_t {
            vs_uuid_count: 2,
        }),
        conn_gap: Some(nrf_softdevice::raw::ble_gap_conn_cfg_t {
            conn_count: 1,
            event_length: 24,
        }),
        conn_gatts: Some(nrf_softdevice::raw::ble_gatts_conn_cfg_t {
            hvn_tx_queue_size: 16,
        }),
        conn_gatt: Some(nrf_softdevice::raw::ble_gatt_conn_cfg_t {
            att_mtu: 247,
        }),
        gatts_attr_tab_size: Some(nrf_softdevice::raw::ble_gatts_cfg_attr_tab_size_t {
            attr_tab_size: 1408,
        }),
        gap_role_count: Some(nrf_softdevice::raw::ble_gap_cfg_role_count_t {
            adv_set_count: 1,
            periph_role_count: 1,
            central_role_count: 0,
            central_sec_count: 0,
            _bitfield_1: nrf_softdevice::raw::ble_gap_cfg_role_count_t::new_bitfield_1(0),
        }),
        ..Default::default()
    };

    // Safely probe SoftDevice enablement without touching SCB->VTOR
    let p_clock = sd_config.clock.as_ref().map(|x| x as _).unwrap_or(core::ptr::null());
    let ret = unsafe { nrf_softdevice::raw::sd_softdevice_enable(p_clock, Some(sd_fault_handler as _)) };
    if ret != 0 {
        error!("BLE: sd_softdevice_enable returned status code {=u32:#x}", ret);
        BLE_STATUS.store(0x1000 | ret, Ordering::Relaxed);
        return;
    }

    info!("BLE: SoftDevice S140 successfully enabled!");
    BLE_STATUS.store(2, Ordering::Relaxed);

    // Disable raw SD so that embassy's Softdevice::enable can manage state cleanly
    unsafe { nrf_softdevice::raw::sd_softdevice_disable() };

    let sd = Softdevice::enable(&sd_config);
    if let Ok(server) = MeshCoreGattServer::new(sd) {
        if spawner.spawn(softdevice_task(sd)).is_ok()
            && spawner.spawn(crate::storage::storage_task(sd)).is_ok()
            && spawner.spawn(ble_companion_task(sd, server, public_key, private_key)).is_ok()
        {
            info!("Ferrix BLE companion service successfully started!");
            BLE_STATUS.store(3, Ordering::Relaxed);
        } else {
            BLE_STATUS.store(4, Ordering::Relaxed);
        }
    } else {
        BLE_STATUS.store(5, Ordering::Relaxed);
    }
}

pub fn init_ble_companion(
    spawner: embassy_executor::Spawner,
    public_key: [u8; 32],
    private_key: [u8; 64],
) -> bool {
    spawner.spawn(ble_companion_supervisor_task(spawner, public_key, private_key)).is_ok()
}
