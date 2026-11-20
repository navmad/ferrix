//! MeshCore Bluetooth Low Energy (BLE) Companion Subsystem
//!
//! Exposes Nordic UART Service (NUS) over BLE GATT, enabling official MeshCore
//! mobile apps (iOS & Android) to discover, pair, and communicate with ThinkNode M1.

use core::cell::RefCell;
use core::future::poll_fn;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use core::task::Poll;

use defmt::{debug, error, info, warn};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::channel::Channel;
use embassy_sync::waitqueue::AtomicWaker;

use heapless::spsc::Queue;
use heapless::Vec;
use nrf_softdevice::ble::gatt_server;
use nrf_softdevice::ble::peripheral;
use nrf_softdevice::ble::security::{IoCapabilities, SecurityHandler};
use nrf_softdevice::ble::{Connection, EncryptionInfo, IdentityKey, MasterId, PasskeyReply};
use nrf_softdevice::Softdevice;
use static_cell::StaticCell;

use ferrix_core::command::PushSink;
use ferrix_core::companion::cli::CliHost;
use ferrix_core::storage::bonds::{self, Bond};
use ferrix_core::companion::session::{CompanionLink, CompanionSession};

#[allow(unsafe_code)]
mod gatt {
    // `nrf_softdevice::gatt_service` expands `unsafe` SoftDevice FFI calls for
    // characteristic accessors; keep the allow scoped strictly to these macro
    // definitions.
    use heapless::Vec;

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
        #[characteristic(uuid = "6e400002-b5a3-f393-e0a9-e50e24dcca9e", write, write_without_response, security = "mitm")]
        pub rx: Vec<u8, 256>,

        #[characteristic(uuid = "6e400003-b5a3-f393-e0a9-e50e24dcca9e", notify, security = "mitm")]
        pub tx: Vec<u8, 256>,
    }

    /// Legacy Nordic/Adafruit buttonless DFU service (`00001530-...`).
    ///
    /// Registered after `dis` and `nus` so bonded phones' cached NUS attribute
    /// handles never shift. Writing `0x01` to `control_point` reboots into the
    /// bootloader's BLE OTA mode (`GPREGRET = 0xA8`).
    #[nrf_softdevice::gatt_service(uuid = "00001530-1212-efde-1523-785feabcd123")]
    pub struct LegacyDfuService {
        #[characteristic(uuid = "00001531-1212-efde-1523-785feabcd123", write, notify, security = "mitm")]
        pub control_point: Vec<u8, 20>,

        #[characteristic(uuid = "00001532-1212-efde-1523-785feabcd123", write_without_response, security = "mitm")]
        pub packet: Vec<u8, 20>,

        #[characteristic(uuid = "00001534-1212-efde-1523-785feabcd123", read, security = "mitm")]
        pub revision: Vec<u8, 2>,
    }

    #[nrf_softdevice::gatt_server]
    pub struct MeshCoreGattServer {
        pub dis: DeviceInformationService,
        pub nus: NordicUartService,
        pub dfu: LegacyDfuService,
    }
}

pub use gatt::{
    DeviceInformationService, LegacyDfuService, LegacyDfuServiceEvent, MeshCoreGattServer,
    MeshCoreGattServerEvent, NordicUartService, NordicUartServiceEvent,
};

/// Responses are queued here and drained by `push_fut`, never notified from
/// inside the GATT callback. Sized for a full contact sync burst
/// (`GET_CONTACTS` emits at most: start + 32 contacts + end).

/// Largest companion command accepted over NUS.
const CMD_MAX_LEN: usize = 247;
/// Number of slots in the SPSC command queue (7 usable). Commands arrive one
/// at a time from the app, so a small ring is plenty.
const CMD_RING_SLOTS: usize = 8;

#[derive(Clone, Copy)]
struct CmdSlot {
    len: u8,
    data: [u8; CMD_MAX_LEN],
}

impl CmdSlot {
    fn from_slice(data: &[u8]) -> Option<Self> {
        if data.is_empty() || data.len() > CMD_MAX_LEN {
            return None;
        }
        let mut slot = Self {
            len: data.len() as u8,
            data: [0u8; CMD_MAX_LEN],
        };
        slot.data[..data.len()].copy_from_slice(data);
        Some(slot)
    }
}

/// Lock-free single-producer/single-consumer queue between the GATT callback
/// and `command_fut`. Split once inside `ble_companion_task` into disjoint
/// producer/consumer halves so the GATT callback never takes locks or runs
/// companion protocol code inside SoftDevice event context.
static BLE_CMD_QUEUE: StaticCell<Queue<CmdSlot, CMD_RING_SLOTS>> = StaticCell::new();
static BLE_CMD_WAKER: AtomicWaker = AtomicWaker::new();
static BLE_DFU_REQUESTED: AtomicBool = AtomicBool::new(false);

pub use ferrix_core::command::{ContactOperation, CONTACT_OP_CHANNEL};

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
                ferrix_core::state::DISCOVERED_CONTACTS.lock(|c| {
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
                        let _ = list.push(ferrix_core::types::DiscoveredContact {
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
                            saved_hash: 0,
                            sync_since: 0,
                        });
                    }
                });
            }
            ContactOperation::ResetPath { pub_key } => {
                ferrix_core::state::DISCOVERED_CONTACTS.lock(|c| {
                    if let Some(ct) = c.borrow_mut().iter_mut().find(|ct| ct.pub_key == pub_key) {
                        ct.out_path = [0u8; 64];
                        ct.out_path_len = 0xFF;
                    }
                });
            }
            ContactOperation::Remove { pub_key } => {
                let removed = ferrix_core::state::DISCOVERED_CONTACTS.lock(|c| {
                    let mut list = c.borrow_mut();
                    if let Some(pos) = list.iter().position(|ct| ct.pub_key == pub_key) {
                        list.remove(pos);
                        true
                    } else {
                        false
                    }
                });
                if removed {
                    ferrix_core::storage::adverts::forget(&pub_key);
                    ferrix_core::storage::contacts::queue_remove(pub_key).await;
                }
            }
        }
        // Persist a structural change. The storage task's hash check means
        // volatile-only updates do not write flash.
        ferrix_core::storage::request_contacts();
    }
}

/// Stable storage for the SoftDevice's static passkey.
static PASSKEY_BUF: Mutex<CriticalSectionRawMutex, RefCell<[u8; 6]>> =
    Mutex::new(RefCell::new([b'0'; 6]));

/// Program the SoftDevice's fixed 6-digit pairing passkey.
fn set_static_passkey(pin: u32) {
    PASSKEY_BUF.lock(|cell| {
        let mut buf = cell.borrow_mut();
        let mut value = pin % 1_000_000;
        for i in (0..6).rev() {
            buf[i] = b'0' + (value % 10) as u8;
            value /= 10;
        }
        let ret = crate::sd_ffi::ble_set_static_passkey(&buf);
        if ret != 0 {
            error!("BLE: static passkey set failed {=u32:#x}", ret);
        }
    });
}

pub struct MeshCoreSecurityHandler;

impl SecurityHandler for MeshCoreSecurityHandler {
    fn io_capabilities(&self) -> IoCapabilities {
        IoCapabilities::DisplayOnly
    }

    fn can_bond(&self, _conn: &Connection) -> bool {
        true
    }

    fn on_bonded(
        &self,
        _conn: &Connection,
        master_id: MasterId,
        key: EncryptionInfo,
        peer_id: IdentityKey,
    ) {
        bonds::upsert(Bond {
            master_ediv: master_id.ediv,
            master_rand: master_id.rand,
            ltk: key.ltk,
            ltk_flags: key.flags,
            peer_addr_flags: peer_id.addr.flags,
            peer_addr: peer_id.addr.bytes,
            peer_irk: peer_id.irk.as_raw().irk,
            sys_len: 0,
            sys_attrs: [0u8; 64],
        });
    }

    fn get_key(&self, _conn: &Connection, master_id: MasterId) -> Option<EncryptionInfo> {
        let bond = bonds::get_key(master_id.ediv, &master_id.rand)?;
        Some(EncryptionInfo {
            ltk: bond.ltk,
            flags: bond.ltk_flags,
        })
    }

    fn save_sys_attrs(&self, conn: &Connection) {
        let mut buf = [0u8; 64];
        if let Ok(len) = gatt_server::get_sys_attrs(conn, &mut buf) {
            let addr = conn.peer_address();
            bonds::set_sys_attrs(addr.flags, addr.bytes, &buf[..len]);
        }
    }

    fn load_sys_attrs(&self, conn: &Connection) {
        let addr = conn.peer_address();
        let attrs = bonds::sys_attrs(addr.flags, addr.bytes);
        let slice = attrs.as_ref().map(|(buf, len)| &buf[..*len]);
        if gatt_server::set_sys_attrs(conn, slice).is_err() {
            debug!("BLE: set sys attrs failed");
        }
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

/// Outbound BLE frames: command responses and asynchronous pushes.
pub static BLE_PUSH_CHANNEL: Channel<CriticalSectionRawMutex, Vec<u8, 256>, 24> = Channel::new();

/// Push sink registered with the core router.
pub struct BlePushSink;

pub static BLE_PUSH_SINK: BlePushSink = BlePushSink;

impl PushSink for BlePushSink {
    fn is_connected(&self) -> bool {
        BLE_CONNECTED.load(Ordering::Relaxed)
    }

    fn try_push(&self, frame: &[u8]) -> bool {
        if frame.len() > 256 {
            return false;
        }
        let mut vec = Vec::<u8, 256>::new();
        if vec.extend_from_slice(frame).is_err() {
            return false;
        }
        BLE_PUSH_CHANNEL.try_send(vec).is_ok()
    }
}

/// Companion link backed by the BLE notification queue.
struct BleLink;

impl CompanionLink for BleLink {
    async fn write_frame(&mut self, frame: &[u8]) {
        let mut vec = Vec::<u8, 256>::new();
        let _ = vec.extend_from_slice(&frame[..frame.len().min(247)]);
        let _ = BLE_PUSH_CHANNEL.send(vec).await;
    }

    async fn write_text(&mut self, text: &[u8]) {
        let mut vec = Vec::<u8, 256>::new();
        let _ = vec.extend_from_slice(&text[..text.len().min(247)]);
        let _ = BLE_PUSH_CHANNEL.send(vec).await;
    }
}

const FAST_ADV_SECS: u64 = 30;
/// 1 second in 625 us advertising units.
const ADVERTISING_SLOW_INTERVAL: u32 = 1600;
/// Keep low-latency connection events during the initial sync burst, then
/// restore the controller's slave-latency setting.
const SLAVE_LATENCY_RESTORE_SECS: u64 = 15;

#[embassy_executor::task]
pub async fn ble_companion_task(
    sd: &'static Softdevice,
    server: MeshCoreGattServer,
    public_key: [u8; 32],
    private_key: [u8; 64],
    hw_model: &'static str,
    capabilities: u32,
    model_name: &'static str,
    host: &'static dyn CliHost,
) {
    info!("Starting MeshCore BLE companion advertising...");

    let callsign = ferrix_core::storage::get_callsign();
    let mut session = CompanionSession::new(
        callsign.as_str(),
        hw_model,
        capabilities,
        public_key,
        private_key,
        host,
    );

    // Use fast advertising (20ms) for the first 30 seconds for discovery,
    // then fall back to 1s even if no app ever connects. Interval is in
    // 625 us units: 20ms = 32, 1s = 1600.
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

    // Flags: LE General Discoverable Mode, BR/EDR Not Supported
    adv_data[0] = 2; // length
    adv_data[1] = 0x01; // Flags
    adv_data[2] = 0x06;

    // Complete 128-bit Service UUID list
    adv_data[3] = 17; // 1 (type) + 16 (uuid)
    adv_data[4] = 0x07; // Complete list of 128-bit Service UUIDs
    adv_data[5..21].copy_from_slice(&nus_uuid_128);

    let mut model = Vec::<u8, 32>::new();
    let _ = model.extend_from_slice(model_name.as_bytes());
    let _ = server.dis.model_number_set(&model);

    let mut fw = Vec::<u8, 32>::new();
    let rev = ferrix_core::version::BLE_FW_REVISION.as_bytes();
    let _ = fw.extend_from_slice(&rev[..rev.len().min(32)]);
    let _ = server.dis.firmware_revision_set(&fw);

    let mut mfg = Vec::<u8, 32>::new();
    let _ = mfg.extend_from_slice(b"Elecrow");
    let _ = server.dis.manufacturer_name_set(&mfg);

    let mut dfu_rev = Vec::<u8, 2>::new();
    // DFU_REV_APPMODE (0x0001, little-endian) signals Nordic/Adafruit DFU support.
    let _ = dfu_rev.extend_from_slice(&[0x01, 0x00]);
    let _ = server.dfu.revision_set(&dfu_rev);

    let (mut cmd_tx, mut cmd_rx) = BLE_CMD_QUEUE.init(Queue::new()).split();

    loop {
        info!("BLE: Waiting for pairable connection from MeshCore companion app...");
        // Pick up a PIN or callsign changed since the last advertising cycle.
        set_static_passkey(BLE_PIN.load(Ordering::Relaxed));

        const NAME_PREFIX: &[u8] = b"MeshCore-";
        const BLE_NAME_MAX: usize = 29;
        let callsign = ferrix_core::storage::get_callsign();
        let mut name_buf = [0u8; BLE_NAME_MAX];
        let callsign_bytes = callsign.as_bytes();
        let copy_len = callsign_bytes.len().min(BLE_NAME_MAX - NAME_PREFIX.len());
        name_buf[..NAME_PREFIX.len()].copy_from_slice(NAME_PREFIX);
        name_buf[NAME_PREFIX.len()..NAME_PREFIX.len() + copy_len]
            .copy_from_slice(&callsign_bytes[..copy_len]);
        let name = &name_buf[..NAME_PREFIX.len() + copy_len];

        let mut scan_data = [0u8; 31];
        scan_data[0] = (name.len() + 1) as u8;
        scan_data[1] = 0x09; // Complete Local Name
        scan_data[2..2 + name.len()].copy_from_slice(name);

        let adv = peripheral::ConnectableAdvertisement::ScannableUndirected {
            adv_data: &adv_data[..21],
            scan_data: &scan_data[..2 + name.len()],
        };
        let advertise_fut =
            peripheral::advertise_pairable(sd, adv, &adv_config, &SECURITY_HANDLER);
        let fast_window = if adv_config.interval == 32 {
            embassy_time::Timer::after_secs(FAST_ADV_SECS)
        } else {
            // Already on the slow interval; this is just a long no-op window.
            embassy_time::Timer::after_secs(3600)
        };
        let conn = match embassy_futures::select::select(advertise_fut, fast_window).await {
            embassy_futures::select::Either::First(Ok(mut c)) => {
                info!("BLE: Phone connected & pairing!");
                BLE_CONNECTED.store(true, Ordering::Relaxed);
                // Request maximum BLE data length (251 bytes) instead of the default 27 bytes.
                // This increases throughput for channel sync and other large responses.
                let _ = c.data_length_update(None);
                // Disable slave latency during sync so the device wakes at every connection event.
                let _ = c.ignore_slave_latency(true);
                if let Err(e) = c.request_security() {
                    debug!("BLE: security request failed: {:?}", e);
                }
                c
            }
            embassy_futures::select::Either::First(Err(e)) => {
                error!("BLE: Advertising error: {:?}", e);
                embassy_time::Timer::after_secs(1).await;
                continue;
            }
            embassy_futures::select::Either::Second(_) => {
                if adv_config.interval == 32 {
                    adv_config.interval = ADVERTISING_SLOW_INTERVAL;
                    info!("BLE: fast advertising window expired; using 1s interval");
                }
                continue;
            }
        };

        BLE_DFU_REQUESTED.store(false, Ordering::Relaxed);
        session.reset();
        while BLE_PUSH_CHANNEL.try_receive().is_ok() {}

        // Keep a second handle so slave latency can be restored while the
        // session futures still borrow `conn`.
        let mut latency_conn = conn.clone();

        let session_result = {
            // The GATT callback only copies the command into the lock-free SPSC
            // queue and wakes `command_fut`. It must not parse commands, take
            // locks, allocate or log: `handle_command` can emit dozens of
            // responses (contact sync) and doing that inside a SoftDevice
            // callback can stall the SoftDevice.
            let gatt_fut = gatt_server::run(&conn, &server, |event| match event {
                MeshCoreGattServerEvent::Nus(NordicUartServiceEvent::RxWrite(data)) => {
                    let pushed = CmdSlot::from_slice(data.as_slice())
                        .and_then(|slot| cmd_tx.enqueue(slot).ok())
                        .is_some();
                    if !pushed {
                        // Queue full: drop rather than spin inside the callback.
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
                MeshCoreGattServerEvent::Dfu(LegacyDfuServiceEvent::ControlPointWrite(data)) => {
                    // Opcode 0x01: Start DFU (optional second byte selects image type).
                    if data.first() == Some(&0x01) {
                        BLE_DFU_REQUESTED.store(true, Ordering::Release);
                        BLE_CMD_WAKER.wake();
                    }
                }
                _ => {}
            });

            // Companion protocol processing, running in normal task context. One
            // command at a time, then yield so `push_fut` can drain the responses
            // before the next command is handled.
            let command_fut = async {
                loop {
                    if BLE_DFU_REQUESTED.swap(false, Ordering::AcqRel) {
                        let _ = ferrix_core::command::USB_COMMAND_CHANNEL
                            .try_send(ferrix_core::command::UsbCommand::RebootToOtaDfu);
                    } else if let Some(slot) = cmd_rx.dequeue() {
                        let drops = BLE_CMD_DROPS.swap(0, Ordering::Relaxed);
                        if drops != 0 {
                            debug!("BLE: dropped {} command(s) because the ring was full", drops);
                        }
                        session
                            .handle_frame(&slot.data[..slot.len as usize], &mut BleLink)
                            .await;
                        embassy_futures::yield_now().await;
                    } else {
                        // Register before the emptiness check to avoid a lost wake.
                        poll_fn(|cx| {
                            BLE_CMD_WAKER.register(cx.waker());
                            if !cmd_rx.ready() && !BLE_DFU_REQUESTED.load(Ordering::Acquire) {
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
                    let msg = BLE_PUSH_CHANNEL.receive().await;
                    let mut vec = Vec::<u8, 256>::new();
                    let copy_len = msg.len().min(247);
                    let _ = vec.extend_from_slice(&msg[..copy_len]);
                    // The SoftDevice HVX queue can be momentarily full during a
                    // contact sync burst. Retry briefly instead of dropping.
                    for attempt in 0..40 {
                        match server.nus.tx_notify(&conn, &vec) {
                            Ok(()) => break,
                            Err(e) => {
                                if attempt == 39 {
                                    debug!("BLE NUS tx_notify push failed: {:?}", e);
                                } else {
                                    embassy_time::Timer::after_millis(5).await;
                                }
                            }
                        }
                    }
                }
            };

            let mut session_fut = core::pin::pin!(embassy_futures::select::select3(
                gatt_fut,
                command_fut,
                push_fut
            ));
            let mut latency_timer =
                embassy_time::Timer::after_secs(SLAVE_LATENCY_RESTORE_SECS);
            let mut latency_restored = false;
            loop {
                match embassy_futures::select::select(session_fut.as_mut(), &mut latency_timer)
                    .await
                {
                    embassy_futures::select::Either::First(result) => break result,
                    embassy_futures::select::Either::Second(_) => {
                        if !latency_restored {
                            let _ = latency_conn.ignore_slave_latency(false);
                            latency_restored = true;
                            info!("BLE: restored slave latency after sync window");
                        }
                        latency_timer = embassy_time::Timer::after_secs(3600);
                    }
                }
            }
        };

        match session_result {
            embassy_futures::select::Either3::First(e) => {
                info!("BLE: Phone disconnected ({:?}), resuming advertising...", e);
                BLE_CONNECTED.store(false, Ordering::Relaxed);
                // Drop anything left over from the closed connection so the
                // next app session never receives stale responses.
                while cmd_rx.dequeue().is_some() {}
                while BLE_PUSH_CHANNEL.try_receive().is_ok() {}
                // After a connection, use the 1s interval for power saving.
                adv_config.interval = ADVERTISING_SLOW_INTERVAL;
                info!("BLE: resumed advertising at 1s interval");
            }
            embassy_futures::select::Either3::Second(_) => {}
            embassy_futures::select::Either3::Third(_) => {}
        }
    }
}

pub use ferrix_core::state::{BLE_CONNECTED, BLE_PIN, BLE_STATUS};

/// Counts commands dropped because the GATT command ring was full.
static BLE_CMD_DROPS: AtomicU32 = AtomicU32::new(0);

#[embassy_executor::task]
pub async fn ble_companion_supervisor_task(
    spawner: embassy_executor::Spawner,
    public_key: [u8; 32],
    private_key: [u8; 64],
    hw_model: &'static str,
    capabilities: u32,
    model_name: &'static str,
    host: &'static dyn CliHost,
    fault_led: Option<crate::fault::FaultLed>,
    storage_spawn: fn(embassy_executor::Spawner, &'static Softdevice),
) {
    crate::fault::set_fault_led(fault_led);
    BLE_STATUS.store(1, Ordering::Relaxed);
    info!("BLE: Initializing SoftDevice S140 subsystem...");

    let clock_cfg = match crate::startup::lfclk_config() {
        crate::startup::Lfclk::InternalRc => nrf_softdevice::raw::nrf_clock_lf_cfg_t {
            source: nrf_softdevice::raw::NRF_CLOCK_LF_SRC_RC as u8,
            rc_ctiv: 16,
            rc_temp_ctiv: 2,
            accuracy: nrf_softdevice::raw::NRF_CLOCK_LF_ACCURACY_500_PPM as u8,
        },
        crate::startup::Lfclk::ExternalXtal(acc) => {
            let accuracy = match acc {
                crate::startup::LfclkAccuracy::Ppm20 => {
                    nrf_softdevice::raw::NRF_CLOCK_LF_ACCURACY_20_PPM
                }
                crate::startup::LfclkAccuracy::Ppm50 => {
                    nrf_softdevice::raw::NRF_CLOCK_LF_ACCURACY_50_PPM
                }
                crate::startup::LfclkAccuracy::Ppm100 => {
                    nrf_softdevice::raw::NRF_CLOCK_LF_ACCURACY_100_PPM
                }
                crate::startup::LfclkAccuracy::Ppm250 => {
                    nrf_softdevice::raw::NRF_CLOCK_LF_ACCURACY_250_PPM
                }
            } as u8;
            nrf_softdevice::raw::nrf_clock_lf_cfg_t {
                source: nrf_softdevice::raw::NRF_CLOCK_LF_SRC_XTAL as u8,
                rc_ctiv: 0,
                rc_temp_ctiv: 0,
                accuracy,
            }
        }
    };

    let sd_config = nrf_softdevice::Config {
        clock: Some(clock_cfg),
        common_vs_uuid: Some(nrf_softdevice::raw::ble_common_cfg_vs_uuid_t {
            vs_uuid_count: 3,
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
            attr_tab_size: 2048,
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
    let ret = crate::sd_ffi::softdevice_probe_enable(sd_config.clock.as_ref());
    if ret != 0 {
        error!("BLE: sd_softdevice_enable returned status code {=u32:#x}", ret);
        BLE_STATUS.store(0x1000 | ret, Ordering::Relaxed);
        return;
    }

    info!("BLE: SoftDevice S140 successfully enabled!");
    BLE_STATUS.store(2, Ordering::Relaxed);

    // Disable raw SD so that embassy's Softdevice::enable can manage state cleanly
    crate::sd_ffi::softdevice_disable();

    let sd = Softdevice::enable(&sd_config);
    if crate::startup::dcdc_reg1_enabled() {
        let ret = crate::sd_ffi::power_dcdc_enable();
        crate::startup::record_sd_dcdc_result(ret);
        if ret != 0 {
            warn!("BLE: sd_power_dcdc_mode_set returned {=u32:#x}", ret);
        } else {
            info!("BLE: REG1 DC-DC regulator enabled");
        }
    }
    set_static_passkey(BLE_PIN.load(Ordering::Relaxed));

    let server = MeshCoreGattServer::new(sd);

    // Storage must run even if GATT/BLE setup fails, otherwise the watchdog
    // never sees STORAGE_HEARTBEAT and the node bootloops.
    storage_spawn(spawner, &*sd);

    if let Ok(server) = server {
        if spawner.spawn(softdevice_task(&*sd)).is_ok()
            && spawner
                .spawn(ble_companion_task(
                    &*sd,
                    server,
                    public_key,
                    private_key,
                    hw_model,
                    capabilities,
                    model_name,
                    host,
                ))
                .is_ok()
        {
            info!("MeshCore BLE companion service successfully started!");
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
    hw_model: &'static str,
    capabilities: u32,
    model_name: &'static str,
    host: &'static dyn CliHost,
    fault_led: Option<crate::fault::FaultLed>,
    storage_spawn: fn(embassy_executor::Spawner, &'static Softdevice),
) -> bool {
    spawner
        .spawn(ble_companion_supervisor_task(
            spawner,
            public_key,
            private_key,
            hw_model,
            capabilities,
            model_name,
            host,
            fault_led,
            storage_spawn,
        ))
        .is_ok()
}
