//! MeshCore Bluetooth Low Energy (BLE) Companion Subsystem
//!
//! Exposes Nordic UART Service (NUS) over BLE GATT, enabling official MeshCore
//! mobile apps (iOS & Android) to discover, pair, and communicate with ThinkNode M1.

use core::sync::atomic::{AtomicBool, Ordering};

use defmt::{debug, error, info};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Channel;

/// Shared raw mutex type — see usb.rs for documentation.
type SharedRawMutex = ThreadModeRawMutex;
use heapless::{String, Vec};
use nrf_softdevice::ble::gatt_server;
use nrf_softdevice::ble::peripheral;
use nrf_softdevice::ble::security::{IoCapabilities, SecurityHandler};
use nrf_softdevice::ble::{Connection, PasskeyReply};
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

pub static COMPANION_PUSH_CHANNEL: Channel<SharedRawMutex, Vec<u8, 256>, 16> = Channel::new();

/// Contact operation deferred from the GATT callback to avoid locking
/// `DISCOVERED_CONTACTS` inside the SoftDevice critical section.
pub enum ContactOperation {
    AddOrUpdate {
        pub_key: [u8; 32],
        callsign: String<32>,
        node_id: [u8; 4],
        last_seen_epoch: u32,
        latitude: i32,
        longitude: i32,
        flags: u8,
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

pub struct MeshCoreSecurityHandler;

impl SecurityHandler for MeshCoreSecurityHandler {
    fn io_capabilities(&self) -> IoCapabilities {
        IoCapabilities::DisplayOnly
    }

    fn can_bond(&self, _conn: &Connection) -> bool {
        true
    }

    fn request_mitm_protection(&self, _conn: &Connection) -> bool {
        true
    }

    fn display_passkey(&self, passkey: &[u8; 6]) {
        if let Ok(s) = core::str::from_utf8(passkey) {
            info!("BLE: Passkey displayed: {}", s);
        }
    }

    fn enter_passkey(&self, reply: PasskeyReply) {
        let pin = BLE_PIN.load(Ordering::Relaxed);
        let mut buf = [b'0'; 6];
        let mut value = pin;
        for i in (0..6).rev() {
            buf[i] = b'0' + (value % 10) as u8;
            value /= 10;
        }
        info!("BLE: Replying with stored PIN");
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
    let name = b"MeshCore-M1";
    scan_data[0] = (name.len() + 1) as u8;
    scan_data[1] = 0x09; // Complete Local Name
    scan_data[2..2 + name.len()].copy_from_slice(name);

    let adv = peripheral::ConnectableAdvertisement::ScannableUndirected {
        adv_data: &adv_data[..21],
        scan_data: &scan_data[..2 + name.len()],
    };

    let mut model = Vec::<u8, 32>::new();
    let _ = model.extend_from_slice(b"ThinkNode M1");
    let _ = server.dis.model_number_set(&model);

    let mut fw = Vec::<u8, 32>::new();
    let _ = fw.extend_from_slice(b"0.1.0-embassy");
    let _ = server.dis.firmware_revision_set(&fw);

    let mut mfg = Vec::<u8, 32>::new();
    let _ = mfg.extend_from_slice(b"Elecrow");
    let _ = server.dis.manufacturer_name_set(&mfg);

    loop {
        info!("BLE: Waiting for pairable connection from MeshCore companion app (PIN 123456)...");
        // Flag to signal when channel sync is complete, so we can defer connection
        // parameter updates until after the sync burst (matching the reference firmware behavior).
        let sync_done = AtomicBool::new(false);

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

        let gatt_fut = gatt_server::run(&conn, &server, |event| match event {
            MeshCoreGattServerEvent::Nus(NordicUartServiceEvent::RxWrite(data)) => {
                info!("BLE command received ({} bytes): opcode=0x{:02x}", data.len(), data.first().copied().unwrap_or(0));
                node.handle_command(data.as_slice(), |resp| {
                    let mut vec = Vec::<u8, 256>::new();
                    let copy_len = resp.len().min(247);
                    let _ = vec.extend_from_slice(&resp[..copy_len]);
                    let _ = server.nus.tx_notify(&conn, &vec);
                });
                // After handling a command, check if all 8 channels have been synced.
                // If so, signal the background task to update connection parameters.
                if node.channels_synced >= 8 && !sync_done.load(Ordering::Relaxed) {
                    sync_done.store(true, Ordering::Relaxed);
                    info!("BLE: Channel sync complete, will update connection params");
                }
            }
            MeshCoreGattServerEvent::Nus(NordicUartServiceEvent::TxCccdWrite { notifications }) => {
                info!("BLE NUS notifications configured: enabled={}", notifications);
            }
            _ => {}
        });

        let push_fut = async {
            loop {
                let msg = COMPANION_PUSH_CHANNEL.receive().await;
                let mut vec = Vec::<u8, 256>::new();
                let copy_len = msg.len().min(247);
                let _ = vec.extend_from_slice(&msg[..copy_len]);
                if let Err(e) = server.nus.tx_notify(&conn, &vec) {
                    debug!("BLE NUS tx_notify push failed: {:?}", e);
                } else {
                    info!("BLE NUS push sent to phone: code=0x{:02x}, len={}", vec.first().copied().unwrap_or(0), vec.len());
                }
            }
        };

        // Background task that waits for channel sync to complete, then requests fast
        // connection parameters (15ms interval) for faster post-sync interaction.
        // This matches the reference firmware's deferred connection parameter update pattern.
        let conn_params_fut = async {
            while !sync_done.load(Ordering::Relaxed) {
                embassy_time::Timer::after_millis(100).await;
            }
            // Small delay to ensure the last response has been sent before changing params.
            embassy_time::Timer::after_millis(200).await;
            info!("BLE: Requesting fast connection interval (15ms) after sync");
            let _ = conn.set_conn_params(nrf_softdevice::raw::ble_gap_conn_params_t {
                min_conn_interval: 12,  // 15 ms (12 * 1.25ms) — minimum iOS accepts
                max_conn_interval: 12,  // 15 ms
                slave_latency: 0,       // connect every interval
                conn_sup_timeout: 40,   // 4000 ms (40 * 100ms)
            });
        };

        match embassy_futures::select::select3(gatt_fut, push_fut, conn_params_fut).await {
            embassy_futures::select::Either3::First(e) => {
                info!("BLE: Phone disconnected ({:?}), resuming advertising...", e);
                // Switch to slow advertising (211ms) after the first connection to save power.
                if adv_config.interval == 32 {
                    adv_config.interval = 338; // 211ms slow advertising
                    info!("BLE: Switched to slow advertising (211ms)");
                }
            }
            embassy_futures::select::Either3::Second(_) => {}
            embassy_futures::select::Either3::Third(_) => {
                // Connection params task completed; this shouldn't happen since it's a one-shot
                // that completes after set_conn_params, but the select3 requires handling all arms.
            }
        }
    }
}

use core::sync::atomic::AtomicU32;

pub static BLE_STATUS: AtomicU32 = AtomicU32::new(0);
/// Pairing PIN reported to the app and used by the security handler.
pub static BLE_PIN: AtomicU32 = AtomicU32::new(123456);

unsafe extern "C" fn sd_fault_handler(id: u32, pc: u32, info: u32) {
    defmt::error!("Softdevice fault: id={=u32:#x} pc={=u32:#x} info={=u32:#x}", id, pc, info);
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
) -> bool {
    spawner.spawn(ble_companion_supervisor_task(spawner, public_key, private_key)).is_ok()
}
