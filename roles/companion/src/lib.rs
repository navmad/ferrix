//! MeshCore companion role.
//!
//! The role owns the companion startup sequence: identity, routing engine,
//! transports, companion node and the application loop. It never touches pins,
//! chip drivers or firmware images; the board supplies those through
//! [`CompanionBoard`].

#![no_std]
#![allow(async_fn_in_trait)]

use embassy_executor::Spawner;
use heapless::String;

use ferrix_core::command::COMPANION_PUSH_CHANNEL;
use ferrix_core::companion::cli::CliHost;
use ferrix_core::companion::protocol::{next_delayed_push, CompanionHost, CompanionNode};
use ferrix_core::engine::EngineContext;
use ferrix_hal::radio::LoraRadio;
use ferrix_runtime::app::AppBoard;
use meshcore_wire::LocalIdentity;

/// The hardware and board services the companion role needs.
#[allow(async_fn_in_trait)]
pub trait CompanionBoard {
    /// Board-side application loop implementation.
    type App: AppBoard;
    /// Physical radio exposed to the runtime scheduler.
    type Radio: LoraRadio;

    /// Hardware model string reported over the protocol.
    fn name(&self) -> &'static str;
    /// Protocol capability bits.
    fn capabilities(&self) -> u32;
    /// USB vendor and product IDs.
    fn usb_ids(&self) -> (u16, u16);
    /// USB manufacturer string.
    fn usb_manufacturer(&self) -> &'static str;
    /// USB product string.
    fn usb_product(&self) -> &'static str;
    /// BLE model name.
    fn ble_model(&self) -> &'static str;
    /// Companion persistence and telemetry interface.
    fn companion_host(&self) -> &'static dyn CompanionHost;
    /// CLI host interface (also a `CompanionHost`).
    fn cli_host(&self) -> &'static dyn CliHost;
    /// Routing engine persistence and radio bindings.
    fn engine_context(&self) -> &'static dyn EngineContext;
    /// Fallback identity seed when no private key is stored.
    fn hardware_seed(&self) -> [u8; 32];
    /// Whether the routing engine should forward traffic.
    fn forwarding_enabled(&self) -> bool;

    /// Boot board hardware and storage.
    ///
    /// Returns the configured callsign and an optional stored 64-byte private
    /// key. Implementations spawn the tasks that must run before the main loop
    /// (watchdog, GNSS, input, storage snapshot load).
    async fn init(&mut self, spawner: Spawner) -> (String<32>, Option<[u8; 64]>);

    /// Publish the resolved identity to shared transports and protocol state.
    fn publish_identity(&mut self, public_key: [u8; 32], private_key: [u8; 64]);

    /// Start USB, BLE and other transports.
    fn start_transports(
        &mut self,
        spawner: Spawner,
        public_key: [u8; 32],
        private_key: [u8; 64],
    );

    /// Draw the boot splash and first dashboard frame.
    async fn start_ui(&mut self, node: &CompanionNode<'_>);

    /// Start the radio task and prepare the radio handle.
    fn start_radio(&mut self, spawner: Spawner);

    /// Take the app implementation for the main loop.
    fn take_app(&mut self) -> Self::App;

    /// Take the radio handle for the main loop.
    fn take_radio(&mut self) -> Self::Radio;
}

/// Forward deferred protocol pushes to the connected companion transport.
#[embassy_executor::task]
async fn delayed_push_task() -> ! {
    loop {
        let frame = next_delayed_push().await;
        let _ = COMPANION_PUSH_CHANNEL.try_send(frame);
    }
}

/// Run the companion role until the executor stops.
pub async fn run<B: CompanionBoard>(mut board: B, spawner: Spawner) -> ! {
    let (node_name, custom_key) = board.init(spawner).await;

    let local_id = if let Some(key) = custom_key {
        LocalIdentity::from_private(node_name.as_str(), &key).unwrap_or_else(|| {
            defmt::info!("Stored private key is reserved or invalid; using hardware seed");
            LocalIdentity::from_seed(node_name.as_str(), &board.hardware_seed())
        })
    } else {
        LocalIdentity::from_seed(node_name.as_str(), &board.hardware_seed())
    };

    let public_key = local_id.pub_key_bytes();
    let private_key = local_id.private_key_bytes();
    board.publish_identity(public_key, private_key);

    let mut mesh = ferrix_core::engine::MeshEngine::new(
        local_id,
        false,
        board.engine_context(),
    );
    mesh.is_repeater = board.forwarding_enabled();

    spawner.spawn(delayed_push_task()).unwrap();
    board.start_transports(spawner, public_key, private_key);

    let mut node = CompanionNode::new(
        node_name.as_str(),
        board.name(),
        board.capabilities(),
        public_key,
        private_key,
        board.companion_host(),
    );

    board.start_ui(&node).await;
    board.start_radio(spawner);

    let mut app = board.take_app();
    let mut radio = board.take_radio();
    ferrix_runtime::app::run(&mut app, &mut mesh, &mut node, &mut radio).await
}
