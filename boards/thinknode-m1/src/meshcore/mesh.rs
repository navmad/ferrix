//! Compatibility shim for the routing engine now owned by `ferrix-core`.
//!
//! Keep the old `crate::meshcore::mesh::*` paths working while call sites
//! migrate to `ferrix_core::engine` and `ferrix_core::types`.

pub use ferrix_core::engine::{InboundAction, MeshEngine};
pub use ferrix_core::types::{DiscoveredContact, ReceivedChannelData, ReceivedMessage};
