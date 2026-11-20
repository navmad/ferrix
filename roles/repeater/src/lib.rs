//! MeshCore repeater role (placeholder).
//!
//! This crate reserves the role boundary in the workspace. It implements no
//! behavior, no firmware image depends on it, and there is no repeater binary
//! yet.
//!
//! When the repeater is started it gets its own board-facing trait rather than
//! reusing `ferrix-role-companion`'s `CompanionBoard`: a repeater has different
//! transport, advertisement and forwarding requirements, and must not inherit
//! companion protocol assumptions.

#![no_std]
#![forbid(unsafe_code)]

/// Placeholder for the repeater role.
///
/// A zero-sized marker keeps the package concrete while the role behavior is
/// still undefined. It will be replaced by a `run(...)` entry point once the
/// repeater's transport, advertisement and forwarding policy are specified.
#[derive(Clone, Copy, Debug, Default)]
pub struct RepeaterRole;

impl RepeaterRole {
    /// Create the placeholder role.
    pub const fn new() -> Self {
        Self
    }
}
