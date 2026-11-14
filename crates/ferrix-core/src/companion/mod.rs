//! Companion protocol: transport framing, contact encoding, command handling
//! and the text CLI. Board services are reached through
//! [`protocol::CompanionHost`] and [`cli::CliHost`].

pub mod cli;
pub mod contacts;
pub mod protocol;
pub mod framing;
