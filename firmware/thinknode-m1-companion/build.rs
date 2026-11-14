use std::env;

fn main() {
    // The board crate owns memory.x so a new role for the same board does not
    // duplicate the flash/RAM layout.
    let manifest = env::var("CARGO_MANIFEST_DIR").expect("CARGO_MANIFEST_DIR");
    let board = format!("{manifest}/../../boards/thinknode-m1");
    println!("cargo:rustc-link-search={board}");
    println!("cargo:rerun-if-changed={board}/memory.x");
}
