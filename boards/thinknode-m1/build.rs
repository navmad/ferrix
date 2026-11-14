use std::env;

fn main() {
    // cortex-m-rt's link.x includes `memory.x`; point the linker at this
    // board crate so each board can carry its own flash/RAM layout.
    let manifest = env::var("CARGO_MANIFEST_DIR").expect("CARGO_MANIFEST_DIR");
    println!("cargo:rustc-link-search={manifest}");
    println!("cargo:rerun-if-changed=memory.x");
}
