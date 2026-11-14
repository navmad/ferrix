# Local patches

## defmt + `_test` compatibility

`sequential-storage` 8.0.2 derives `defmt::Format` for its `Error<S>` enum.
Under the `_test` feature the enum also carries `std::backtrace::Backtrace`
fields, and that type does not implement `defmt::Format`, so enabling both
`defmt` and `_test` fails to compile.

`ferrix-core` enables `defmt` for storage error logs, while
`meshcore-wire`'s tests enable `_test` for the mock flash. Replace the derive
with a manual `defmt::Format` impl that formats every field except the
test-only backtraces (`src/lib.rs`).

This patch can be dropped when upstream fixes the derive or adds the
backtrace behind a separate feature.
