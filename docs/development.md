# Development

Run checks from the repository root:

```sh
cargo test --locked
cargo fmt --check
cargo clippy --all-targets --locked -- -D warnings
```

`cargo test` also compiles and runs the example in `getting-started.md`.
To run only the documentation tests, use `cargo test --doc --locked`.

## Rust documentation

```sh
cargo doc --no-deps --open
```

The guide pages are included in the crate documentation alongside its types
and methods.

## Embedded targets

To check a target without the standard library:

```sh
rustup target add thumbv7em-none-eabihf
cargo check --lib --locked --target thumbv7em-none-eabihf
```
