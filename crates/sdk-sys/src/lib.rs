//! Bindgen bindings for the unmodified official SCS SDK 1.15 (x64).
//! Regenerate using `LIBCLANG_PATH=... cargo build -p etsads-sdk-sys --features regenerate`.
#![allow(
    non_camel_case_types,
    non_upper_case_globals,
    non_snake_case,
    clippy::all
)]
include!("bindings.rs");
