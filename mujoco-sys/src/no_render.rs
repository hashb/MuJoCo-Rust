#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]
#![allow(clippy::approx_constant)]

include!(concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/generated/no_render.rs"
));
