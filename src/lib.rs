#![no_std]
#![doc = include_str!("../docs/getting-started.md")]
#![doc = include_str!("../docs/conventions.md")]
#![doc = include_str!("../docs/api.md")]
#![doc = include_str!("../docs/development.md")]

mod constraint;
mod space;
mod state;

pub use constraint::*;
pub use space::*;
pub use state::*;
