//! ElRobot ping-pong paddle balancer: direct ST3215 bus access, config, control, kinematics and vision.
//! The `paddle-balance` binary (src/main.rs) uses these.

pub mod bus;
pub mod config;
pub mod control;
pub mod estimator;
pub mod kinematics;
pub mod vision;
