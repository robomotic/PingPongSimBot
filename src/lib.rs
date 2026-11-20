//! ElRobot ping-pong paddle balancer: direct ST3215 bus access, config, control, kinematics, vision, the balancing
//! engine, a simulator and run analysis. The `paddle-balance` binary (src/main.rs) uses these.

pub mod analysis;
pub mod arm;
pub mod balance;
pub mod calib;
pub mod bus;
pub mod config;
pub mod control;
pub mod estimator;
pub mod kinematics;
pub mod sim;
pub mod throw;
pub mod vision;
