/*!
 * Synchronized IMU reader for BNO055 and Hiwonder sensors.
 * 
 * This module provides functionality to read and synchronize data
 * from both the BNO055 and Hiwonder IMU sensors.
 */
 
// Re-export dependencies to make them available to users of this crate
pub use linux_bno055;
pub use hiwonder;
pub use nalgebra; 