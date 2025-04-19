use pyo3::exceptions::PyRuntimeError;
use pyo3::prelude::*;
use pyo3::types::PyDict;

use imu::{ImuReader, Quaternion, Vector3};

// Wrap the Vector3 struct
#[pyclass(name = "Vector3")]
#[derive(Clone)]
struct PyVector3 {
    #[pyo3(get, set)]
    x: f32,
    #[pyo3(get, set)]
    y: f32,
    #[pyo3(get, set)]
    z: f32,
}

#[pymethods]
impl PyVector3 {
    #[new]
    fn new(x: f32, y: f32, z: f32) -> Self {
        PyVector3 { x, y, z }
    }

    fn __repr__(&self) -> PyResult<String> {
        Ok(format!("Vector3(x={}, y={}, z={})", self.x, self.y, self.z))
    }
}

// Convert between Rust and Python types
impl From<Vector3> for PyVector3 {
    fn from(v: Vector3) -> Self {
        PyVector3 {
            x: v.x,
            y: v.y,
            z: v.z,
        }
    }
}

impl From<PyVector3> for Vector3 {
    fn from(v: PyVector3) -> Self {
        Vector3 {
            x: v.x,
            y: v.y,
            z: v.z,
        }
    }
}

// Wrap the Quaternion struct
#[pyclass(name = "Quaternion")]
#[derive(Clone)]
struct PyQuaternion {
    #[pyo3(get, set)]
    w: f32,
    #[pyo3(get, set)]
    x: f32,
    #[pyo3(get, set)]
    y: f32,
    #[pyo3(get, set)]
    z: f32,
}

#[pymethods]
impl PyQuaternion {
    #[new]
    fn new(w: f32, x: f32, y: f32, z: f32) -> Self {
        PyQuaternion { w, x, y, z }
    }

    fn __repr__(&self) -> PyResult<String> {
        Ok(format!(
            "Quaternion(w={}, x={}, y={}, z={})",
            self.w, self.x, self.y, self.z
        ))
    }
}

// Convert between Rust and Python types
impl From<Quaternion> for PyQuaternion {
    fn from(q: Quaternion) -> Self {
        PyQuaternion {
            w: q.w,
            x: q.x,
            y: q.y,
            z: q.z,
        }
    }
}

impl From<PyQuaternion> for Quaternion {
    fn from(q: PyQuaternion) -> Self {
        Quaternion {
            w: q.w,
            x: q.x,
            y: q.y,
            z: q.z,
        }
    }
}

// Generic IMU Reader class for Python
#[pyclass(name = "ImuReader")]
struct PyImuReader {
    // We'll use a Box<dyn ImuReader> to allow different implementations
    reader: Box<dyn ImuReader + Send + Sync>,
}

#[pymethods]
impl PyImuReader {
    // Shared method to get IMU data
    fn get_data(&self, py: Python) -> PyResult<PyObject> {
        match self.reader.get_data() {
            Ok(data) => {
                let dict = PyDict::new(py);

                if let Some(accel) = data.accelerometer {
                    dict.set_item("accelerometer", PyVector3::from(accel))?;
                }
                if let Some(gyro) = data.gyroscope {
                    dict.set_item("gyroscope", PyVector3::from(gyro))?;
                }
                if let Some(mag) = data.magnetometer {
                    dict.set_item("magnetometer", PyVector3::from(mag))?;
                }
                if let Some(quat) = data.quaternion {
                    dict.set_item("quaternion", PyQuaternion::from(quat))?;
                }
                if let Some(euler) = data.euler {
                    dict.set_item("euler", PyVector3::from(euler))?;
                }
                if let Some(lin_accel) = data.linear_acceleration {
                    dict.set_item("linear_acceleration", PyVector3::from(lin_accel))?;
                }
                if let Some(gravity) = data.gravity {
                    dict.set_item("gravity", PyVector3::from(gravity))?;
                }
                if let Some(temp) = data.temperature {
                    dict.set_item("temperature", temp)?;
                }
                if let Some(cal) = data.calibration_status {
                    dict.set_item("calibration_status", cal)?;
                }

                Ok(dict.into())
            }
            Err(e) => Err(PyRuntimeError::new_err(e.to_string())),
        }
    }

    // Shared method to stop the reader
    fn stop(&self) -> PyResult<()> {
        self.reader
            .stop()
            .map_err(|e| PyRuntimeError::new_err(e.to_string()))
    }
}

// Factory functions for different IMU types - conditional compilation
#[pyfunction]
fn create_bno055_reader(i2c_device: &str) -> PyResult<PyImuReader> {
    match imu::Bno055Reader::new(i2c_device) {
        Ok(reader) => Ok(PyImuReader {
            reader: Box::new(reader),
        }),
        Err(e) => Err(PyRuntimeError::new_err(e.to_string())),
    }
}

#[pyfunction]
fn create_hiwonder_reader(serial_port: &str, baud_rate: u32) -> PyResult<PyImuReader> {
    match imu::HiwonderReader::new(serial_port, baud_rate) {
        Ok(reader) => Ok(PyImuReader {
            reader: Box::new(reader),
        }),
        Err(e) => Err(PyRuntimeError::new_err(e.to_string())),
    }
}

#[pyfunction]
fn create_bmi088_reader(i2c_device: &str) -> PyResult<PyImuReader> {
    match imu::Bmi088Reader::new(i2c_device) {
        Ok(reader) => Ok(PyImuReader {
            reader: Box::new(reader),
        }),
        Err(e) => Err(PyRuntimeError::new_err(e.to_string())),
    }
}

#[pyfunction]
fn create_hexmove_reader(can_interface: &str, node_id: u8, param_id: u8) -> PyResult<PyImuReader> {
    match imu::HexmoveImuReader::new(can_interface, node_id, param_id) {
        Ok(reader) => Ok(PyImuReader {
            reader: Box::new(reader),
        }),
        Err(e) => Err(PyRuntimeError::new_err(e.to_string())),
    }
}

#[pymodule]
fn bindings(m: &Bound<PyModule>) -> PyResult<()> {
    m.add_class::<PyVector3>()?;
    m.add_class::<PyQuaternion>()?;
    m.add_class::<PyImuReader>()?;

    m.add_function(wrap_pyfunction!(create_bno055_reader, m)?)?;

    m.add_function(wrap_pyfunction!(create_hiwonder_reader, m)?)?;

    m.add_function(wrap_pyfunction!(create_bmi088_reader, m)?)?;

    m.add_function(wrap_pyfunction!(create_hexmove_reader, m)?)?;

    Ok(())
}
