use hiwonder::IMU;
use pyo3::prelude::*;
use pyo3_stub_gen::derive::{gen_stub_pyclass, gen_stub_pymethods};
use std::sync::{Arc, Mutex};

#[gen_stub_pyclass]
#[pyclass(name = "ImuData")]
#[derive(Clone)]
struct PyImuData {
    #[pyo3(get)]
    accelerometer: Vec<f32>,
    #[pyo3(get)]
    gyroscope: Vec<f32>,
    #[pyo3(get)]
    angle: Vec<f32>,
    #[pyo3(get)]
    quaternion: Vec<f32>,
}

#[gen_stub_pymethods]
#[pymethods]
impl PyImuData {
    #[new]
    fn new(accelerometer: Vec<f32>, gyroscope: Vec<f32>, angle: Vec<f32>, quaternion: Vec<f32>) -> Self {
        Self {
            accelerometer,
            gyroscope,
            angle,
            quaternion,
        }
    }
}

#[gen_stub_pyclass]
#[pyclass(name = "HiwonderImu")]
pub struct PyHiwonderImu {
    inner: Arc<Mutex<IMU>>,
}

#[gen_stub_pymethods]
#[pymethods]
impl PyHiwonderImu {
    #[new]
    fn new(interface: String, baud_rate: u32) -> PyResult<Self> {
        let imu = IMU::new(&interface, baud_rate)
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))?;
        Ok(PyHiwonderImu {
            inner: Arc::new(Mutex::new(imu)),
        })
    }

    fn read_data(&mut self) -> PyResult<Option<PyObject>> {
        let mut imu = self
            .inner
            .lock()
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))?;

        match imu.read_data() {
            Ok(Some((accel, gyro, angle, quat))) => {
                Python::with_gil(|py| {
                    let data = PyImuData::new(
                        accel.to_vec(),
                        gyro.to_vec(),
                        angle.to_vec(),
                        quat.to_vec()
                    );
                    Ok(Some(Py::new(py, data)?.into_py(py)))
                })
            }
            Ok(None) => Ok(None),
            Err(e) => Err(PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))
        }
    }
}
