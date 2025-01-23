use hexmove::{IMUData as HexmoveImuData, ImuReader as HexmoveImuReader};
use pyo3::prelude::*;
use pyo3_stub_gen::derive::{gen_stub_pyclass, gen_stub_pymethods};
use std::sync::{Arc, Mutex};

#[gen_stub_pyclass]
#[pyclass]
pub struct PyHexmoveImuReader {
    inner: Arc<Mutex<HexmoveImuReader>>,
}

#[gen_stub_pymethods]
#[pymethods]
impl PyHexmoveImuReader {
    #[new]
    fn new(interface: String, serial_number: u8, model: u8) -> PyResult<Self> {
        let imu_reader = HexmoveImuReader::new(&interface, serial_number, model)
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))?;
        Ok(PyHexmoveImuReader {
            inner: Arc::new(Mutex::new(imu_reader)),
        })
    }

    fn get_data(&self) -> PyResult<PyHexmoveImuData> {
        let imu_reader = self
            .inner
            .lock()
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))?;
        let data = imu_reader
            .get_data()
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e))?;
        Ok(PyHexmoveImuData::from(data))
    }

    fn get_angles(&self) -> PyResult<(f32, f32, f32)> {
        let imu_reader = self
            .inner
            .lock()
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))?;
        imu_reader
            .get_angles()
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e))
    }

    fn get_velocities(&self) -> PyResult<(f32, f32, f32)> {
        let imu_reader = self
            .inner
            .lock()
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))?;
        imu_reader
            .get_velocities()
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e))
    }

    fn get_accelerations(&self) -> PyResult<(f32, f32, f32)> {
        let imu_reader = self
            .inner
            .lock()
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))?;
        imu_reader
            .get_accelerations()
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e))
    }

    fn get_quaternion(&self) -> PyResult<(f32, f32, f32, f32)> {
        let imu_reader = self
            .inner
            .lock()
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))?;
        imu_reader
            .get_quaternion()
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e))
    }

    #[pyo3(signature = (duration_ms=None, max_retries=None, max_variance=None))]
    fn zero_imu(
        &self,
        duration_ms: Option<u64>,
        max_retries: Option<u32>,
        max_variance: Option<f32>,
    ) -> PyResult<()> {
        let imu_reader = self.inner.lock().unwrap();
        imu_reader
            .zero_imu(duration_ms, max_retries, max_variance)
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e))
    }

    fn stop(&self) -> PyResult<()> {
        let imu_reader = self
            .inner
            .lock()
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))?;
        imu_reader
            .stop()
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))?;
        Ok(())
    }
}

#[gen_stub_pyclass]
#[pyclass]
#[derive(Clone)]
pub struct PyHexmoveImuData {
    #[pyo3(get)]
    euler: Vec<f32>,
    #[pyo3(get)]
    gyroscope: Vec<f32>,
    #[pyo3(get)]
    accelerometer: Vec<f32>,
    #[pyo3(get)]
    quaternion: Vec<f32>,
}

impl From<HexmoveImuData> for PyHexmoveImuData {
    fn from(data: HexmoveImuData) -> Self {
        PyHexmoveImuData {
            euler: vec![data.euler.roll, data.euler.pitch, data.euler.yaw],
            gyroscope: vec![data.gyroscope.x, data.gyroscope.y, data.gyroscope.z],
            accelerometer: vec![data.accelerometer.x, data.accelerometer.y, data.accelerometer.z],
            quaternion: vec![data.quaternion.w, data.quaternion.x, data.quaternion.y, data.quaternion.z],
        }
    }
}
