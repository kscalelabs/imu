use hexmove::{ImuData as HexmoveImuData, ImuReader as HexmoveImuReader};
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
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))?;
        Ok(PyHexmoveImuData::from(data))
    }

    fn get_angles(&self) -> PyResult<(f32, f32, f32)> {
        let imu_reader = self
            .inner
            .lock()
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))?;
        let (x, y, z) = imu_reader
            .get_angles()
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))?;
        Ok((x, y, z))
    }

    fn get_velocities(&self) -> PyResult<(f32, f32, f32)> {
        let imu_reader = self
            .inner
            .lock()
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))?;
        let (x, y, z) = imu_reader
            .get_velocities()
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))?;
        Ok((x, y, z))
    }

    fn get_accelerations(&self) -> PyResult<(f32, f32, f32)> {
        let imu_reader = self
            .inner
            .lock()
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))?;
        let (x, y, z) = imu_reader
            .get_accelerations()
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))?;
        Ok((x, y, z))
    }

    fn get_quaternion(&self) -> PyResult<(f32, f32, f32, f32)> {
        let imu_reader = self
            .inner
            .lock()
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))?;
        let (w, x, y, z) = imu_reader
            .get_quaternion()
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))?;
        Ok((w, x, y, z))
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
    x_angle: f32,
    #[pyo3(get)]
    y_angle: f32,
    #[pyo3(get)]
    z_angle: f32,
    #[pyo3(get)]
    x_velocity: f32,
    #[pyo3(get)]
    y_velocity: f32,
    #[pyo3(get)]
    z_velocity: f32,
    #[pyo3(get)]
    x_angle_offset: f32,
    #[pyo3(get)]
    y_angle_offset: f32,
    #[pyo3(get)]
    z_angle_offset: f32,
    #[pyo3(get)]
    accel_x: f32,
    #[pyo3(get)]
    accel_y: f32,
    #[pyo3(get)]
    accel_z: f32,
    #[pyo3(get)]
    qw: f32,
    #[pyo3(get)]
    qx: f32,
    #[pyo3(get)]
    qy: f32,
    #[pyo3(get)]
    qz: f32,
}

impl From<HexmoveImuData> for PyHexmoveImuData {
    fn from(data: HexmoveImuData) -> Self {
        PyHexmoveImuData {
            x_angle: data.x_angle,
            y_angle: data.y_angle,
            z_angle: data.z_angle,
            x_velocity: data.x_velocity,
            y_velocity: data.y_velocity,
            z_velocity: data.z_velocity,
            x_angle_offset: data.x_angle_offset,
            y_angle_offset: data.y_angle_offset,
            z_angle_offset: data.z_angle_offset,
            accel_x: data.accel_x,
            accel_y: data.accel_y,
            accel_z: data.accel_z,
            qw: data.qw,
            qx: data.qx,
            qy: data.qy,
            qz: data.qz,
        }
    }
}
