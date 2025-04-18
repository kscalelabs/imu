use hiwonder::{HiwonderReader, ImuData, ImuReader};
use pyo3::prelude::*;
use pyo3_stub_gen::derive::{gen_stub_pyclass, gen_stub_pymethods};
use std::sync::{Arc, Mutex};

#[gen_stub_pyclass]
#[pyclass(name = "ImuData")]
#[derive(Clone)]
struct PyImuData {
    #[pyo3(get)]
    accelerometer: Option<Vec<f32>>,
    #[pyo3(get)]
    gyroscope: Option<Vec<f32>>,
    #[pyo3(get)]
    angle: Option<Vec<f32>>,
    #[pyo3(get)]
    quaternion: Option<Vec<f32>>,
    #[pyo3(get)]
    magnetometer: Option<Vec<f32>>,
    #[pyo3(get)]
    temperature: Option<f32>,
}

#[gen_stub_pymethods]
#[pymethods]
impl PyImuData {
    #[new]
    fn new(
        accelerometer: Option<Vec<f32>>,
        gyroscope: Option<Vec<f32>>,
        angle: Option<Vec<f32>>,
        quaternion: Option<Vec<f32>>,
        magnetometer: Option<Vec<f32>>,
        temperature: Option<f32>,
    ) -> Self {
        Self {
            accelerometer,
            gyroscope,
            angle,
            quaternion,
            magnetometer,
            temperature,
        }
    }
}

impl From<ImuData> for PyImuData {
    fn from(data: ImuData) -> Self {
        PyImuData {
            accelerometer: data.accelerometer.map(|v| vec![v.x, v.y, v.z]),
            gyroscope: data.gyroscope.map(|v| vec![v.x, v.y, v.z]),
            angle: data.euler.map(|v| vec![v.x, v.y, v.z]),
            quaternion: data.quaternion.map(|v| vec![v.w, v.x, v.y, v.z]),
            magnetometer: data.magnetometer.map(|v| vec![v.x, v.y, v.z]),
            temperature: data.temperature,
        }
    }
}

#[gen_stub_pyclass]
#[pyclass(name = "HiwonderImu")]
pub struct PyHiwonderImu {
    inner: Arc<Mutex<HiwonderReader>>,
}

#[gen_stub_pymethods]
#[pymethods]
impl PyHiwonderImu {
    #[new]
    fn new(interface: String, baud_rate: u32) -> PyResult<Self> {
        let reader = HiwonderReader::new(&interface, baud_rate)
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))?;
        Ok(PyHiwonderImu {
            inner: Arc::new(Mutex::new(reader)),
        })
    }

    fn get_data(&self) -> PyResult<PyImuData> {
        let reader = self
            .inner
            .lock()
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))?;
        let data = reader
            .get_data()
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))?;
        Ok(PyImuData::from(data))
    }

    fn reset(&self) -> PyResult<()> {
        let reader = self
            .inner
            .lock()
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))?;
        reader
            .reset()
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))?;
        Ok(())
    }

    fn stop(&self) -> PyResult<()> {
        let reader = self
            .inner
            .lock()
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))?;
        reader
            .stop()
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))?;
        Ok(())
    }
}
