use hiwonder::{HiwonderReader, ImuData};
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
    fn new(
        accelerometer: Vec<f32>,
        gyroscope: Vec<f32>,
        angle: Vec<f32>,
        quaternion: Vec<f32>,
    ) -> Self {
        Self {
            accelerometer,
            gyroscope,
            angle,
            quaternion,
        }
    }
}

impl From<ImuData> for PyImuData {
    fn from(data: ImuData) -> Self {
        PyImuData {
            accelerometer: data.accelerometer.to_vec(),
            gyroscope: data.gyroscope.to_vec(),
            angle: data.angle.to_vec(),
            quaternion: data.quaternion.to_vec(),
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
