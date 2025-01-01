use hiwonder::IMU;
use pyo3::prelude::*;
use pyo3_stub_gen::derive::{gen_stub_pyclass, gen_stub_pymethods};
use std::sync::{Arc, Mutex};

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

    fn read_data(&mut self) -> PyResult<Option<([f32; 3], [f32; 3], [f32; 3])>> {
        let mut imu = self
            .inner
            .lock()
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))?;
        imu.read_data()
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))
    }
}
