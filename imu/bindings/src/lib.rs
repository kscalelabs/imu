mod hexmove;
mod hiwonder;

use pyo3::prelude::*;

pub use hexmove::{PyHexmoveImuData, PyHexmoveImuReader};
pub use hiwonder::PyHiwonderImu;

#[pymodule]
fn bindings(m: &Bound<PyModule>) -> PyResult<()> {
    m.add_class::<PyHexmoveImuReader>()?;
    m.add_class::<PyHexmoveImuData>()?;
    m.add_class::<PyHiwonderImu>()?;
    Ok(())
}
