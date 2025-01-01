mod hexmove;
mod hiwonder;

use pyo3::prelude::*;
use pyo3_stub_gen::define_stub_info_gatherer;

pub use hexmove::{PyHexmoveImuData, PyHexmoveImuReader};
pub use hiwonder::PyHiwonderImu;

#[pymodule]
fn bindings(m: &Bound<PyModule>) -> PyResult<()> {
    m.add_class::<PyHexmoveImuReader>()?;
    m.add_class::<PyHexmoveImuData>()?;
    m.add_class::<PyHiwonderImu>()?;
    Ok(())
}

define_stub_info_gatherer!(stub_info);
