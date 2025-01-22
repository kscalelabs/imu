use pyo3::prelude::*;
use pyo3_stub_gen::Result;

fn main() -> Result<()> {
    // Initialize the Python interpreter
    pyo3::prepare_freethreaded_python();
    
    Python::with_gil(|_py| {
        let stub = bindings::stub_info()?;
        stub.generate()?;
        Ok(())
    })
}
