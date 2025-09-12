"""Python bindings for the IMU package."""

# Import the compiled extension module from parent directory
import sys
import os
import importlib.util

# Get the path to the compiled binary
parent_dir = os.path.dirname(os.path.dirname(__file__))
binary_path = os.path.join(parent_dir, 'bindings.cpython-311-aarch64-linux-gnu.so')

# Load the compiled binary directly
spec = importlib.util.spec_from_file_location("bindings", binary_path)
bindings_binary = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bindings_binary)

# Make all symbols available in this module's namespace
globals().update({name: getattr(bindings_binary, name) for name in dir(bindings_binary) if not name.startswith('_')})
