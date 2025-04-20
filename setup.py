#!/usr/bin/env python
"""Setup script for the project."""
# mypy: disable-error-code="import-untyped"

from pathlib import Path

import toml
from setuptools import find_packages, setup
from setuptools_rust import Binding, RustExtension

# Read README for long description
with open("README.md", "r", encoding="utf-8") as f:
    long_description: str = f.read()

# Read version from root Cargo.toml workspace definition
cargo_toml_path = Path(__file__).parent / "Cargo.toml"
with open(cargo_toml_path, "r", encoding="utf-8") as f:
    cargo_toml = toml.load(f)
    # Adjust this path if your version is defined differently (e.g., not in workspace)
    version: str = cargo_toml["workspace"]["package"]["version"]

with open("imu/requirements.txt", "r", encoding="utf-8") as f:
    requirements: list[str] = f.read().splitlines()

with open("imu/requirements-dev.txt", "r", encoding="utf-8") as f:
    requirements_dev: list[str] = f.read().splitlines()

# Define package data (simplified)
package_data = {"imu": ["py.typed", "*.pyi"]}


setup(
    name="imu-py",
    version=version,
    description="Python interface for interacting with IMUs",
    author="Wesley Maa",
    url="https://github.com/kscalelabs/imu",
    rust_extensions=[
        RustExtension(
            "imu.bindings",  # Use target only, path inferred or explicitly set
            path="imu/bindings/Cargo.toml",
            binding=Binding.PyO3,
            py_limited_api="auto",  # Recommended for PyO3
            # features=["pyo3/extension-module"] # May be needed?
        ),
    ],
    packages=find_packages(include=["imu", "imu.*"]),  # Find packages under imu
    zip_safe=False,
    long_description=long_description,
    long_description_content_type="text/markdown",
    python_requires=">=3.11",
    setup_requires=["setuptools-rust>=1.5.2", "toml"],
    install_requires=requirements,
    extras_require={"dev": requirements_dev},
    include_package_data=True,
    package_data=package_data,
    # No custom cmdclass needed
)
