FROM ubuntu:22.04

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1
ENV CARGO_HOME=/usr/local/cargo
ENV RUSTUP_HOME=/usr/local/rustup
ENV PATH="/usr/local/cargo/bin:${PATH}"

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3.11 \
    python3.11-dev \
    python3-pip \
    curl \
    pkg-config \
    libudev-dev \
    build-essential \
    git \
    && rm -rf /var/lib/apt/lists/*

# Install Rust
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y

# Copy requirements files
COPY imu/requirements.txt /tmp/requirements.txt
COPY imu/requirements-dev.txt /tmp/requirements-dev.txt

# Install Python dependencies
RUN python3.11 -m pip install --upgrade pip && \
    python3.11 -m pip install build wheel setuptools-rust && \
    python3.11 -m pip install -r /tmp/requirements.txt -r /tmp/requirements-dev.txt

# Create a non-root user
RUN useradd -m -u 1000 builder

WORKDIR /app

# Create cache directories with correct permissions
RUN mkdir -p /home/builder/.cache/black \
    && mkdir -p /home/builder/.cache/ruff \
    && mkdir -p /home/builder/.mypy_cache \
    && chown -R builder:builder /home/builder/.cache \
    && chown -R builder:builder /home/builder/.mypy_cache \
    && chown builder:builder /app

USER builder

# Create .ruff_cache in the user's home directory instead
ENV RUFF_CACHE_DIR=/home/builder/.cache/ruff

CMD ["/bin/bash"]
