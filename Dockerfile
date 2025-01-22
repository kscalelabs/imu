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

# Install Python dependencies
RUN python3.11 -m pip install --upgrade pip && \
    python3.11 -m pip install build wheel setuptools-rust

WORKDIR /app

# Create a non-root user
RUN useradd -m -u 1000 builder
USER builder

CMD ["/bin/bash"]
