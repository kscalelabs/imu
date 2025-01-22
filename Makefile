# Makefile

# ------------------------ #
#          Build           #
# ------------------------ #

install:
	cargo run --bin stub_gen
	@touch setup.py
	@uv pip install -e '.[dev]'
.PHONY: build

# ------------------------ #
#       Static Checks      #
# ------------------------ #

py-files := $(shell find . -name '*.py')

format:
	@black $(py-files)
	@ruff format $(py-files)
	@cargo fmt
.PHONY: format

format-cpp:
	@clang-format -i $(shell find . -name '*.cpp' -o -name '*.h')
	@cmake-format -i $(shell find . -name 'CMakeLists.txt' -o -name '*.cmake')
.PHONY: format-cpp

static-checks-python:
	@black --diff --check $(py-files)
	@ruff check $(py-files)
	@mypy --install-types --non-interactive $(py-files)
.PHONY: static-checks-python

static-checks-rust:
	@cargo clippy
	@cargo fmt --check
.PHONY: lint

static-checks:
	@$(MAKE) static-checks-python
	@$(MAKE) static-checks-rust
.PHONY: static-checks

mypy-daemon:
	@dmypy run -- $(py-files)
.PHONY: mypy-daemon

# ------------------------ #
#        Unit tests        #
# ------------------------ #

test:
	python -m pytest
.PHONY: test

# ------------------------ #
#         Docker          #
# ------------------------ #

DOCKER_IMAGE := imu-builder
DOCKER_TAG := latest

docker-build:
	docker build -t $(DOCKER_IMAGE):$(DOCKER_TAG) .
.PHONY: docker-build

docker-shell:
	docker run --rm -it \
		-v $(PWD):/app \
		$(DOCKER_IMAGE):$(DOCKER_TAG) /bin/bash
.PHONY: docker-shell

docker-install:
	docker run --rm \
		-v $(PWD):/app \
		$(DOCKER_IMAGE):$(DOCKER_TAG) \
		make install
.PHONY: docker-install

docker-test:
	docker run --rm \
		-v $(PWD):/app \
		$(DOCKER_IMAGE):$(DOCKER_TAG) \
		make test
.PHONY: docker-test

docker-static-checks:
	docker run --rm \
		-v $(PWD):/app \
		$(DOCKER_IMAGE):$(DOCKER_TAG) \
		make static-checks
.PHONY: docker-static-checks

docker-build-wheel:
	docker run --rm \
		-v $(PWD):/app \
		-v $(PWD)/dist:/app/dist \
		$(DOCKER_IMAGE):$(DOCKER_TAG) \
		python3.11 -m build --wheel --outdir dist
.PHONY: docker-build-wheel

docker-build-sdist:
	docker run --rm \
		-v $(PWD):/app \
		-v $(PWD)/dist:/app/dist \
		$(DOCKER_IMAGE):$(DOCKER_TAG) \
		python3.11 -m build --sdist --outdir dist
.PHONY: docker-build-sdist
