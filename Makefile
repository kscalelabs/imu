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
