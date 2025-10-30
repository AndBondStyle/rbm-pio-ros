.SILENT:
.ONESHELL:

SHELL = /bin/bash

.PHONY: all
all:
	$(error Please use explicit targets)

.PHONY: build
build:
	source /opt/ros/jazzy/setup.sh
	colcon --log-base /dev/null build \
		--base-paths packages \
		--executor parallel \
		--parallel-workers $$(nproc) \
		--symlink-install \
		--packages-up-to $(packages)

.PHONY: build-all
build-all:
	source /opt/ros/jazzy/setup.sh
	colcon --log-base /dev/null build \
		--base-paths packages \
		--executor parallel \
		--parallel-workers $$(nproc) \
		--symlink-install

.PHONY: clean
clean:
	rm -rf build install
