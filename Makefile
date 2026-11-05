ROOT_DIR := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))
BUILD_DIR := $(ROOT_DIR)/build
APRS_DIR := $(ROOT_DIR)/UV98_868_220430_D4
APRS_PROJECT := UV98_868_220430_D4.uvproj
APRS_OUTPUT := OBJ/UV98_868_220430_D4.hex
APRS_IMAGE := $(BUILD_DIR)/aprs/UV98_868_220430_D4.hex

DOCKER ?= docker
DOCKER_IMAGE ?= hg-uv98-firmware-builder
KEIL_UV4 ?= UV4

.DEFAULT_GOAL := all

.PHONY: all help main main-container aprs test-host clean

all: main-container aprs

help:
	@printf '%s\n' \
	  'HG-UV98 firmware build targets:' \
	  '  make all           Build the ARM image in Docker and APRS HEX with Keil C51' \
	  '  make main          Build the ARM reconstruction using the local GNU toolchain' \
	  '  make main-container Build the ARM reconstruction in Docker' \
	  '  make aprs          Build APRS HEX using Keil UV4 (set KEIL_UV4 if needed)' \
	  '  make test-host     Run host-side protocol tests' \
	  '  make clean         Remove generated build outputs'

main:
	$(MAKE) -C "$(ROOT_DIR)/reconstruction" all

main-container:
	$(DOCKER) build --tag "$(DOCKER_IMAGE)" --file "$(ROOT_DIR)/Dockerfile" "$(ROOT_DIR)"
	$(DOCKER) run --rm --user "$$(id -u):$$(id -g)" \
		--volume "$(ROOT_DIR):/workspace" \
		--workdir /workspace/reconstruction \
		"$(DOCKER_IMAGE)" make all

aprs:
	@command -v "$(KEIL_UV4)" >/dev/null 2>&1 || { \
		printf 'Keil UV4 not found: %s. Install Keil C51 and set KEIL_UV4 to its UV4 executable.\n' "$(KEIL_UV4)" >&2; \
		exit 127; \
	}
	@mkdir -p "$(BUILD_DIR)/aprs" "$(APRS_DIR)/OBJ"
	@rm -f "$(APRS_DIR)/$(APRS_OUTPUT)"
	cd "$(APRS_DIR)" && "$(KEIL_UV4)" -b "$(APRS_PROJECT)" -o "$(BUILD_DIR)/aprs/keil-build.log"
	@test -s "$(APRS_DIR)/$(APRS_OUTPUT)" || { \
		printf 'Keil did not produce the expected HEX file: %s\n' "$(APRS_DIR)/$(APRS_OUTPUT)" >&2; \
		exit 1; \
	}
	cp "$(APRS_DIR)/$(APRS_OUTPUT)" "$(APRS_IMAGE)"
	@printf 'APRS firmware: %s\n' "$(APRS_IMAGE)"

test-host:
	$(MAKE) -C "$(ROOT_DIR)/reconstruction" test-host

clean:
	$(MAKE) -C "$(ROOT_DIR)/reconstruction" clean
	rm -f "$(BUILD_DIR)/aprs/UV98_868_220430_D4.hex" "$(BUILD_DIR)/aprs/keil-build.log"
	rm -f "$(APRS_DIR)/$(APRS_OUTPUT)"
