# SPDX-License-Identifier: GPL-2.0-only
TEST_PROGS := run.sh
TEST_FILES := bind-uart3.sh
CARGO ?= cargo
ifneq ($(shell command -v $(CARGO) 2>/dev/null),)
TEST_GEN_FILES := rs485-test
endif

include ../lib.mk

$(OUTPUT)/rs485-test: Cargo.toml Cargo.lock $(wildcard src/*.rs)
	CARGO_TARGET_DIR=$(OUTPUT)/cargo $(CARGO) build --locked --release --bin rs485-test $(CARGO_FLAGS)
	cp $(OUTPUT)/cargo/$(if $(CARGO_BUILD_TARGET),$(CARGO_BUILD_TARGET)/)release/rs485-test $@

EXTRA_CLEAN := cargo
