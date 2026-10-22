# SPDX-License-Identifier: Apache-2.0
# Host build (gcc). Cross build for the Pluto later with CC=arm-linux-gnueabihf-gcc.
CC      ?= gcc
CFLAGS  ?= -O2 -g -Wall -Wextra -std=gnu11
LDLIBS  += -lm
BUILD   := build
SRC     := src/dab_crc.c src/dab_rs.c src/dabplus_sf.c src/aac_mdct.c src/aac_huff_tab.c src/aac_enc.c src/dabplus_enc.c src/dab_mod.c src/dab_pad.c
OBJ     := $(patsubst src/%.c,$(BUILD)/%.o,$(SRC))
PY      ?= python3

all: $(BUILD)/test_sf $(BUILD)/test_mdct $(BUILD)/enc_file $(BUILD)/dab_tx_file $(BUILD)/skypluto-dab

$(BUILD)/skypluto-dab: app/skypluto_dab.c $(OBJ) | $(BUILD)
	$(CC) $(CFLAGS) $^ -o $@ $(LDLIBS) -lpthread

$(BUILD)/dab_tx_file: tests/dab_tx_file.c $(OBJ) | $(BUILD)
	$(CC) $(CFLAGS) $^ -o $@ $(LDLIBS)

$(BUILD)/test_mdct: tests/test_mdct.c $(OBJ) | $(BUILD)
	$(CC) $(CFLAGS) $^ -o $@ $(LDLIBS)

$(BUILD)/enc_file: tests/enc_file.c $(OBJ) | $(BUILD)
	$(CC) $(CFLAGS) $^ -o $@ $(LDLIBS)

$(BUILD)/%.o: src/%.c | $(BUILD)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/test_sf: tests/test_sf.c $(OBJ) | $(BUILD)
	$(CC) $(CFLAGS) $^ -o $@ $(LDLIBS)

$(BUILD):
	mkdir -p $(BUILD)/out

test: all
	$(BUILD)/test_sf $(BUILD)/out
	$(PY) tests/check_sf.py $(BUILD)/out

clean:
	rm -rf $(BUILD)

.PHONY: all test clean
