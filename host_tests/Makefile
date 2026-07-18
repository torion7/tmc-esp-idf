# Host-Tests: compilieren die IDF-freien tmc_motion-Quellen mit dem Host-gcc
# gegen Referenzvektoren (kein ESP-IDF, kein Framework noetig).
#
#   make -C host_tests        # bauen + ausfuehren
#   make -C host_tests clean

CC      ?= cc
CFLAGS  ?= -O2 -Wall -Wextra -Werror -std=c11
CFLAGS  += -I stubs -I ../tmc_motion/include -I .

BIN     := build/test_decode_diag build/test_units

all: $(BIN)
	@set -e; for t in $(BIN); do ./$$t; done

build:
	mkdir -p build

build/test_decode_diag: test_decode_diag.c ../tmc_motion/src/tmc_motion_decode.c \
                        test_util.h stubs/esp_err.h | build
	$(CC) $(CFLAGS) -o $@ test_decode_diag.c ../tmc_motion/src/tmc_motion_decode.c -lm

build/test_units: test_units.c ../tmc_motion/src/tmc_motion_units.c \
                  test_util.h stubs/esp_err.h | build
	$(CC) $(CFLAGS) -o $@ test_units.c ../tmc_motion/src/tmc_motion_units.c -lm

clean:
	rm -rf build

.PHONY: all clean
