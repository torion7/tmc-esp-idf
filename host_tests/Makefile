# Host-Tests: compilieren die IDF-freien tmc_motion-Quellen und den
# TMC5160-Registertreiber (gegen IDF-Stubs + SPI-Ersatz) mit dem Host-gcc
# gegen Referenzvektoren (kein ESP-IDF, kein Framework noetig).
#
#   make -C host_tests        # bauen + ausfuehren
#   make -C host_tests clean

CC      ?= cc
CFLAGS  ?= -O2 -Wall -Wextra -Werror -std=c11
CFLAGS  += -I stubs -I ../tmc_motion/include -I .

BIN     := build/test_decode_diag build/test_units build/test_tmc5160

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

# tmc5160.c wird unveraendert gebaut; fake_spi.c ersetzt SPI/GPIO/Log und
# zeichnet jeden Registerzugriff auf.
STUBS_IDF := stubs/esp_err.h stubs/esp_log.h stubs/esp_check.h \
             stubs/driver/gpio.h stubs/driver/spi_master.h

build/test_tmc5160: test_tmc5160.c fake_spi.c fake_spi.h ../tmc5160/tmc5160.c \
                    ../tmc5160/include/tmc5160.h ../tmc5160/include/tmc5160_regs.h \
                    ../tmc5240/include/tmc5240_regs.h test_util.h $(STUBS_IDF) | build
	$(CC) $(CFLAGS) -I ../tmc5160/include -I ../tmc5240/include -o $@ \
	    test_tmc5160.c fake_spi.c ../tmc5160/tmc5160.c -lm

clean:
	rm -rf build

.PHONY: all clean
