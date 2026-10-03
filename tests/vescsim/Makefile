# A controller over TCP, with no controller. See vescsim.c.
#
#   make && ./vescsim -p 65102
#
# Uses this repository's own comm/packet.c and util/crc.c, so the framing and
# the CRC are the firmware's. NO_STM32 is what lets crc.c build for a host;
# tests/packet_recovery does the same.

TARGET  = vescsim
CC      = gcc
CFLAGS  = -O2 -g -Wall -Wextra -Wundef -std=gnu99 \
          -I../../util -I../../comm -DNO_STM32
SOURCES = vescsim.c ../../comm/packet.c ../../util/crc.c
OBJECTS = $(notdir $(SOURCES:.c=.o))

.PHONY: default all clean check-version check-layout check

default: $(TARGET)
all: default

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

%.o: ../../comm/%.c
	$(CC) $(CFLAGS) -c $< -o $@

%.o: ../../util/%.c
	$(CC) $(CFLAGS) -c $< -o $@

$(TARGET): $(OBJECTS)
	$(CC) $(OBJECTS) -Wall -o $@

# The version the simulator reports has to match the firmware, or a tool will
# refuse to talk to it for a reason that looks like a protocol bug. conf_general.h
# cannot be included from a host build -- it pulls in the hardware headers and
# ChibiOS -- so it is compared rather than shared.
check-version: $(TARGET)
	@maj=$$(grep -E '^#define FW_VERSION_MAJOR' ../../conf_general.h | awk '{print $$3}'); \
	min=$$(grep -E '^#define FW_VERSION_MINOR' ../../conf_general.h | awk '{print $$3}' | sed 's/^0*//'); \
	smaj=$$(grep -E '^#define SIM_FW_VERSION_MAJOR' vescsim.c | awk '{print $$3}'); \
	smin=$$(grep -E '^#define SIM_FW_VERSION_MINOR' vescsim.c | awk '{print $$3}'); \
	if [ "$$maj" != "$$smaj" ] || [ "$$min" != "$$smin" ]; then \
	    echo "  FAILED  vescsim reports $$smaj.$$smin, conf_general.h says $$maj.$$min"; \
	    exit 1; \
	fi; \
	echo "  ok      vescsim reports the firmware version ($$smaj.$$smin)"

# The reply layout, compared against the firmware that defines it.
check-layout:
	@python3 ./layout.py

check: check-version check-layout

clean:
	-rm -f *.o $(TARGET)
