TARGET = test
CC = gcc
CFLAGS = -O2 -g -Wall -Wextra -Wundef -Wno-unused-function -std=gnu99 -DNO_STM32 \
         -DHW_SOURCE=\"hw.h\" -DHW_HEADER=\"hw.h\" \
         -I. -I../.. -I../../util -I../../applications -I../../motor -I../../comm
LIBS = -lm

SOURCES = main.c ../../applications/app_pas.c

# Board configurations to cover. The empty one is dedicated pedal sensor pins
# with no torque sensor.
VARIANTS = default TEST_NO_PAS_PINS TEST_PAS_PINS_SHARED TEST_TORQUE_SENSOR

.PHONY: default all clean run run-all

default: $(TARGET)
all: default

$(TARGET): $(SOURCES)
	$(CC) $(CFLAGS) $(SOURCES) $(LIBS) -o $@

clean:
	rm -f $(TARGET) $(addprefix $(TARGET)-,$(VARIANTS))

run: $(TARGET)
	./$(TARGET)

# Build and run every board configuration, and fail if any of them fails.
run-all:
	@rc=0; \
	for v in $(VARIANTS); do \
		if [ "$$v" = default ]; then d=""; else d="-D$$v"; fi; \
		$(CC) $(CFLAGS) $$d $(SOURCES) $(LIBS) -o $(TARGET)-$$v || exit 1; \
		echo "--- $$v ---"; \
		./$(TARGET)-$$v | tail -2 || rc=1; \
	done; \
	exit $$rc
