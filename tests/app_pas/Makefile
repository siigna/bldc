TARGET = test
CC = gcc
CFLAGS = -O2 -g -Wall -Wextra -Wundef -Wno-unused-function -std=gnu99 -DNO_STM32 \
         -DHW_SOURCE=\"hw.h\" -DHW_HEADER=\"hw.h\" \
         -I. -I../.. -I../../util -I../../applications -I../../motor -I../../comm
LIBS = -lm

FIRMWARE = ../../applications/app_pas.c ../../util/utils_math.c
SOURCES = main.c fixture.c $(FIRMWARE)

# Board configurations to cover. The empty one is dedicated pedal sensor pins
# with no torque sensor.
VARIANTS = default TEST_NO_PAS_PINS TEST_PAS_PINS_SHARED TEST_TORQUE_SENSOR

.PHONY: default all clean run run-all run-san coverage fuzz fuzz-run

default: $(TARGET)
all: default

$(TARGET): $(SOURCES)
	$(CC) $(CFLAGS) $(SOURCES) $(LIBS) -o $@

clean:
	rm -f $(TARGET) $(addprefix $(TARGET)-,$(VARIANTS))
	rm -f fuzz test-cov *.gcda *.gcno *.gcov
	rm -rf coverage

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

# --- Analysis ---------------------------------------------------------------

# Address and undefined-behaviour sanitizers over every board variant. -O1 and
# a frame pointer so a report names the right line.
SANFLAGS = -O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer \
           -fno-sanitize-recover=undefined

run-san:
	@rc=0; \
	for v in $(VARIANTS); do \
		if [ "$$v" = default ]; then d=""; else d="-D$$v"; fi; \
		$(CC) $(filter-out -O2,$(CFLAGS)) $(SANFLAGS) $$d $(SOURCES) $(LIBS) \
			-o $(TARGET)-san-$$v || exit 1; \
		echo "--- $$v (asan+ubsan) ---"; \
		./$(TARGET)-san-$$v | tail -2 || rc=1; \
	done; \
	rm -f $(addprefix $(TARGET)-san-,$(VARIANTS)); \
	exit $$rc

# Line coverage of the firmware under test, not of the test file: what matters
# is which branches of app_pas.c the tests actually reach.
coverage:
	@$(CC) $(filter-out -O2,$(CFLAGS)) -O0 -g --coverage $(SOURCES) $(LIBS) \
		-o test-cov 2>/dev/null
	@./test-cov > /dev/null
	@gcov -b test-cov-app_pas.gcda 2>/dev/null | \
		sed -n "/app_pas.c'/,/^$$/p" | head -5
	@echo "app_pas.c.gcov is the annotated source. Lines marked ##### were"
	@echo "never reached; a branch marked 'never executed' was never taken."
	@echo
	@echo "Uncovered, by function:"
	@python3 uncovered.py app_pas.c.gcov

# Property fuzzer. Needs clang for libFuzzer; gcc has no equivalent.
FUZZ_CC ?= clang
fuzz:
	$(FUZZ_CC) $(filter-out -O2,$(CFLAGS)) -O1 -g \
		-fsanitize=fuzzer,address,undefined -fno-sanitize-recover=undefined \
		fuzz.c fixture.c $(FIRMWARE) $(LIBS) -o fuzz

# A short default run, so this is usable as a check rather than only as a
# campaign. Override the time for a longer one.
FUZZ_SECS ?= 30
fuzz-run: fuzz
	mkdir -p corpus
	./fuzz corpus -max_total_time=$(FUZZ_SECS) -print_final_stats=1
