# Host tests for the script container parser.
#
# The parser is compiled straight from script/script_pack.c, not copied here,
# so the test cannot drift from what ships. It has no firmware dependencies --
# string.h and nothing else -- which is the point: untrusted-input parsing
# belongs where it can be fuzzed and run under sanitizers.
#
# container.bin is the cross-repo anchor: a container that
# vesc_express/tools/luapack.py actually emitted. The unit tests build their
# own containers and would keep passing if the format drifted; that one would
# not.

TARGET = test
LIBS = -lm
CC = gcc
CFLAGS = -O1 -g -Wall -Wextra -Wundef -Wconversion -Wsign-conversion \
         -std=gnu99 -I../../script -DNO_STM32
SAN ?= -fsanitize=address,undefined -fno-omit-frame-pointer
SOURCES = main.c ../../script/script_pack.c
HEADERS = ../../script/script_pack.h

.PHONY: default all run clean

default: $(TARGET)
all: default
run: $(TARGET)
	./$(TARGET)

$(TARGET): $(SOURCES) $(HEADERS)
	$(CC) $(CFLAGS) $(SAN) -o $@ $(SOURCES) $(LIBS)

clean:
	rm -f $(TARGET)

# Not in the firmware build yet. script/script_pack.c is unreferenced by
# make/fw.mk on purpose: with LispBM in, the app is 95.5% of the partition,
# and adding code nothing calls would spend flash to no end. It goes into
# CSRC when an engine needs it.
