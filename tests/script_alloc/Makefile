# Host tests for the script heap's realloc semantics.
#
# Compiled from script/script_alloc.c, not a copy, with
# SCRIPT_ALLOC_HOST_TEST so the arena is malloc rather than a ChibiOS heap
# over CCM. What is under test is the realloc arithmetic and the ceiling; the
# arena itself needs the target.

TARGET = test
CC = gcc
CFLAGS = -O1 -g -Wall -Wextra -Wundef -std=gnu99 -I../../script \
         -DSCRIPT_ALLOC_HOST_TEST
SAN ?= -fsanitize=address,undefined -fno-omit-frame-pointer
SOURCES = main.c ../../script/script_alloc.c
HEADERS = ../../script/script_alloc.h

.PHONY: default all run clean

default: $(TARGET)
all: default
run: $(TARGET)
	./$(TARGET)

$(TARGET): $(SOURCES) $(HEADERS)
	$(CC) $(CFLAGS) $(SAN) -o $@ $(SOURCES)

clean:
	rm -f $(TARGET)
