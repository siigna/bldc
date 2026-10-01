# Host tests for the script event queue.
#
# Compiled from script/script_queue.c, not a copy, with
# SCRIPT_QUEUE_HOST_TEST to leave out the ChibiOS system lock. The test is
# single-threaded, so it tests the ring arithmetic rather than the mutual
# exclusion -- that part needs the target.

TARGET = test
CC = gcc
CFLAGS = -O1 -g -Wall -Wextra -Wundef -Wconversion -Wsign-conversion \
         -std=gnu99 -I../../script -DSCRIPT_QUEUE_HOST_TEST
SAN ?= -fsanitize=address,undefined -fno-omit-frame-pointer
SOURCES = main.c ../../script/script_queue.c
HEADERS = ../../script/script_queue.h ../../script/script_event.h

.PHONY: default all run clean

default: $(TARGET)
all: default
run: $(TARGET)
	./$(TARGET)

$(TARGET): $(SOURCES) $(HEADERS)
	$(CC) $(CFLAGS) $(SAN) -o $@ $(SOURCES)

clean:
	rm -f $(TARGET)
