TARGET = test
LIBS = -lm
CC = gcc
CFLAGS = -O2 -g -Wall -Wextra -Wundef -std=gnu99 \
         -I. -I../../imu -I../../util -I../.. -DNO_STM32
SOURCES = main.c ../../imu/ahrs.c ../../util/utils_math.c
OBJECTS = $(notdir $(SOURCES:.c=.o))

.PHONY: default all clean run

default: $(TARGET)
all: default

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

%.o: ../../imu/%.c
	$(CC) $(CFLAGS) -c $< -o $@

%.o: ../../util/%.c
	$(CC) $(CFLAGS) -c $< -o $@

$(TARGET): $(OBJECTS)
	$(CC) $(OBJECTS) -Wall $(LIBS) -o $@

clean:
	rm -f $(OBJECTS) $(TARGET)

run: $(TARGET)
	./$(TARGET)
