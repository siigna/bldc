# Tests the ADC channel mapping in script/lua_vesc_io.c.
#
# adc_index() is cut out of the real source rather than copied, so it cannot
# drift from what the firmware compiles. The extraction is checked: if the
# marker moves, the build fails rather than testing an empty file.
#
#   nix-shell -p gcc gnumake --run 'make run'

TOP = ../..
SRC = $(TOP)/script/lua_vesc_io.c

CFLAGS = -std=gnu99 -O1 -g -Wall -Wextra -I.

all: test

adc_index.inc: $(SRC) Makefile
	@sed -n '/^static int adc_index(int channel, bool \*aliased) {$$/,/^}$$/p' \
		$(SRC) > $@
	@test -s $@ || { echo "adc_index.inc is empty: the function signature in"\
		" $(SRC) changed, so the extraction marker needs updating"; \
		rm -f $@; exit 1; }
	@grep -q 'ADC_IND_EXT8' $@ || { echo "adc_index.inc looks truncated"; \
		rm -f $@; exit 1; }

test: main.c adc_index.inc
	gcc $(CFLAGS) -o $@ main.c

run: test
	./test

clean:
	rm -f test adc_index.inc

.PHONY: all run clean
