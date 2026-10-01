# The utils_math unit tests, on Google Test.
#
#   nix-shell -p gcc gnumake gtest --run 'make run'
#
# This replaces a Makefile that had bit-rotted in two ways, both of which made
# the suite silently unrunnable rather than failing:
#
#   * It built $(TOP)/utils_math.c, which moved to util/ at some point. The
#     suite therefore had nothing to test.
#   * It went through make/unittest.mk, which expects $(GTEST_DIR) to hold the
#     Google Test *sources* so it can compile gtest-all.cc itself. A packaged
#     gtest provides a built library instead, so the include path and the link
#     never resolved.
#
# Self-contained now, because unittest.mk is shared with nothing else that
# still works and fixing it would mean guessing at what else once used it.
# git has the old one.

TOP = ../..

SRC = $(TOP)/util/utils_math.c
TEST = unittest.cpp

# -fsingle-precision-constant matches how the firmware builds this file: the
# tests assert on float results, and letting a constant widen to double here
# would make them pass against arithmetic the board does not do.
# -Wno-error=cpp demotes exactly one warning: the nixpkgs gcc wrapper defines
# _FORTIFY_SOURCE itself, appended after these flags so -U cannot win, and
# glibc then warns that it needs optimisation -- which -Werror makes fatal.
# Demoting the #warning category is narrower than dropping -Werror, and
# narrower than raising -O0: these tests assert on exact float results and -O0
# is what keeps the arithmetic unrearranged.
CFLAGS = -O0 -g -std=gnu99 -Wall -Wdouble-promotion -Werror -Wno-error=cpp \
         -fsingle-precision-constant -I. -I$(TOP) -I$(TOP)/util

# c++17 because gtest 1.18 requires it; the old Makefile asked for c++11,
# which predates that and is part of why this suite stopped building.
#
# -fsingle-precision-constant matters here more than on the C side. Several of
# these functions -- utils_norm_angle_rad among them -- are static inline in
# utils_math.h, so they are compiled into *this* translation unit, not into
# utils_math.c. Without the flag, M_PI in `while (*angle < -M_PI)` is a
# double, a float holding -pi compares strictly less than it, the loop fires
# once, and the function returns +pi for an input of -pi: outside the
# "-pi <= angle < pi" its own header promises. Four tests fail that way, and
# they are right to -- the firmware builds this header with the flag, so the
# bug is in the harness and not in the tree.
CXXFLAGS = -O0 -g -std=c++17 -Wall -Wno-error=cpp \
           -fsingle-precision-constant \
           -I. -I$(TOP) -I$(TOP)/util

LDLIBS = -lgtest -lgtest_main -lpthread

all: test

utils_math.o: $(SRC) Makefile
	$(CC) $(CFLAGS) -c -o $@ $<

unittest.o: $(TEST) Makefile
	$(CXX) $(CXXFLAGS) -c -o $@ $<

test: utils_math.o unittest.o
	$(CXX) -o $@ utils_math.o unittest.o $(LDLIBS)

run: test
	./test

clean:
	rm -f test *.o

.PHONY: all run clean
