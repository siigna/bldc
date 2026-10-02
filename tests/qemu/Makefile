# Builds a ChibiOS test image for qemu-system-arm -M olimex-stm32-h405, which
# models an STM32F405 -- the part the firmware actually targets.
#
# Kernel only: no HAL, no STM32 LLD. The clock tree is whatever QEMU provides
# and SysTick is a Cortex-M core peripheral, so none of the board's peripheral
# init has to be simulated for the kernel to run.
#
#   nix-shell -p gcc-arm-embedded gnumake qemu --run 'make run'
#   nix-shell -p gcc-arm-embedded gnumake qemu --run './run.sh'

TEST   ?= test_kernel

TOP     = ../..
CHIBIOS = $(TOP)/ChibiOS_3.0.5

include $(CHIBIOS)/os/rt/rt.mk
include $(CHIBIOS)/os/rt/ports/ARMCMx/compilers/GCC/mk/port_v7m.mk
include $(CHIBIOS)/os/common/ports/ARMCMx/compilers/GCC/mk/startup_stm32f4xx.mk

CSRC   = $(KERNSRC) $(PORTSRC) $(STARTUPSRC) qrt.c $(TEST).c

ASMSRC = $(STARTUPASM) $(PORTASM)

# $(TOP) is on the include path for bldc's own chconf.h, so the kernel here is
# configured exactly as it is in firmware -- same tick rate, same debug options.
INCDIR := . $(TOP) $(KERNINC) $(PORTINC) $(STARTUPINC) \
          $(CHIBIOS)/os/hal/osal/rt $(CHIBIOS)/os/hal/include

# test_engine links the real engine: the vendored Lua core, the container
# parser, and the CCM arena allocator. syscalls.c is what make/fw.mk gives the
# firmware for newlib's _sbrk, so malloc behaves here as it does on the board.
ENGINE_SRC = $(wildcard $(TOP)/script/lua/*.c) \
             $(TOP)/script/script_pack.c \
             $(TOP)/script/script_lua.c \
             $(TOP)/script/script_alloc.c \
             $(CHIBIOS)/os/various/syscalls.c

# Every binding set luaif.c registers, with the fakes they compile against.
#
# One list rather than a per-test one, because test_luaif links luaif.c and so
# needs all of them: adding a binding set and registering it broke that test
# twice, each time caught only by run.sh's build check. Now adding one here
# fixes both at once.
BINDING_SRC = $(TOP)/script/lua_vesc_mc.c \
              $(TOP)/script/lua_vesc_conf.c \
              $(TOP)/script/lua_vesc_can.c
# fake_io.c stands in for lua_vesc_io.c, which cannot be linked here: it
# reaches the ChibiOS HAL through conf_general.h and these images have no HAL.
# See the note in fake_io.c, and tests/lua_adc for what does cover it.
BINDING_FAKES = fake/fake_mc.c fake/fake_conf.c fake/fake_can.c fake/fake_io.c
BINDING_INC = $(TOP)/script $(TOP)/script/lua $(TOP)/util

ifeq ($(TEST),test_engine)
  CSRC += $(ENGINE_SRC)
  INCDIR += $(TOP)/script $(TOP)/script/lua
endif

# test_bindings compiles script/lua_vesc_mc.c against the fakes in fake/,
# which sit earlier on the include path than the real mc_interface.h and
# timeout.h. The bindings therefore have no test seam of their own.
ifeq ($(TEST),test_bindings)
  CSRC += $(ENGINE_SRC) \
          $(TOP)/script/lua_vesc_mc.c \
          fake/fake_mc.c
  # fake/ must come FIRST: $(TOP) also has a real timeout.h, and whichever
  # directory is earlier on the path wins.
  INCDIR := fake $(INCDIR) $(TOP)/script $(TOP)/script/lua
endif

# test_conf compiles lua_vesc_conf.c against the fakes, including the real
# datatypes.h so the configuration structs are the firmware's own -- the
# offsets in the generated table have to mean something.
ifeq ($(TEST),test_conf)
  CSRC += $(ENGINE_SRC) \
          $(TOP)/script/lua_vesc_conf.c \
          fake/fake_mc.c \
          fake/fake_conf.c
  INCDIR := fake $(INCDIR) $(TOP)/script $(TOP)/script/lua
endif

# demo is not a test: it prints a console session for showing what the engine
# does. Same sources as test_script.
ifeq ($(TEST),demo)
  CSRC += $(ENGINE_SRC) \
          $(TOP)/script/lua_vesc_mc.c \
          fake/fake_mc.c \
          fake/fake_io.c
  INCDIR := fake $(INCDIR) $(BINDING_INC) .
endif

# test_script runs script/examples/bench_throttle.lua. The source is embedded
# from the real file by the rule below, so the test cannot drift from what
# gets uploaded. The io bindings here are the stand-ins in fake_io.c, not
# script/lua_vesc_io.c -- see the note in that file.
ifeq ($(TEST),test_script)
  CSRC += $(ENGINE_SRC) \
          $(TOP)/script/lua_vesc_mc.c \
          fake/fake_mc.c \
          fake/fake_io.c
  INCDIR := fake $(INCDIR) $(BINDING_INC) .
endif

# test_can compiles lua_vesc_can.c against a fake comm_can whose status caches
# the test fills in. It links lua_vesc_mc.c too, only for the timeout counter
# it shares with fake_mc -- the point being to prove the CAN setters do *not*
# touch it.
ifeq ($(TEST),test_can)
  CSRC += $(ENGINE_SRC) \
          $(TOP)/script/lua_vesc_can.c \
          $(TOP)/script/lua_vesc_mc.c \
          fake/fake_mc.c \
          fake/fake_can.c \
          fake/fake_conf.c
  INCDIR := fake $(INCDIR) $(TOP)/script $(TOP)/script/lua $(TOP)/util
endif

# test_luaif adds the firmware adapter and the queue it drains. LUAIF_HOST_TEST
# swaps only flash_helper and the terminal for the stubs in luaif_host.h; the
# kernel, the engine, the arena and the queue are all the real thing.
ifeq ($(TEST),test_luaif)
  CSRC += $(ENGINE_SRC) \
          $(TOP)/script/script_queue.c \
          $(TOP)/script/luaif.c \
          $(TOP)/util/buffer.c \
          $(BINDING_SRC) \
          $(BINDING_FAKES)
  # fake/ first: $(TOP) also has a real timeout.h, and the earlier directory
  # on the path wins.
  INCDIR := fake $(INCDIR) $(BINDING_INC)
  # Without this luaif.c takes its firmware include path and reaches
  # flash_helper.h, which needs the HW_SOURCE/HW_HEADER defines a board build
  # supplies.
  TESTDEFS += -DLUAIF_HOST_TEST
endif


LDSCRIPT = $(CHIBIOS)/os/common/ports/ARMCMx/compilers/GCC/ld/STM32F405xG.ld

# -DSTM32F405xx is what lets cmparams.h skip including a board.h.
# USE_FPU=hard and these two stack sizes are what make/fw.mk uses.
ARCH   = -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16
DEFS   = -DCORTEX_USE_FPU=TRUE -DSTM32F405xx \
         -DTHUMB_PRESENT -DTHUMB_NO_INTERWORKING

# -MMD -MP emits a .d per object listing the headers it used, pulled in at the
# bottom. Without it, editing a header rebuilds nothing: a struct in fake/ was
# changed once and only half the objects picked it up, so two translation units
# disagreed about a field's offset and the test failed with values that looked
# like a logic bug.
CFLAGS = $(ARCH) $(DEFS) -O2 -g -std=gnu99 -ffreestanding -MMD -MP \
         -fomit-frame-pointer -falign-functions=16 \
         -ffunction-sections -fdata-sections -fsingle-precision-constant \
         -Wall -Wextra $(TESTDEFS) $(CFLAGS_EXTRA) $(patsubst %,-I%,$(INCDIR))

# ChibiOS's .s files are macro-heavy and lowercase .s is not preprocessed
# unless asked, so assembly needs its own rule rather than riding on CFLAGS.
AFLAGS = $(ARCH) $(DEFS) -g -MMD -MP -x assembler-with-cpp $(patsubst %,-I%,$(INCDIR))

# -specs=nosys.specs is what make/fw.mk uses to satisfy newlib's syscall
# stubs; ChibiOS's syscalls.c supplies _sbrk_r over __heap_base__ beside it.
LDFLAGS = $(ARCH) -nostartfiles -specs=nosys.specs -T$(LDSCRIPT) \
          -L$(STARTUPLD) \
          -L$(CHIBIOS)/os/common/ports/ARMCMx/compilers/GCC \
          -Wl,--defsym=__process_stack_size__=0x800 \
          -Wl,--defsym=__main_stack_size__=0x800 \
          -Wl,--gc-sections,--no-warn-rwx-segments,-Map=$(TEST).map

# Object directory keyed on the extra flags. Without this, changing
# CFLAGS_EXTRA between builds silently reuses objects compiled with the old
# value -- a sweep over a -D parameter then reports the first build's number
# for every point in the sweep, which looks like a flat result rather than a
# broken one.
FLAGKEY := $(shell printf '%s' "$(CFLAGS_EXTRA)" | cksum | cut -d' ' -f1)
OBJDIR  = obj/$(TEST)-$(FLAGKEY)
OBJ    = $(addprefix $(OBJDIR)/,$(notdir $(CSRC:.c=.o)) $(notdir $(ASMSRC:.s=.o)))
VPATH  = $(sort $(dir $(CSRC) $(ASMSRC)))

QEMU     ?= qemu-system-arm
QEMUFLAGS = -M olimex-stm32-h405 -cpu cortex-m4 -nographic \
            -semihosting --semihosting-config enable=on,target=native \
            -serial mon:stdio

all: $(TEST).elf

$(OBJDIR):
	@mkdir -p $(OBJDIR)

# The example script, as a C string literal. Generated rather than pasted so
# the test runs the file that actually ships; the guard catches an empty or
# truncated conversion instead of testing nothing.
bench_throttle.inc: $(TOP)/script/examples/bench_throttle.lua Makefile
	@sed -e 's/\\/\\\\/g' -e 's/"/\\"/g' -e 's/^/"/' -e 's/$$/\\n"/' $< > $@
	@test -s $@ || { echo "bench_throttle.inc is empty"; rm -f $@; exit 1; }
	@grep -q 'M.step' $@ || { echo "bench_throttle.inc looks truncated"; \
		rm -f $@; exit 1; }

$(OBJDIR)/test_script.o: bench_throttle.inc
$(OBJDIR)/demo.o: bench_throttle.inc

$(OBJDIR)/%.o: %.c | $(OBJDIR)
	@arm-none-eabi-gcc $(CFLAGS) -c -o $@ $<

$(OBJDIR)/%.o: %.s | $(OBJDIR)
	@arm-none-eabi-gcc $(AFLAGS) -c -o $@ $<

$(TEST).elf: $(OBJ)
	@arm-none-eabi-gcc $(LDFLAGS) -o $@ $(OBJ) -lm
	@arm-none-eabi-size $@

run: $(TEST).elf
	@timeout 60 $(QEMU) $(QEMUFLAGS) -kernel $(TEST).elf

clean:
	rm -rf obj *.elf *.map bench_throttle.inc

-include $(OBJ:.o=.d)

.PHONY: all run clean
