# The Lua script engine, as an alternative to LispBM.
#
# Selected with USE_LUA=1, which requires USE_LISPBM=0: the two engines are
# mutually exclusive in one build and nothing picks between them at runtime.
# On an STM32F405 that is not a style choice -- the firmware with LispBM is
# 95.5% of the app partition, and dropping it is what makes room.
#
#   make fw_75_100_V2 USE_LISPBM=0 USE_LUA=1
#
# LUA_32BITS is set in script/lua/luaconf.h rather than here: Lua defines it
# itself and a command-line define collides with that.
#
# The vendored Lua is a copy of vesc_express/main/lua, unmodified apart from
# luaconf.h, so the same interpreter runs on both a controller and a display.

LUASRC = $(wildcard script/lua/*.c) \
         script/script_pack.c \
         script/script_lua.c \
         script/script_queue.c \
         script/script_alloc.c \
         script/luaif.c \
         script/lua_vesc_mc.c \
         script/lua_vesc_conf.c \
         script/lua_vesc_can.c

LUAINC = script script/lua

# Lua's limit on nested C calls, lowered from its default of 200.
#
# This is the guard that is supposed to stop runaway recursion before it
# overflows the C stack, and at 200 it cannot do its job on this part.
# Measured on the simulated F405 in tests/qemu, each nested pcall level costs
# 464 bytes of thread stack over a 2,088-byte baseline, so the default limit
# wants
#
#   2088 + 200 * 464 = 94,888 bytes
#
# of stack for one thread. All of CCM is 65,536. The hardware stack therefore
# dies long before Lua's own check fires, and the measured difference is
# stark: at the default a 40-deep pcall chain takes a HardFault with
# BFSR.STKERR, and with the limit lowered the same script gets a catchable
# "C stack overflow" error and the firmware carries on.
#
# Pick a value for the engine thread's working area with
#
#   LUAI_MAXCCALLS = (usable_stack - 2088) / 464
#
# less some margin, where usable_stack is the working area minus
# sizeof(thread_t). 16 suits a 12 KB working area: 2088 + 16 * 464 = 9,512,
# leaving about 2.8 KB spare. Raising the working area is what buys a higher
# limit; there is no free way to get one.
#
# llimits.h guards the default with #if !defined, so defining it here is the
# documented way in. (LUA_32BITS is different -- Lua defines that one itself
# and it has to go in luaconf.h.)
LUAOPT = -DLUAI_MAXCCALLS=16

# Nothing references the engine yet, and ChibiOS links with
# -ffunction-sections --gc-sections, so a USE_LUA=1 build is the same size as
# one without it: every Lua function is discarded. That is why the cost was
# measured from the objects rather than from the linked image.
#
# Measured on 75_100_V2, ARM Thumb-2:
#
#   Lua core, 26 objects   text  103,491
#   engine glue, 2 objects text    3,610
#   free with LispBM out         186,344
#   remaining                     79,243
#
# The same core measured 132,752 cross-compiled for RISC-V on the P4, so
# Thumb-2 is 22% denser here. The object sum is an upper bound: gc-sections
# will drop whatever the bindings never reach.
