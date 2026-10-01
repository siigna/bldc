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
         script/script_queue.c

LUAINC = script script/lua

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
