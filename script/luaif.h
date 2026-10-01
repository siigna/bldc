/*
	Copyright 2025 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	The VESC firmware is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * The firmware side of the Lua engine: a thread, the arena it allocates from,
 * and the handful of entry points the rest of the firmware calls.
 *
 * This is the Lua counterpart to lispBM/lispif.h, and the two are alternatives
 * rather than layers -- USE_LUA requires USE_LISPBM=0. The engine does not
 * impersonate LispBM: the call sites carry an arm for each engine, because
 * only two of lispif.h's twenty-one entry points have anything to do with
 * running a script, and pretending otherwise would mean implementing nineteen
 * functions about lbm_uint and const heaps that have no meaning here.
 *
 * Everything below is safe to call with the engine stopped or never started;
 * each one says so where it is not obvious.
 */

#ifndef SCRIPT_LUAIF_H_
#define SCRIPT_LUAIF_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/*
 * Start the engine thread. Called from the blocking thread rather than main,
 * which is what lispif_init does and for the same reason: main's stack cannot
 * carry an interpreter being built.
 *
 * Starting the thread is not the same as running a script. The thread loads
 * whatever is in flash and reports what it finds; an empty slot is a normal
 * outcome, not an error.
 */
void luaif_init(void);

/*
 * Stop the running script and release the interpreter, then block until the
 * engine thread has actually let go.
 *
 * This has to be synchronous because of who calls it: flash_helper erases the
 * very pages the script's source and imports are read from, so returning
 * while the engine still holds a pointer into them would leave it parsing a
 * page mid-erase. The stop is requested through the instruction hook, so a
 * script spinning in a loop still unwinds and its finalisers still run.
 */
void luaif_stop(void);

/*
 * Reload from flash and start again. Returns true only if the script's main
 * chunk ran to completion: a missing or corrupt container, an interpreter
 * that would not fit, and a script that raised all return false. A script
 * that raised still leaves the interpreter in place, since it may have
 * registered handlers before it failed. With print set, the outcome goes to
 * the terminal, which is what the REPL and the upload path want.
 */
bool luaif_restart(bool print, bool load_code);

// How many times the engine has been (re)started since boot, for GET_STATS.
int luaif_get_restart_cnt(void);

/*
 * Deliver a CAN frame to the script, if it asked for them.
 *
 * Called from the CAN receive path, which is a thread but a hot one, so this
 * only copies into a queue and never touches the interpreter. Frames that
 * arrive with the queue full are dropped and counted rather than blocking the
 * bus; see script_queue_dropped.
 */
void luaif_process_can(uint32_t can_id, uint8_t *data8, int len, bool is_ext);

/*
 * Handle a COMM_LISP_* packet, with data[0] the packet id.
 *
 * The container format and these packet ids are shared with LispBM
 * deliberately, so VESC Tool's existing upload, erase and REPL carry Lua
 * unchanged. Code upload is not here at all: COMM_LISP_READ_CODE,
 * _WRITE_CODE and _ERASE_CODE are handled in commands.c against
 * flash_helper, with no engine involved, so they already work in a USE_LUA
 * build.
 *
 * What is here is the five that need an engine: SET_RUNNING, GET_STATS,
 * REPL_CMD and STREAM_CODE. RMSG is lisp-specific -- it dispatches to a
 * lisp recv-rmsg channel that has no Lua equivalent yet -- and is answered
 * rather than ignored, so the sender sees a refusal instead of a timeout.
 */
void luaif_process_cmd(unsigned char *data, unsigned int len,
		void (*reply_func)(unsigned char *data, unsigned int len));

// Deliver a COMM_CUSTOM_APP_DATA payload, same queueing rules as CAN.
void luaif_process_custom_app_data(unsigned char *data, unsigned int len);

/*
 * Let the script veto a shutdown. Returns with no opinion -- and no delay --
 * if no script is running, since this sits in the power-off path.
 */
void luaif_process_shutdown(void);

/*
 * The prefix commands.c stamps on script output. This is a *format string*
 * consumed as sprintf(buf, prefix, "%s"), not a label -- LispBM's equivalent
 * is a user-settable buffer. Currently "%s", i.e. no prefix.
 */
char *luaif_print_prefix(void);

#endif /* SCRIPT_LUAIF_H_ */
