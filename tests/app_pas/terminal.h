#ifndef TERMINAL_H
#define TERMINAL_H
void terminal_register_command_callback(const char* command, const char *help,
		const char *arg_names, void(*cbf)(int argc, const char **argv));
void terminal_unregister_callback(void(*cbf)(int argc, const char **argv));
#endif
