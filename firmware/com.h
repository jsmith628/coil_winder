
#ifndef _COM_H_
#define _COM_H_

#include <Arduino.h>

#define COM_BUFFER_ORDER 10
#define COM_BUFFER_SIZE (1<<COM_BUFFER_ORDER)
#define COM_BAUD 115200

void com_init();
void com_loop();

void enable_input();
void disable_input();

bool command_available(size_t max_size);
size_t next_command(char* dest, size_t max_size);

#endif
