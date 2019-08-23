
#include "com.h"
#include "machine.h"
#include "parser.h"

void setup() {
  com_init();
  parser_setup();
  machine_init();
}

void loop() {
  com_loop();
  read_command();
  machine_loop();
}
