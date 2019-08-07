


#include <TMC2130Stepper.h>
// #include "timings.h"
#include "machine.h"
#include "gcodes.h"
#include "parser.h"

void setup() {
  parser_setup();
  machine_init();
}

void loop() {
  read_command();
  machine_loop();
}
