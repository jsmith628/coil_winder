#ifndef _PARSER_H_
  #define _PARSER_H_

#include <WString.h>

#define MODIFIERLENGTH 6
#define BUFFERLENGTH 60
#define A 0
#define B 1
#define W 2
#define S 3
#define FR 4
#define P 5

typedef struct {
  char c;
  float f;
} modifier;

typedef struct {
  char type;
  unsigned int number;
  modifier modifiers[MODIFIERLENGTH];
} command;


void display_warning(String type);


void display_warning(String type, String details);


void display_warning(String type, unsigned int specificType, String details);


void interpret_gcode(command c);


void parse(String g);


bool read_command();


void parser_setup();

#endif

// void setup(){
//   parser_setup();
// }
// void loop(){
//
//     if (Serial.available()){
//         read_command();
//     }
// }
