#ifndef _PARSER_H_
  #define _PARSER_H_

#include <WString.h>

#define MODIFIERLENGTH 6
#define BUFFERLENGTH 64
#define MINIMUMSPACE 2

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
