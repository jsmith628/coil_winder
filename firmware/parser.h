#ifndef _PARSER_H_
  #define _PARSER_H_

#define MODIFIERLENGTH 6
#define BUFFERLENGTH 64
#define MINIMUMSPACE 32

typedef struct {
  char c;
  float f;
} modifier;

typedef struct {
  char type;
  unsigned int number;
  modifier modifiers[MODIFIERLENGTH];
} command;


void display_warning(const char * type);


void display_warning(const char * type, const char * details);


void display_warning(const char * type, unsigned int specificType, const char * details);


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
