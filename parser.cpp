
#include "parser.h"
#include "gcodes.h"
#include <Arduino.h>

#define A 0
#define B 1
#define W 2
#define S 3
#define FR 4
#define P 5

command com;
char buffer[BUFFERLENGTH+1];

void display_warning(String type){
  Serial.print("Warning: ");
  Serial.println(type);
}

void display_warning(String type, String details){
  Serial.print("Warning: ");
  Serial.println(type);
  Serial.println(details);
}

void display_warning(String type, unsigned int specificType, String details){
  Serial.print("Warning: ");
  Serial.print(type);
  Serial.print(specificType);
  Serial.println(details);
}


void interpret_gcode(command c){
  if(c.type == 'G'){
    switch (c.number) {
      default:
        display_warning("G",c.number," is not a valid G command.");
        break;
      case 0:
        g0(c.modifiers[A].f, c.modifiers[B].f, c.modifiers[W].f, c.modifiers[S].f, c.modifiers[FR].f);
        break;
      case 1:
        g1(c.modifiers[A].f, c.modifiers[B].f, c.modifiers[W].f, c.modifiers[S].f, c.modifiers[FR].f);
        break;
      case 4:
        g4(c.modifiers[P].f, c.modifiers[S].f);
        break;
      case 20:
        g20();
        break;
      case 21:
        g21();
        break;
      case 28:
        g28((c.modifiers[A].f == c.modifiers[A].f), (c.modifiers[B].f == c.modifiers[B].f));
        break;
      case 31:
        g31((int8_t)c.modifiers[A].f,(int8_t)(c.modifiers[B].f!=0));
        break;
      case 50:
        g50(c.modifiers[S].f);
        break;
      case 52:
        g52(c.modifiers[A].f, c.modifiers[B].f, c.modifiers[W].f);
        break;
      case 90:
        g90();
        break;
      case 91:
        g91();
        break;
      case 92:
        g92(c.modifiers[A].f, c.modifiers[B].f, c.modifiers[W].f);
        break;
      case 94:
        g94();
        break;
      case 95:
        g95();
        break;
    }
  }else if(c.type == 'M'){
    switch (c.number) {
      default:
        display_warning("M",c.number," is not a valid G command.");
        break;
      case 0:
        m0();
        break;
      case 17:
        m17();
        break;
      case 18:
        m18();
        break;
      case 30:
        m30();
        break;
      case 82:
        m82();
        break;
      case 83:
        m83();
        break;
      case 92:
        m92(c.modifiers[A].f, c.modifiers[B].f, c.modifiers[W].f);
        break;
      case 98:
        m98();
        break;
      case 99:
        m99();
        break;
      case 112:
        m112();
        break;
      case 114:
          if((c.modifiers[A].c != 0)||(c.modifiers[B].c != 0)){
            m114((c.modifiers[A].f != 0), (c.modifiers[B].f != 0));
          }else{
            m114();
          }
        break;
      case 120:
        m120();
        break;
      case 121:
        m121();
        break;
      case 125:
        m125(c.modifiers[A].f, c.modifiers[B].f, c.modifiers[W].f);
        break;
      case 201:
        m201(c.modifiers[S].f);
        break;
      case 203:
        m203(c.modifiers[FR].f);
        break;
      case 204:
        m204(c.modifiers[A].f, c.modifiers[B].f, c.modifiers[W].f);
        break;
      case 500:
        m500();
        break;
      case 501:
        m501();
        break;
      case 503:
        m503();
        break;
    }

  }else{
    display_warning("Invalid GCODE", "Commands must start with G or M.");
  }
}


void parse(size_t s, char* buf){

  command c;
  for (int x = 0; x < MODIFIERLENGTH; x++){
      c.modifiers[x] = {0,NAN};
  }

  // Evaluate command

  if(*buf == 'G' || *buf == 'M'){
    c.type = *(buf++);
    int x = 1;
    int y = 0;
    c.number = atoi(buf);
    while (x<s && isDigit(*buf)) {
      x++;
      buf++;
    }

    // Evaluate parameters

    //A, B, S, F, D/P, R

    while (x < s){
      if(*buf = ';')break;
      if (isAlpha(*buf)){
        if(*buf == 'G' || *buf == 'M'){
          interpret_gcode(c);
          parse(s-x, buf);
          return;
        }
        modifier m;
        m.c = *(buf++);
        y = ++x;
        m.f = atof(buf);
        while ((isDigit(*buf)||*buf=='.'||*buf=='-')&& y<s){
           y++;
           buf++;
        }
        switch (m.c) {
          case 'A':
            c.modifiers[A] = m;
            break;
          case 'B':
            c.modifiers[B] = m;
            break;
          case 'W':
            c.modifiers[W] = m;
            break;
          case 'S':
            c.modifiers[S] = m;
            break;
          case 'F':
            c.modifiers[FR] = m;
            break;
          case 'P':
            c.modifiers[P] = m;
            break;
          }
          x = y;
      }else{
        x++;
        buf++;
      }
    }

    interpret_gcode(c);

  } else {
      c.type = 0;
      c.number = 0;
      interpret_gcode(c);
  }
}


bool read_command(){

  static bool xon = true;

  if(!xon && space_in_queue() >= MINIMUMSPACE){
    Serial.print((char) 17);
    // Serial.println("Queue open!");
    xon = true;
  }

  if(Serial.available()){
    if(space_in_queue() > 0) {
      size_t len = Serial.readBytesUntil('\n', buffer, BUFFERLENGTH);
      buffer[len] = '\0';
      parse(len, buffer);
      Serial.println(space_in_queue());
    }
  }

  if(xon && space_in_queue() < MINIMUMSPACE){
    // Serial.println("Queue full!");
    Serial.print((char) 19);
    xon = false;
  }
}

void parser_setup(){

  Serial.begin(115200);
  while (!Serial){}
}
