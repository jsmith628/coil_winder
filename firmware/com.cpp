
#include "com.h"
#include "ascii_control.h"
#include "control.h"
#include <Arduino.h>
#include "queue.h"

void com_init() {
  Serial.begin(COM_BAUD);
  Serial.setTimeout(0);
  while(!Serial) {}

  Serial.print(ACK);
}

bool in_comment = false;
Queue<char, COM_BUFFER_ORDER> text_buffer;
uint16_t buffered_commands = 0;

void backspace() { text_buffer.pop_top(); }

void newline() {
  in_comment = false;
  text_buffer.push_bottom('\n');
  buffered_commands++;
}

void(* const PROGMEM control_callbacks[0x20])(void) = {
  NULL,                NULL,            NULL,        interrupt, //0x00-0x03
  end_of_transmission, enquiry,         acknowledge, bell,      //0x04-0x07
  backspace,           NULL,            newline,     NULL,      //0x08-0x0B
  NULL,                NULL,            NULL,        NULL,      //0x0C-0x0F
  NULL,                NULL,            pause,       NULL,      //0x10-0x13
  resume,              neg_acknowledge, NULL,        NULL,      //0x14-0x17
  cancel,              NULL,            NULL,        NULL,      //0x18-0x1B
  quit,                NULL,            NULL,        NULL,      //0x1C-0x1F
};

bool xon = true;

void enable_input() {
  if(!xon && buffered_commands==0) {
    xon = true;
    Serial.write(XON);
    // Serial.println("Queue Open!");
  }
}

void disable_input() {
  if(xon) {
    xon = false;
    Serial.write(XOFF);
    // Serial.println("Queue Full!");
  }
}

bool command_available(size_t max_size) {return buffered_commands>0 || text_buffer.count()>=max_size;}

size_t next_command(char* dest, size_t max_size) {
  size_t len = 0;

  //only get the next command if one is available (and dest has space)
  if(command_available(max_size) && max_size>0) {

    //copy the command string
    while(len<(max_size-1) && text_buffer.count()>0) {
      //write to the destination string
      char x = text_buffer.pop_top();
      dest[len++] = x;

      //if we've reached a newline
      if(x=='\n'){
        if(buffered_commands>0) buffered_commands--;
        break;
      }
    }

    dest[len++] = '\0'; //make sure the string is NUL-terminated
    // Serial.print(buffered_commands);
    // Serial.print(" ");
    // Serial.println(text_buffer.available());
  }

  return len;
}

void com_loop() {
  for(byte i=0; i<BUF_PAGE_SIZE && Serial.available() && text_buffer.available()>1; i++) {
    //read the next char in transmission
    char x = (char) Serial.read();

    //if we actually got an ASCII character
    if(x>=0) {
      if(x<0x20) {//if x is a control character
        void(*callback)(void) = (void(*)(void)) pgm_read_ptr_near(&control_callbacks[x]);
        if(callback!=NULL) { (callback)(); }
      } else if(x==';'){//if the rest of the line is supposed to be a comment
        in_comment = true;
      } else if(x==DEL) {//DEL should backspace
        backspace();
      } else if(!in_comment && x!=' ') {

        //if we have a G or an M, insert a newline to signify the end of the last command
        if((x=='G' || x=='M') && text_buffer.count()>0) newline();

        //read the character into the buffer
        text_buffer.push_bottom(x);
      }
    }
  }
}
