
#include "com.h"
#include "ascii_control.h"
#include "control.h"
#include <Arduino.h>
#include "queue.h"

void com_init() {

  int mode;
  switch(COM_PARITY&3) {
    case 0:
      switch(COM_DATA_BITS) {
        case 5: mode = COM_TWO_STOPB?SERIAL_5N2:SERIAL_5N1; break;
        case 6: mode = COM_TWO_STOPB?SERIAL_6N2:SERIAL_6N1; break;
        case 7: mode = COM_TWO_STOPB?SERIAL_7N2:SERIAL_7N1; break;
        case 8:
        default: mode = COM_TWO_STOPB?SERIAL_8N2:SERIAL_8N1; break;
      }
      break;
    case 1:
      switch(COM_DATA_BITS) {
        case 5: mode = COM_TWO_STOPB?SERIAL_5O2:SERIAL_5O1; break;
        case 6: mode = COM_TWO_STOPB?SERIAL_6O2:SERIAL_6O1; break;
        case 7: mode = COM_TWO_STOPB?SERIAL_7O2:SERIAL_7O1; break;
        case 8:
        default: mode = COM_TWO_STOPB?SERIAL_8O2:SERIAL_8O1; break;
      }
    case 2:
      switch(COM_DATA_BITS) {
        case 5: mode = COM_TWO_STOPB?SERIAL_5E2:SERIAL_5E1; break;
        case 6: mode = COM_TWO_STOPB?SERIAL_6E2:SERIAL_6E1; break;
        case 7: mode = COM_TWO_STOPB?SERIAL_7E2:SERIAL_7E1; break;
        case 8:
        default: mode = COM_TWO_STOPB?SERIAL_8E2:SERIAL_8E1; break;
      }
  }

  Serial.begin(COM_BAUD, mode);
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
  // Serial.write('\n');
  // Serial.println(buffered_commands);
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
    // Serial.println(buffered_commands);
    // Serial.print(" ");
    // Serial.print(dest);
    // Serial.print(" ");
    // Serial.println(text_buffer.available());
  }

  return len;
}

void com_loop() {
  for(size_t i=0; i<BUF_PAGE_SIZE && Serial.available() && text_buffer.available()>2; i++) {
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
