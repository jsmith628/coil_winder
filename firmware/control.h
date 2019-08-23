
#ifndef _CONTROL_H_
#define _CONTROL_H_

#include <stdint.h>
#include <stdbool.h>

//control functions

//clears queue and sends INTERRUPT code once current command is complete
void interrupt();

//queues an M30
void end_of_transmission();

//sends an ACK or NAK depending on whether the queue can accept stuffs
void enquiry();

//run when receiving an ACK
void acknowledge();

//makes a noise
void bell();

//pauses machine excecution
void pause();

//resumes machine excecution from pause
void resume();

//run when receiving an NAK
void neg_acknowledge();

//cancels the last command put into the queue (assuming it hasn't started yet)
void cancel();

//runs an emergency stop and echos the QUIT code
void quit();

#endif
