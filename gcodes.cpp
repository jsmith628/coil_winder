
#include <Arduino.h>
#include "gcodes.h"
#include "machine.h"

//STATE

float a_pos = 0;
float b_pos = 0;

//G codes define movement and interpretation commands

//Rapid move (A axis , B axis position, Speed, Feedrate)
void g0 (float a, float b, float s, float f /*rev/mm*/) {}

//Linear interpolate (A axis position, B axis position, Speed | Feedrate)
void g1 (float a, float b, float s, float f) {}

//Dwell (P (millis) | S (seconds) )
void g4 (float p, float s) {}

//Programming in inches
void g20 () {}

//Programming in millimeters
void g21 () {}

//Home axis (A final position, B final position, Speed)
void g28 (float a, float b, float s) {}

//Feed until skip (A axis enable, B axis enable, Speed)
void g31 (bool a, bool b, float s) {}

//Single point threading, non-cycle (for cycle, use G76) (A axis position, B axis position, Spindle speed, Feedrate)
void g32 (float a, float b, float s, float f) {}

//Define maximum spindle Speed (Speed)
void g50 (float s) {}

//Local coordinates, defines program zero to a new location (A position 0, B position 0)
void g52 (float a, float b) {}

//Repetitive threading cycle (A axis position, B axis position, Spindle speed, Feedrate, Repititions, Symmetrical)
void g76 (float a, float b, float s, float f, int r, bool p) {}

//Absolute positioning (position defined from machine zero)
void g90() {}

//Incremental positioning (position defined relative to previous position)
void g91() {}

//Feedrate per minute
void g94() {}

//Feedrate per revolution
void g95() {}

//M codes define miscellaneous commands

//Unconditional stop
void m0() {}

//Spindle on, CW (Speed)
void m3(float s) {}

//Spindle on, CCW (Speed)
void m4(float s) {}

//Spindle stop
void m5() {}

//Enable steppers
void m17() {
  Jobs next;
  for(byte i=0; i<3; i++) {
    next.jobs[i] = NOOP_JOB;
    next.jobs[i].en = SET;
  }
  Serial.println("Steppers Enabled");
  queue_jobs(next);
}

void m17(bool a, bool b, bool c){
  Jobs next;
  if(a){
    next.jobs[0] = NOOP_JOB;
    next.jobs[0].en = SET;
  }
  if(a){
    next.jobs[1] = NOOP_JOB;
    next.jobs[1].en = SET;
  }
  if(a){
    next.jobs[2] = NOOP_JOB;
    next.jobs[2].en = SET;
  }
    next.jobs[3] = NOOP_JOB;

  Serial.print("Steppers ");
  Serial.print((a ?"A " : ""));
  Serial.print((b ?"B " : ""));
  Serial.print((c ?"C " : ""));
  Serial.println("Enabled");
  queue_jobs(next);
}

//Disable steppers
void m18() {
  Jobs next;
  for(byte i=0; i<3; i++) {
    next.jobs[i] = NOOP_JOB;
    next.jobs[i].en = UNSET;
  }
  Serial.println("Steppers Disabled");
  queue_jobs(next);
}

void m18(bool a, bool b, bool c) {
  Jobs next;
  if(a){
    next.jobs[0] = NOOP_JOB;
    next.jobs[0].en = UNSET;
  }
  if(a){
    next.jobs[1] = NOOP_JOB;
    next.jobs[1].en = UNSET;
  }
  if(a){
    next.jobs[2] = NOOP_JOB;
    next.jobs[2].en = UNSET;
  }
  next.jobs[3] = NOOP_JOB;

  Serial.print("Steppers ");
  Serial.print((a ?"A " : ""));
  Serial.print((b ?"B " : ""));
    Serial.print((c ?"C " : ""));
  Serial.println("Disabled");
  queue_jobs(next);
}

//End of program, return to program top
void m30() {}

//Subprogram call
void m98() {}

//Subprogram end
void m99() {}

//Emergency stop, immediately stop program and disable all steppers
void m112() {
  digitalWrite(EN_FEED, FEED_INVERT_EN ? HIGH : LOW);
  digitalWrite(EN_CLAMP, CLAMP_INVERT_EN ? HIGH : LOW);
  digitalWrite(EN_DRIVE, DRIVE_INVERT_EN ? HIGH : LOW);
  clear_jobs();
}

//Current position
float m114() {}
void m114(bool a, bool b) {}

//Park (A axis position, B axis postion)
void m125(float a, float b) {}

//Enable software endstops
void m120() {}

//Disable software endstops
void m121() {}

//Set max feedrate (Feedrate)
void m203(float f) {}

//Save settings to EEPROM
void m500() {}

//Load settings from EEPROM
void m501() {}

//Read out settings from EEPROM
void m503() {}
