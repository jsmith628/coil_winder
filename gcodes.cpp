
#include <Arduino.h>
#include "gcodes.h"
#include "machine.h"

//STATE

enum {FEEDRATE_TIME,FEEDRATE_DIST} feed_mode = FEEDRATE_DIST;

float units = 1;//millimeters

//current axis position
float a_pos = 0;
float b_pos = 0;

//G codes define movement and interpretation commands

//Rapid move (A axis , B axis position, Speed, Feedrate)
void g0 (float a, float b, float s, float f) {

  Jobs next = {{NOOP_JOB, NOOP_JOB, NOOP_JOB, NOOP_JOB}};

  float turn_speed;
  switch(feed_mode) {
    case FEEDRATE_TIME: turn_speed = f*units; break;
    case FEEDRATE_DIST: turn_speed = s*f; break;
  }

  s *= units;

  next.jobs[0].frequency = (uint16_t) ((s / ROD_MM_PER_TURN) * FEED_STEPS_PER_TURN * FEED_MS);
  next.jobs[1].frequency = (uint16_t) ((s / ROD_MM_PER_TURN) * CLAMP_STEPS_PER_TURN * CLAMP_MS);
  next.jobs[2].frequency = (uint16_t) (((turn_speed * GEAR_1_TEETH) / GEAR_2_TEETH) * DRIVE_STEPS_PER_TURN * DRIVE_MS);

  float da = a - a_pos;
  float db = b - b_pos;

  next.jobs[0].dir = da<0 ? SET : da>0 ? UNSET : KEEP;
  next.jobs[1].dir = db<0 ? SET : db>0 ? UNSET : KEEP;
  next.jobs[2].dir = f<0 ? SET : f>0 ? UNSET : KEEP;

  for(byte i=0; i<3; i++) next.jobs[i].end.ty = COUNT;

  next.jobs[0].end.cond = (float) ((da / s) * next.jobs[0].frequency);
  next.jobs[1].end.cond = (float) ((da / s) * next.jobs[1].frequency);
  next.jobs[2].end.cond = next.jobs[2].end.cond;

}

//Linear interpolate (A axis position, B axis position, Speed | Feedrate)
void g1 (float a, float b, float s, float f) {}

//Dwell (P (millis) | S (seconds) )
void g4 (float p, float s) {}

//Programming in inches
void g20 () {units = 25.4;}

//Programming in millimeters
void g21 () {units = 1.0;}

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
void g94() {feed_mode = FEEDRATE_TIME;}

//Feedrate per revolution
void g95() {feed_mode = FEEDRATE_DIST;}

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
  next.jobs[3] = NOOP_JOB;
  Serial.println("Steppers Enabled");
  queue_jobs(next);
}

//Disable steppers
void m18() {
  Jobs next;
  for(byte i=0; i<3; i++) {
    next.jobs[i] = NOOP_JOB;
    next.jobs[i].en = UNSET;
  }
  next.jobs[3] = NOOP_JOB;
  Serial.println("Steppers Enabled");
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
