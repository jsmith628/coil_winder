
#include <Arduino.h>
#include "gcodes.h"
#include "machine.h"

//STATE

float a_pos = 0;
float b_pos = 0;

bool steppers_enabled[3] = {false, false, false};


//G codes define movement and interpretation commands

//Rapid move (A axis , B axis position, Speed, Feedrate)
void g0 (float a, float b, float s, float f /*rev/mm*/) {

  Job next = Job();

  float da = a - a_pos;
  float db = b - b_pos;

  next.dirs[0] = da < 0;
  next.dirs[1] = db < 0;
  next.dirs[2] = da < 0 ^ f < 0;

  for(byte i=0; i<3; i++){
    next.end[i].ty = COUNT;
    next.end[i].triggered = false;
    next.enabled[i] = steppers_enabled[i];
  }

  float drive_turn_per_a_turn = f * ROD_MM_PER_TURN;

  int n1 = (1 << TIMINGS_PRECISION);
  int n2 = (int) (drive_turn_per_a_turn * (1 << TIMINGS_PRECISION));
  if(n2<0) n2 *= -1;

  n1 *= FEED_STEPS_PER_TURN * GEAR_2_TEETH;
  n2 *= DRIVE_STEPS_PER_TURN * GEAR_1_TEETH;

  int g = gcd(n1, n2);
  n1 /= g;
  n2 /= g;

  next.ratio[0] = n1;
  next.ratio[1] = n1;
  next.ratio[2] = n2;

  float steps_per_mm = n2 * FEED_STEPS_PER_TURN / (float) ROD_MM_PER_TURN;

  next.frequency = (unsigned int) (s * steps_per_mm);

  next.end[0].cond = (unsigned int) (da * steps_per_mm);
  next.end[1].cond = (unsigned int) (db * steps_per_mm);
  next.end[2].cond = (unsigned int) (max(da,db) * steps_per_mm);

  queue_job(next);

}

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

//Feed until skip (A axis enable, B axis enable, Direction (0 = +, 1 = -), Speed)
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
  Job next = Job();
  for(byte i=0; i<3; i++) {
    steppers_enabled[i] = next.enabled[i] = true;
  }
  Serial.println("Steppers Enabled");
  queue_job(next);
}

//Disable steppers
void m18() {
  Job next = Job();
  for(byte i=0; i<3; i++) {
    steppers_enabled[i] = next.enabled[i] = false;
  }
  Serial.println("Steppers Disabled");
  queue_job(next);
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
  for(byte i=0; i<3; steppers_enabled[i++] = false);
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
