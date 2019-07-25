
#include <Arduino.h>
#include "gcodes.h"
#include "machine.h"

#define DEFAULT_SPEED 1
#define FEED_STEPS_PER_MM (FEED_STEPS_PER_TURN * FEED_MS * (FEED_DEDGE?1.0:2.0) / (float) ROD_MM_PER_TURN)
#define CLAMP_STEPS_PER_MM (CLAMP_STEPS_PER_TURN * CLAMP_MS * (CLAMP_DEDGE?1.0:2.0) / (float) ROD_MM_PER_TURN)
#define DRIVE_STEPS_PER_REV ((GEAR_2_TEETH / (float) GEAR_1_TEETH) * DRIVE_STEPS_PER_TURN * DRIVE_MS * (DRIVE_DEDGE?1.0:2.0))

//STATE

enum {FEEDRATE_TIME,FEEDRATE_DIST} feed_mode = FEEDRATE_DIST;

float units = 1;//millimeters

//current axis position
float a_pos = 0;
float b_pos = 0;

Job from_speed_dist(float s, float d, const float steps_per_mm) {
  Job j = NOOP_JOB;

  if(d==d && d!=0) {

    s = s==s ? abs(s) : (DEFAULT_SPEED * units);

    j.frequency = (uint16_t) (s * steps_per_mm);
    j.dir = d<0 ? SET : UNSET;

    j.end.ty = COUNT;
    j.end.cond = (uint16_t) ((abs(d) / s) * j.frequency);

  }

  return j;
}



//G codes define movement and interpretation commands

//Rapid move (A axis , B axis position, Speed, Feedrate)
void g0 (float a, float b, float s, float f) {

  Jobs next = {{NOOP_JOB, NOOP_JOB, NOOP_JOB, NOOP_JOB}};

  next.jobs[0] = from_speed_dist(s*units, (a-a_pos)*units, FEED_STEPS_PER_MM);
  next.jobs[1] = from_speed_dist(s*units, (b-b_pos)*units, CLAMP_STEPS_PER_MM);

  switch(feed_mode) {
    case FEEDRATE_TIME:
      next.jobs[2] = from_speed_dist(f, f*abs((a-a_pos)/s), DRIVE_STEPS_PER_REV);
      break;
    case FEEDRATE_DIST:
      next.jobs[2] = from_speed_dist(s*f, abs(a-a_pos)*f, DRIVE_STEPS_PER_REV);
      break;
  }

  if(a==a) a_pos = a;
  if(b==b) b_pos = b;

  queue_jobs(next);

}

//Linear interpolate (A axis position, B axis position, Speed | Feedrate)
void g1 (float a, float b, float s, float f) {
  Jobs next = {{NOOP_JOB, NOOP_JOB, NOOP_JOB, NOOP_JOB}};

  float da = a-a_pos;
  float db = b-b_pos;
  float d = max(da,db);
  float t = d/s;

  next.jobs[0] = from_speed_dist(da*units/t, da*units, FEED_STEPS_PER_MM);
  next.jobs[1] = from_speed_dist(db*units/t, db*units, CLAMP_STEPS_PER_MM);

  switch(feed_mode) {
    case FEEDRATE_TIME:
      next.jobs[2] = from_speed_dist(f, f*abs(t), DRIVE_STEPS_PER_REV);
      break;
    case FEEDRATE_DIST:
      next.jobs[2] = from_speed_dist(s*f, abs(d)*f, DRIVE_STEPS_PER_REV);
      break;
  }

  if(a==a) a_pos = a;
  if(b==b) b_pos = b;

  queue_jobs(next);
}

//Dwell (P (millis) | S (seconds) )
void g4 (float p, float s) {}

//Programming in inches
void g20 () {
  a_pos /= units;
  b_pos /= units;
  units = 25.4;
  a_pos *= units;
  b_pos *= units;
}

//Programming in millimeters
void g21 () {
  a_pos /= units;
  b_pos /= units;
  units = 1.0;
  a_pos *= units;
  b_pos *= units;
}

//Home axis (A final position, B final position, Speed)
void g28 (float a, float b, float s) {

  // g0(a_pos+1,NAN,s,0);

  Jobs next = {{NOOP_JOB, NOOP_JOB, NOOP_JOB, NOOP_JOB}};

  if(a==a) {
    next.jobs[0] = from_speed_dist(s*units, 1, FEED_STEPS_PER_MM);
    next.jobs[0].end.ty = STALL_GUARD;
    next.jobs[0].end.cond = FEED_SGT;
  }

  if(b==b) {
    next.jobs[1] = from_speed_dist(s*units, 1, CLAMP_STEPS_PER_MM);
    next.jobs[1].end.ty = STALL_GUARD;
    next.jobs[1].end.cond = CLAMP_SGT;
  }

  queue_jobs(next);

  a_pos = b_pos = 0;

  g0(a,b,s,0);

}

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
