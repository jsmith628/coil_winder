
#include <Arduino.h>
#include "gcodes.h"
#include "machine.h"

#define DEFAULT_SPEED 1

#define DWELL_MS_PRECISION 1

#define A_AXIS 0
#define B_AXIS 1
#define W_AXIS 2
#define NUM_AXES 3

//STATE

enum {FEEDRATE_TIME,FEEDRATE_DIST} feed_mode = FEEDRATE_DIST;

enum CoordinateSystem: uint8_t {
  MACHINE=0, LOCAL=1, INCREMENTAL=0xFF
};

typedef struct {

  const char name;

  int32_t machine_pos; //steps
  const int32_t min_pos, max_pos; //steps

  float pos; //currect units
  int32_t zero; //steps

  float steps_per_unit;
  const float steps_per_machine_unit;

  CoordinateSystem coords;
} Axis;

Axis axes[NUM_AXES] = {
  {'A', 0,0,0, 0.0,0, FEED_STEPS_PER_MM,FEED_STEPS_PER_MM, MACHINE},
  {'B', 0,0,0, 0.0,0, CLAMP_STEPS_PER_MM,CLAMP_STEPS_PER_MM, MACHINE},
  {'W', 0,0,0, 0.0,0, DRIVE_STEPS_PER_REV,DRIVE_STEPS_PER_REV, INCREMENTAL},
};

bool endstops_enabled = true;

inline float pos_from_steps(byte axis, int32_t new_pos) {
  if(axes[axis].coords==LOCAL) {
    return (float) (new_pos-axes[axis].zero) / axes[axis].steps_per_unit;
  } else {
    return (float) new_pos / axes[axis].steps_per_unit;
  }
}

inline float position_change(byte axis, float a) {
  return axes[axis].coords==INCREMENTAL||a!=a ? a : (a-axes[axis].pos);
}

Job move_to(byte axis, float x, float s) {
  Job j = NOOP_JOB;

  if(x==x && s==s) {

    float d = position_change(axis, x);

    if(d==d && d!=0) {
      j.frequency = (uint16_t) (s * axes[axis].steps_per_unit);
      j.dir = d<0 ? SET : UNSET;

      j.end.ty = COUNT;

      int32_t new_pos = axes[axis].machine_pos + (int32_t) (d*axes[axis].steps_per_unit);
      if(endstops_enabled) new_pos = min(max(new_pos,axes[axis].min_pos), axes[axis].max_pos);
      j.end.cond = (uint16_t) abs(new_pos - axes[axis].machine_pos);

      axes[axis].machine_pos = new_pos;
      axes[axis].pos = pos_from_steps(axis, new_pos);
    }

  }
  return j;
}

void set_units(byte axis, float steps_per_unit) {
  if(steps_per_unit==steps_per_unit) {
    axes[axis].steps_per_unit = steps_per_unit;
    axes[axis].pos = pos_from_steps(axis, axes[axis].machine_pos);
  }
}

void set_units(float units_per_mm) {
  set_units(A_AXIS, units_per_mm * axes[A_AXIS].steps_per_machine_unit);
  set_units(B_AXIS, units_per_mm * axes[B_AXIS].steps_per_machine_unit);
}

void set_machine_coords(byte axis) {
  if(axes[axis].coords == MACHINE) return;
  axes[axis].coords = MACHINE;
  axes[axis].pos = pos_from_steps(axis, axes[axis].machine_pos);
}

void set_local_coords(byte axis, float zero) {
  if(axes[axis].coords == LOCAL) return;
  if(zero==zero) axes[axis].zero = zero * axes[axis].steps_per_unit;
  axes[axis].coords = LOCAL;
  axes[axis].pos = pos_from_steps(axis, axes[axis].machine_pos);
}

void set_incremental_coords(byte axis) {
  if(axes[axis].coords == INCREMENTAL) return;
  set_machine_coords(axis);
  axes[axis].coords = INCREMENTAL;
}

float last_feedrate = 0;

inline float next_feedrate(float f) {
  if(f==f) {
    last_feedrate = f;
    return f;
  } else {
    return last_feedrate;
  }
}

inline float drive_speed(float da, float w, float s, float travel) {
  if(s!=s && da==da) {
    float dw = position_change(W_AXIS, w);
    return dw / (da / travel);
  } else {
    return s;
  }
}

//G codes define movement and interpretation commands

//Rapid move (A axis position, B axis position, Spindle rotations, Speed, Feedrate)
void g0 (float a, float b, float w, float s, float f) {

  float travel = next_feedrate(f);
  float drive = drive_speed(position_change(A_AXIS,a),w,s,travel);

  Jobs next;

  next.jobs[A_AXIS] = move_to(A_AXIS, a, travel);
  next.jobs[B_AXIS] = move_to(B_AXIS, b, travel);
  next.jobs[W_AXIS] = move_to(W_AXIS, w, drive);
  next.jobs[3] = NOOP_JOB;

  queue_jobs(next);

}

//Linear interpolate (A axis position, B axis position, Spindle rotations, Speed, Feedrate)
void g1 (float a, float b, float w, float s, float f) {

  float da = position_change(A_AXIS,a);
  float db = position_change(B_AXIS,b);

  float travel = next_feedrate(f);

  float max_move = da==da ? db==db ? max(da,db) : da : db;
  float time = max_move / travel;

  float a_speed = da / time;
  float b_speed = db / time;
  float drive = drive_speed(da,w,s,a_speed);

  Jobs next;

  next.jobs[A_AXIS] = move_to(A_AXIS, a, a_speed);
  next.jobs[B_AXIS] = move_to(B_AXIS, b, b_speed);
  next.jobs[W_AXIS] = move_to(W_AXIS, w, drive);
  next.jobs[3] = NOOP_JOB;

  queue_jobs(next);

}

//Dwell (P (millis) | S (seconds) )
void g4 (float p, float s) {
  Jobs next = {{NOOP_JOB, NOOP_JOB, NOOP_JOB, NOOP_JOB}};

  if(p==p) {
    if(s==s) p += s*1000.0;
    next.jobs[3].frequency = AVR_CLK_FREQ / (DWELL_MS_PRECISION*1000);//microsecond precision
    next.jobs[3].end.ty = COUNT;
    next.jobs[3].end.cond = (uint16_t) (p * DWELL_MS_PRECISION);
  } else if(s==s) {
    next.jobs[3].frequency = AVR_CLK_FREQ / (DWELL_MS_PRECISION*1000);//microsecond precision
    next.jobs[3].end.ty = COUNT;
    next.jobs[3].end.cond = (uint16_t) (s * DWELL_MS_PRECISION*1000);
  }

  queue_jobs(next);
}

//Programming in inches
void g20 () { set_units(25.4); }

//Programming in millimeters
void g21 () { set_units(1.0); }

//Home axis (A final position, B final position)
void g28 (bool a, bool b) {

  // Jobs next = {{NOOP_JOB, NOOP_JOB, NOOP_JOB, NOOP_JOB}};
  //
  // if(a) {
  //   next.jobs[0] = from_speed_dist(6*units, 1, FEED_STEPS_PER_MM);
  //   next.jobs[0].end.ty = STALL_GUARD;
  //   next.jobs[0].end.cond = FEED_SGT;
  // }
  //
  // if(b) {
  //   next.jobs[1] = from_speed_dist(6*units, 1, CLAMP_STEPS_PER_MM);
  //   next.jobs[1].end.ty = STALL_GUARD;
  //   next.jobs[1].end.cond = CLAMP_SGT;
  // }
  //
  // queue_jobs(next);
  //
  // a_pos = b_pos = 0;

}

//Feed until skip (A axis enable, B axis enable)
void g31 (int8_t a, int8_t b) {}

//Define maximum spindle Speed (Speed)
void g50 (float s) {}

//Local coordinates, defines program zero to a new location (A position 0, B position 0, W position 0)
void g52 (float a, float b, float w) {
  Serial.print("Local Coords: ");

  float zero[NUM_AXES] = {a,b,w};
  for(byte i=0; i<NUM_AXES; i++) {
    set_local_coords(i,zero[i]);
    Serial.print(axes[i].name);
    Serial.print("0=");
    Serial.print(axes[i].zero/axes[i].steps_per_unit);
    Serial.print(' ');
    Serial.print(axes[i].name);
    Serial.print("=");
    Serial.print(axes[i].pos);
  }

  Serial.println();
}

//Absolute positioning (position defined from machine zero)
void g90() {
  Serial.print("Machine Coords: ");

  for(byte i=0; i<NUM_AXES; i++) {
    set_machine_coords(i);
    Serial.print(axes[i].name);
    Serial.print("=");
    Serial.print(axes[i].pos);
  }

  Serial.println();
}

//Incremental positioning (position defined relative to previous position)
void g91() {
  Serial.println("Incremental Coords");
  for(byte i=0; i<NUM_AXES; i++) set_incremental_coords(i);
}

//Set current position to specified value (A position, B position, Spindle position)
void g92(float a, float b, float w){}

//Feedrate per minute
void g94() {feed_mode = FEEDRATE_TIME;}

//Feedrate per revolution
void g95() {feed_mode = FEEDRATE_DIST;}

//M codes define miscellaneous commands

//Unconditional stop
void m0() {}

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

//Enable Steppers
void m17(bool a, bool b, bool c){
  Jobs next;
  if(a){
    next.jobs[A_AXIS] = NOOP_JOB;
    next.jobs[A_AXIS].en = SET;
  }
  if(b){
    next.jobs[B_AXIS] = NOOP_JOB;
    next.jobs[B_AXIS].en = SET;
  }
  if(c){
    next.jobs[W_AXIS] = NOOP_JOB;
    next.jobs[W_AXIS].en = SET;
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
  Serial.println("Steppers Disabled");
  queue_jobs(next);
}

//End of program, return to program top
void m30() {}

//Spindle absolute positioning
void m82(){

}

//Spindle incremental positioning
void m83(){}

//Set axis steps per unit
void m92(float a, float b, float w){
  set_units(A_AXIS, a);
  set_units(B_AXIS, b);
  set_units(W_AXIS, w);
}

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

//Enable software endstops
void m120() {
  endstops_enabled = true;
  Serial.println("Endstops Enabled");
}

//Disable software endstops
void m121() {
  endstops_enabled = false;
  Serial.println("Endstops Disabled");
}

//Park (A axis position, B axis postion)
void m125(float a, float b, float w) {}

//Set max acceleration
void m201(float s) {}

//Set max feedrate (Feedrate)
void m203(float f) {}

//Set starting acceleration (A axis acceleration, B axis acceleration, Spindle acceleration)
void m204(float a, float b, float w) {}

//Save settings to EEPROM
void m500() {}

//Load settings from EEPROM
void m501() {}

//Read out settings from EEPROM
void m503() {}
