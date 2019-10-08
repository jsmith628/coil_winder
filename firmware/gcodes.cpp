
#include <Arduino.h>
#include <EEPROM.h>

#include "gcodes.h"
#include "control.h"
#include "machine.h"
#include "ascii_control.h"

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

  float pos; //current units
  int32_t zero; //steps

  bool homing;

  float steps_per_unit;
  const float steps_per_machine_unit;

  CoordinateSystem coords;
} Axis;

Axis axes[NUM_AXES] = {
  {'A', 0,-FEED_STEP_RANGE,0, 0.0,0,false, FEED_STEPS_PER_MM,FEED_STEPS_PER_MM, MACHINE},
  {'B', 0,-CLAMP_STEP_RANGE,0, 0.0,0,false, CLAMP_STEPS_PER_MM,CLAMP_STEPS_PER_MM, MACHINE},
  {'W', 0,-0x7FFFFFFF,(int32_t) 0x7FFFFFFF, 0.0,0,false, DRIVE_STEPS_PER_REV,DRIVE_STEPS_PER_REV, INCREMENTAL},
};

struct Settings {
  bool endstops_enabled = true;
  float max_feedrate = DEFAULT_MAX_FEEDRATE;
  float max_spindle_speed = DEFAULT_MAX_SPINDLE_SPEED;
  float start_acceleration = DEFAULT_BASE_SPEED;
  float max_acceleration = DEFAULT_MAX_ACCELERATION;
} settings;

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

  if(x==x && s==s && s!=0) {

    float d = position_change(axis, x);

    if(d==d && d!=0) {
      j.frequency = (int16_t) (s * axes[axis].steps_per_unit);
      j.frequency *= d<0 ? -1 : 1;

      j.end.ty = COUNT;

      int32_t new_pos = axes[axis].machine_pos + (int32_t) (d*axes[axis].steps_per_unit);
      if(settings.endstops_enabled)
        new_pos = min(max(new_pos,axes[axis].min_pos), axes[axis].max_pos);

      // Serial.print(axes[axis].min_pos);
      // Serial.print(" ");
      // Serial.print(new_pos);
      // Serial.print(" ");
      // Serial.println(axes[axis].max_pos);

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
  if(zero==zero) {
    axes[axis].zero = zero * axes[axis].steps_per_unit;
    axes[axis].coords = LOCAL;
    axes[axis].pos = pos_from_steps(axis, axes[axis].machine_pos);
  }
}

void set_incremental_coords(byte axis) {
  if(axes[axis].coords == INCREMENTAL) return;
  set_machine_coords(axis);
  axes[axis].coords = INCREMENTAL;
}

void update_hw_pos(const void* args) {

  byte i = (byte) (int) args;

  axes[i].machine_pos += steps_moved(i);
  axes[i].pos = pos_from_steps(i, axes[i].machine_pos);
  axes[i].homing = false;

}

float last_feedrate = DEFAULT_FEEDRATE;

inline float next_feedrate(float f) {
  if(f==f) {
    f = abs(f);
    last_feedrate = f;
    return min(f,settings.max_feedrate);
  } else {
    return min(last_feedrate,settings.max_feedrate);
  }
}

inline float drive_speed(float da, float w, float s, float travel) {
  if(s!=s && da==da && travel==travel) {
    float dw = position_change(W_AXIS, w);
    return abs(dw / (da / travel));
  } else if(s==s) {
    return min(abs(s),settings.max_spindle_speed);
  } else {
    return DEFAULT_SPINDLE_SPEED;
  }
}

void queue_callback(void (*callback)(const void*), const void * callback_args) {
  Job j = NOOP_JOB;
  j.callback = callback;
  j.callback_args = callback_args;

  queue_jobs({{NOOP_JOB, NOOP_JOB, NOOP_JOB, j}});
}

bool interrupt_queued = false;

void print_eot(const void* arg) { Serial.print(EOT); }
void print_interrupt(const void* arg) { interrupt_queued=false; Serial.print(INTERRUPT); }

void println_callback(const void* msg) {
  Serial.println((char*) msg);
}

void print_callback(const void* msg) {
  Serial.print((char*) msg);
}

//control functions

//returns the min number of commands available in the queue
bool queue_open() {
  for(byte i=0; i<NUM_AXES; i++) if(axes[i].homing) return false;
  return job_queue_open();
}

//cancels the last command put into the queue (assuming it hasn't started yet)
void cancel() {
  Serial.println("Canceling last command");
  // cancel_last_job();
}

//clears queue and sends INTERRUPT code once current command is complete
void interrupt() {
  if(!interrupt_queued) {
    interrupt_queued = true;
    clear_job_queue();
    queue_callback(print_interrupt, NULL);
  } else {
    m112();
    Serial.print(INTERRUPT);
  }
}

//queues an M30
void end_of_transmission() {m30();}

//runs an emergency stop and echos the QUIT code
void quit() { m112(); Serial.print(QUIT); }

//sends an ACK or NAK depending on whether the queue can accept stuffs
void enquiry() {Serial.print(queue_open() ? ACK : NAK);}

//run when receiving an ACK
void acknowledge() {}

//makes a noise
void bell() {}

//pauses machine excecution
void pause() {
  pause_jobs();
  Serial.println("Pausing!");
  Serial.print(PAUSE);
}

//resumes machine excecution from pause
void resume() {
  Serial.println("Resuming!");
  resume_jobs();
}

//run when receiving an NAK
void neg_acknowledge() {}

//G codes define movement and interpretation commands

//Rapid move (A axis position, B axis position, Spindle rotations, Speed, Feedrate)
void g0 (float a, float b, float w, float s, float f) {

  float travel = next_feedrate(f);
  float drive = drive_speed(position_change(A_AXIS,a),w,s,travel);

  //clamp down the speeds if the drive exceeds the max and is determined by feedrate
  if(s!=s && drive==drive && drive>settings.max_spindle_speed) {
      float factor = settings.max_spindle_speed / drive;
      drive = settings.max_spindle_speed;
      travel *= factor;
  }

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
  float dw = position_change(W_AXIS,w);

  float travel = next_feedrate(f);

  float max_move = da==da ? db==db ? max(abs(da),abs(db)) : da : db;
  float time = max_move / travel;
  if(s==s && dw==dw) time = max(dw, time);

  float a_speed = abs(da / time);
  float b_speed = abs(db / time);
  float w_speed = abs(dw / time);
  float drive = drive_speed(da,w,w_speed,a_speed);

  //clamp down the speeds if the drive exceeds the max and is determined by feedrate
  if(s!=s && drive==drive && drive>settings.max_spindle_speed) {
      float factor = settings.max_spindle_speed / drive;
      drive = settings.max_spindle_speed;
      a_speed *= factor;
      b_speed *= factor;
  }

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
    next.jobs[3].frequency = DWELL_FREQUENCY;
    next.jobs[3].end.ty = COUNT;
    next.jobs[3].end.cond = (uint16_t) (p/1000.0 * DWELL_FREQUENCY);
  } else if(s==s) {
    next.jobs[3].frequency = DWELL_FREQUENCY;
    next.jobs[3].end.ty = COUNT;
    next.jobs[3].end.cond = (uint16_t) (s * DWELL_FREQUENCY);
  }

  queue_jobs(next);
}

//Programming in inches
void g20 () {
  set_units(25.4);
  Serial.println("Using Inches");
}

//Programming in millimeters
void g21 () {
  set_units(1.0);
  Serial.println("Using Millimeters");
}

void feed_until_skip(bool do_zero, int8_t axis_dirs[]) {
  Jobs next = {{NOOP_JOB, NOOP_JOB, NOOP_JOB, NOOP_JOB}};

  for(byte i=0; i<NUM_AXES; i++) {
    if(axis_dirs[i]!=0) {
      next.jobs[i].frequency = (int16_t) (6.0*axes[i].steps_per_machine_unit);
      next.jobs[i].frequency *= axis_dirs[i]<0 ? -1 : 1;
      next.jobs[i].end.ty = STALL_GUARD;
      next.jobs[i].end.cond = FEED_SGT;

      if(do_zero) {
        for(byte i=0; i<NUM_AXES; i++) {
          if(axis_dirs)
          axes[i].machine_pos = 0;
          axes[i].pos = pos_from_steps(i,0);
        }
      } else {
        next.jobs[i].callback = update_hw_pos;
        next.jobs[i].callback_args = (void *) (int) i;
        axes[i].homing = true;
      }

    }
  }

  queue_jobs(next);



}

//Home axis (A final position, B final position)
void g28 (bool a, bool b) {
  if(!a && !b){
    a = true;
    b = true;
  }
  int8_t dirs[NUM_AXES] = {(int8_t) (a?1:0), (int8_t) (b?1:0), (int8_t) 0};

  feed_until_skip(true, dirs);

}

//Feed until skip (A axis enable, B axis enable)
void g31 (int8_t a, int8_t b) {
  int8_t dirs[NUM_AXES] = {a,b,0};
  feed_until_skip(false, dirs);
}

//Define maximum spindle Speed (Speed)
void g50 (float s) {
    settings.max_spindle_speed = s==s ? s : DEFAULT_MAX_SPINDLE_SPEED;
    Serial.print("Set maximum spindle speed to ");
    Serial.println(settings.max_spindle_speed);
}

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
    Serial.print('=');
    Serial.print(axes[i].pos);
    if(i<NUM_AXES) Serial.print(", ");
  }

  Serial.println();
}

//Absolute positioning (position defined from machine zero)
void g90() {
  Serial.print("Machine Coords: ");

  for(byte i=0; i<W_AXIS; i++) {
    set_machine_coords(i);
    Serial.print(axes[i].name);
    Serial.print('=');
    Serial.print(axes[i].pos);
    if(i<W_AXIS) Serial.print(", ");
  }

  Serial.println();
}

//Incremental positioning (position defined relative to previous position)
void g91() {
  Serial.println("Incremental Coords");
  for(byte i=0; i<W_AXIS; i++) set_incremental_coords(i);
}

//Set current position to specified value (A position, B position, Spindle position)
void g92(float a, float b, float w){
  Serial.print("Position set to: ");

  for(char i=0; i<3; i++) {
      float param = i==0 ? a : i==1 ? b : w;
      if(param==param) {
        set_machine_coords(i);
        set_local_coords(i, axes[i].pos-param);
        Serial.print(axes[i].name);
        Serial.print('=');
        Serial.print(param);
        if(i<3) Serial.print(", ");
      }
  }

  Serial.println();
}

//Feedrate per minute
void g94() {feed_mode = FEEDRATE_TIME;}

//Feedrate per revolution
void g95() {feed_mode = FEEDRATE_DIST;}

//M codes define miscellaneous commands

//Unconditional stop
void m0() {}

void enable_stepper(const void * axis) { enable_stepper((uint8_t) (uint16_t) axis); }
void disable_stepper(const void * axis) { disable_stepper((uint8_t) (uint16_t) axis); }

void print_enable_status(const void * steppers) {
  uint16_t bits = (uint16_t) steppers;

  bool all_enabled = true;
  for(uint8_t i=0; i<NUM_AXES; i++) all_enabled = all_enabled && ((bits & (1<<i)) != 0);

  if(all_enabled) {
    Serial.println("Steppers enabled");
  } else {
    bool first = true;
    for(uint8_t i=0; i<NUM_AXES; i++) {
      if((bits & (1<<i)) != 0) {
        if(!first) Serial.print(" and ");
        Serial.print(axes[i].name);
        first = false;
      }
    }
    Serial.println(" enabled");
  }

}

//Enable steppers
void m17() { m17(true, true, true); }

//Enable Steppers
void m17(bool a, bool b, bool w){
  Jobs next;
  bool enable[3] = {a,b,w};

  uint16_t args = 0;

  for(uint8_t i=0; i<3; i++) {
    next.jobs[i] = NOOP_JOB;
    if(enable[i]) {
      next.jobs[i].callback = enable_stepper;
      next.jobs[i].callback_args = (void*) (uint16_t) i;
      args |= (1<<i);
    }
  }

  next.jobs[3] = NOOP_JOB;
  next.jobs[3].callback = print_enable_status;
  next.jobs[3].callback_args = (void*) args;


  queue_jobs(next);
}

//Disable steppers
void m18() {
  Jobs next;
  for(byte i=0; i<NUM_AXES; i++) {
    next.jobs[i] = NOOP_JOB;
    next.jobs[i].callback = disable_stepper;
    next.jobs[i].callback_args = (void*) (uint16_t) i;
  }
  next.jobs[NUM_AXES] = NOOP_JOB;
  next.jobs[NUM_AXES].callback = println_callback;
  next.jobs[NUM_AXES].callback_args = "Steppers disabled";
  queue_jobs(next);
}

//End of program, return to program top
void m30() {queue_callback(print_eot, NULL);}

//Spindle absolute positioning
void m82(){set_machine_coords(W_AXIS);}

//Spindle incremental positioning
void m83(){set_incremental_coords(W_AXIS);}

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
  for(uint8_t i=0; i<NUM_AXES; i++) disable_stepper(i);
  clear_jobs();
  Serial.println("Emergency Stop!");
}

//Current position
void m114() {
  for(byte i=0; i<NUM_AXES; i++) {
    Serial.print(axes[i].name);
    Serial.print("=");
    Serial.print(axes[i].pos);
    Serial.print(" ");
  }
  Serial.println();
}

void m114(bool a, bool b, bool w) {
  if(a) {
    Serial.print("A=");
    Serial.print(axes[0].pos);
    Serial.print(" ");
  }
  if(b) {
    Serial.print("B=");
    Serial.print(axes[1].pos);
    Serial.print(" ");
  }
  if(w) {
    Serial.print("W=");
    Serial.print(axes[2].pos);
    Serial.print(" ");
  }
  Serial.println();
}

//Enable software endstops
void m120() {
  settings.endstops_enabled = true;
  Serial.println("Endstops Enabled");
}

//Disable software endstops
void m121() {
  settings.endstops_enabled = false;
  Serial.println("Endstops Disabled");
}

//Park (A axis position, B axis postion)
void m125(float a, float b, float w) {}

void set_max_acceleration(const void*accel) { set_max_acceleration((uint16_t) accel); }
void set_start_acceleration(const void*start) { set_start_acceleration((uint16_t) start); }

//Set max acceleration
void m201(float w) {
  w = w==w ? abs(w) : DEFAULT_MAX_ACCELERATION;
  settings.max_acceleration = w;

  Jobs next = {{NOOP_JOB, NOOP_JOB, NOOP_JOB, NOOP_JOB}};
  next.jobs[W_AXIS].callback = set_max_acceleration;
  next.jobs[W_AXIS].callback_args = (void*) (uint16_t) (w * DRIVE_STEPS_PER_REV);
  queue_jobs(next);

  Serial.print("Set maximum acceleration to ");
  Serial.println(w);
}

//Set max feedrate (Feedrate)
void m203(float f) {
    settings.max_feedrate = f==f ? f : DEFAULT_MAX_FEEDRATE;
    Serial.print("Set maximum feedrate to ");
    Serial.println(settings.max_feedrate);
}

//Set starting acceleration (A axis acceleration, B axis acceleration, Spindle acceleration)
void m204(float w) {
  w = w==w ? abs(w) : DEFAULT_BASE_SPEED;
  settings.start_acceleration = w;

  Jobs next = {{NOOP_JOB, NOOP_JOB, NOOP_JOB, NOOP_JOB}};
  next.jobs[W_AXIS].callback = set_start_acceleration;
  next.jobs[W_AXIS].callback_args = (void*) (uint16_t) (w * DRIVE_STEPS_PER_REV);
  queue_jobs(next);

  Serial.print("Set start acceleration to ");
  Serial.println(w);
}

inline void enact_settings() {
  if(settings.endstops_enabled) {m120();} else {m121();}
  m203(settings.max_feedrate);
  g50(settings.max_spindle_speed);
  m201(settings.max_acceleration);
  m204(settings.start_acceleration);
}

//Save settings to EEPROM
void m500() {
  EEPROM.put(0, settings);
  Serial.println("Saved settings to EEPROM");
}

//Load settings from EEPROM
void m501() {
  EEPROM.get(0, settings);
  Serial.println("Loaded settings from EEPROM:");
  enact_settings();
}

//Factory reset settings
void m502() { settings = Settings(); enact_settings();}

//Read out settings
void m503() {
  Serial.println("Current Settings:");
  Serial.println(settings.endstops_enabled ? "Endstops enabled" : "Endstops disabled");
  Serial.print("Maximum feedrate: ");
  Serial.println(settings.max_feedrate);
  Serial.print("Maximum spindle speed: ");
  Serial.println(settings.max_spindle_speed);
  Serial.print("Maximum acceleration: ");
  Serial.println(settings.max_acceleration);
  Serial.print("Start Acceleration: ");
  Serial.println(settings.start_acceleration);
}
