
#include <TMC2130Stepper.h>
#include "machine.h"
#include "timings.h"

typedef TMC2130Stepper Stepper;

typedef char i8;
typedef int i16;
typedef long i32;

typedef byte u8;
// typedef unsigned int u16;
typedef unsigned long u32;

Stepper clamp = Stepper(EN_CLAMP, DIR_CLAMP, STEP_CLAMP, CS_CLAMP);
Stepper feed = Stepper(EN_FEED, DIR_FEED, STEP_FEED, CS_FEED);


inline void step_clamp() {
  static bool step = false;
  step = !step;

  digitalWrite(STEP_CLAMP,step?HIGH:LOW);
}

inline void step_feed() {
  static bool step = false;
  step = !step;

  digitalWrite(STEP_FEED,step?HIGH:LOW);
}

inline void step_drive() {
  static bool step = false;
  step = !step;
  digitalWrite(STEP_DRIVE,step?HIGH:LOW);
}

enum EndConditionType: u8 {
  NONE = 0,
  COUNT = 1,
  STALL_GUARD = 2
};

struct EndCondition {
  EndConditionType ty;
  u32 cond;
  bool triggered;
};

struct Job {

  SyncEvents<3> sync;
  bool dirs[3];
  EndCondition end[3];
  u32 steps;

  Job() {}

};


struct Job current_job;

ISR(TIMER1_COMPA_vect) {
  static bool do_step[3] = {false,false,false};

  current_job.steps++;
  current_job.sync.step(&do_step[0]);

  if(!current_job.end[0].triggered && do_step[0]) {
    step_clamp();
    if(current_job.end[0].ty == COUNT && current_job.steps>current_job.end[0].cond)
      current_job.end[0].triggered = true;
  }

  if(!current_job.end[1].triggered && do_step[1]) {
    step_feed();
    if(current_job.end[1].ty == COUNT && current_job.steps>current_job.end[1].cond)
      current_job.end[1].triggered = true;
  }

  if(!current_job.end[2].triggered && do_step[2]){
    step_drive();
    if(current_job.end[2].ty == COUNT && current_job.steps>current_job.end[2].cond)
      current_job.end[2].triggered = true;
  }

}

void machine_init() {

  SPI.begin();

  clamp.begin();
  clamp.rms_current(CLAMP_CURRENT);
  clamp.dedge(1);
  clamp.microsteps(CLAMP_MS);
  clamp.TCOOLTHRS(0xFFFFF);
  clamp.sgt(CLAMP_SGT);

  feed.begin();
  feed.rms_current(FEED_CURRENT);
  feed.dedge(1);
  feed.microsteps(FEED_MS);
  feed.TCOOLTHRS(0xFFFFF);
  feed.sgt(FEED_SGT);

  pinMode(EN_DRIVE, OUTPUT);
  pinMode(STEP_DRIVE, OUTPUT);
  pinMode(DIR_DRIVE, OUTPUT);

  digitalWrite(EN_CLAMP, HIGH);
  digitalWrite(EN_FEED, HIGH);
  digitalWrite(EN_DRIVE, HIGH);

  cli();

  TCCR1A = 0;
  TCCR1B = 1 | (1<<WGM12);
  OCR1A = 0x0FFF;
  TIMSK1 = 2;

  sei();


 // digitalWrite(EN_CLAMP, LOW);
 // digitalWrite(EN_FEED, LOW);
 // digitalWrite(EN_DRIVE, LOW);
}
