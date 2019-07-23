

#ifndef _MACHINE_H_
#define _MACHINE_H_


#define EN_CLAMP 38
#define DIR_CLAMP 55
#define STEP_CLAMP 54
#define CS_CLAMP 53
#define CLAMP_STEPS_PER_TURN 200
#define CLAMP_MS 16
#define CLAMP_CURRENT 800
#define CLAMP_SGT 20

#define EN_FEED 56
#define DIR_FEED 61
#define STEP_FEED 60
#define CS_FEED 49
#define FEED_STEPS_PER_TURN 200
#define FEED_MS 16
#define FEED_CURRENT 800
#define FEED_SGT 20

#define EN_DRIVE 62
#define DIR_DRIVE 48
#define STEP_DRIVE 46
#define CS_DRIVE 40
#define DRIVE_MS 1

#define DRIVE_STEPS_PER_TURN 200
#define ROD_MM_PER_TURN 2
#define AVR_CLK_FREQ 16000000
#define AVR_TIMER_MAX 0xFFFF

#define GEAR_1_TEETH 20
#define GEAR_2_TEETH 8

#include "timings.h"
#include "queue.h"

void step_clamp();
void step_feed();
void step_drive();

enum EndConditionType: u8 {
  NONE = 0,
  COUNT = 1,
  STALL_GUARD = 2
};

struct EndCondition {
  EndConditionType ty;
  u32 cond;
  volatile bool triggered;
};

struct Job {

  u16 frequency;
  SyncEvents<3> sync;
  bool dirs[3];
  EndCondition end[3];
  volatile u32 steps;

  Job();

};

typedef struct EndCondition EndCondition;
typedef struct Job Job;

bool queue_job(Job j);

void machine_loop();
void machine_init();



#endif
