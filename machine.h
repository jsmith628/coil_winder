

#ifndef _MACHINE_H_
#define _MACHINE_H_


#define EN_CLAMP 38
#define DIR_CLAMP 55
#define STEP_CLAMP 54
#define CS_CLAMP 53
#define CLAMP_STEPS_PER_TURN 200
#define CLAMP_INVERT_DIR false
#define CLAMP_INVERT_EN true
#define CLAMP_DEDGE true
#define CLAMP_MS 16
#define CLAMP_CURRENT 800
#define CLAMP_SGT 20


#define EN_FEED 56
#define DIR_FEED 61
#define STEP_FEED 60
#define CS_FEED 49
#define FEED_STEPS_PER_TURN 200
#define FEED_INVERT_DIR false
#define FEED_INVERT_EN true
#define FEED_DEDGE true
#define FEED_MS 16
#define FEED_CURRENT 800
#define FEED_SGT 20

#define EN_DRIVE 62
#define DIR_DRIVE 48
#define STEP_DRIVE 46
#define DRIVE_INVERT_DIR false
#define DRIVE_INVERT_EN true
#define DRIVE_DEDGE false
#define DRIVE_MS 1

#define DRIVE_STEPS_PER_TURN 200
#define ROD_MM_PER_TURN 2
#define TIMINGS_PRECISION 8
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
  IMMEDIATE = 0,
  COUNT = 1,
  STALL_GUARD = 2,
  FOREVER = 0xFF
};

struct EndCondition {
  EndConditionType ty;
  u16 cond;
  bool triggered;
};

struct Job {

  u16 frequency;
  int ratio[3];
  bool dirs[3];
  bool enabled[3];
  EndCondition end[3];

  Job();

};

typedef struct EndCondition EndCondition;
typedef struct Job Job;

bool queue_job(Job j);
void clear_jobs();

void machine_loop();
void machine_init();



#endif
