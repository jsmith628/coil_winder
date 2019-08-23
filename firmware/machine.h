

#ifndef _MACHINE_H_
#define _MACHINE_H_


#define EN_CLAMP 56
#define DIR_CLAMP 61
#define STEP_CLAMP 60
#define STEP_CLAMP_PORT PORTF
#define STEP_CLAMP_BIT PORTF6
#define CS_CLAMP 49
#define SG_CLAMP 19
#define CLAMP_STEPS_PER_TURN 200
#define CLAMP_INVERT_DIR false
#define CLAMP_INVERT_EN true
#define CLAMP_DEDGE
#define CLAMP_MS 16
#define CLAMP_CURRENT 800
#define CLAMP_SGT 17
#define CLAMP_RANGE (143.5-10)


#define EN_FEED 38
#define DIR_FEED 55
#define STEP_FEED 54
#define STEP_FEED_PORT PORTF
#define STEP_FEED_BIT PORTF0
#define CS_FEED 53
#define SG_FEED 2
#define FEED_STEPS_PER_TURN 200
#define FEED_INVERT_DIR false
#define FEED_INVERT_EN true
// #define FEED_DEDGE
#define FEED_MS 16
#define FEED_CURRENT 800
#define FEED_SGT 15
#define FEED_RANGE (200-10)

#define EN_DRIVE 62
#define DIR_DRIVE 48
#define STEP_DRIVE 46
#define STEP_DRIVE_PORT PORTL
#define STEP_DRIVE_BIT PORTL3
#define DRIVE_INVERT_DIR false
#define DRIVE_INVERT_EN true
#define FEED_DEDGE
#define DRIVE_MS 1
#define DRIVE_STEPS_PER_TURN 200
#define DRIVE_MAX_BASE_SPEED 20
#define DRIVE_MAX_ACCEL 2
#define DRIVE_ACCEL_RESOLUTION 0.5

#define ROD_MM_PER_TURN 8
#define AVR_CLK_FREQ 16000000
#define AVR_TIMER_MAX 0xFFFF

#define GEAR_1_TEETH 20
#define GEAR_2_TEETH 8

#define FEED_STEPS_PER_MM (FEED_STEPS_PER_TURN * FEED_MS / (float) ROD_MM_PER_TURN)
#define CLAMP_STEPS_PER_MM (CLAMP_STEPS_PER_TURN * CLAMP_MS / (float) ROD_MM_PER_TURN)
#define DRIVE_STEPS_PER_REV (((float) GEAR_2_TEETH / (float) GEAR_1_TEETH) * DRIVE_STEPS_PER_TURN * DRIVE_MS)

#define FEED_STEP_RANGE (int32_t) (FEED_RANGE * FEED_STEPS_PER_MM)
#define CLAMP_STEP_RANGE (int32_t) (CLAMP_RANGE * CLAMP_STEPS_PER_MM)

#define SUBJOBS_PER_JOB 4
#define NOOP_JOB {0, KEEP, KEEP, 0, {IMMEDIATE, 0}, NULL}

#include "queue.h"

void step_clamp();
void step_feed();
void step_drive();

enum EndConditionType: uint8_t {
  IMMEDIATE = 0,
  COUNT = 1,
  STALL_GUARD = 2,
  FOREVER = 0xFF
};

typedef struct {
  EndConditionType ty;
  uint32_t cond;
} EndCondition;

enum PinOption: uint8_t { KEEP=0, SET, UNSET };

typedef struct {
  byte axis;
  PinOption dir;
  PinOption en;
  u16 frequency;
  EndCondition end;
  void (*callback)(const void*);
  const void * callback_args;
} Job;

typedef struct {
  Job jobs[SUBJOBS_PER_JOB];
} Jobs;

bool queue_jobs(Jobs j);
void clear_job_queue();
void clear_jobs();
byte open_jobs();

bool busy();

void machine_loop();
void machine_init();



#endif
