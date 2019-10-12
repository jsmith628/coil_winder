

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
#define FEED_DEDGE
#define FEED_MS 16
#define FEED_CURRENT 800
#define FEED_SGT 15
#define FEED_RANGE (200-10)

#define EN_DRIVE 24
#define DIR_DRIVE 28
#define STEP_DRIVE 26
#define STEP_DRIVE_PORT PORTA
#define STEP_DRIVE_BIT PORTL4
#define DRIVE_INVERT_DIR false
#define DRIVE_INVERT_EN true
// #define DRIVE_DEDGE
#define DRIVE_MS 1
#define DRIVE_STEPS_PER_TURN 200

#define ROD_MM_PER_TURN 8

#define AVR_CLK_FREQ 16000000
#define ACCEL_TIME_RESOLUTION 0.01
#define DWELL_FREQUENCY 10000

#define DEFAULT_FEEDRATE 1
#define DEFAULT_SPINDLE_SPEED 1
#define DEFAULT_MAX_FEEDRATE INFINITY
#define DEFAULT_MAX_SPINDLE_SPEED INFINITY
#define DEFAULT_BASE_SPEED INFINITY
#define DEFAULT_MAX_ACCELERATION 1

#define PERIOD_LUT_FREQ_START 0 //must be an integer
#define PERIOD_LUT_FREQ_STEP 0 //must be an power of two no bigger than 16
#define PERIOD_LUT_SIZE 8192 //must be a power of two no bigger than 8192

#define GEAR_1_TEETH 20
#define GEAR_2_TEETH 8

#define FEED_STEPS_PER_MM (FEED_STEPS_PER_TURN * FEED_MS / (float) ROD_MM_PER_TURN)
#define CLAMP_STEPS_PER_MM (CLAMP_STEPS_PER_TURN * CLAMP_MS / (float) ROD_MM_PER_TURN)
#define DRIVE_STEPS_PER_REV (((float) GEAR_2_TEETH / (float) GEAR_1_TEETH) * DRIVE_STEPS_PER_TURN * DRIVE_MS)

#define FEED_STEP_RANGE (int32_t) (FEED_RANGE * FEED_STEPS_PER_MM)
#define CLAMP_STEP_RANGE (int32_t) (CLAMP_RANGE * CLAMP_STEPS_PER_MM)

#define JOB_QUEUE_ORDER 5
#define JOB_QUEUE_SIZE (1<<JOB_QUEUE_ORDER)

#define SUBJOBS_PER_JOB 4
#define NOOP_JOB {0, 0, {IMMEDIATE, 0}, NULL}
#define NOOP_JOBS {false, {NOOP_JOB, NOOP_JOB, NOOP_JOB, NOOP_JOB}}

#include "queue.h"

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
  int16_t frequency;
  EndCondition end;
  void (*callback)(const void*);
  const void * callback_args;
} Job;

typedef struct {
  bool fence;
  Job jobs[SUBJOBS_PER_JOB];
} Jobs;

void enable_stepper(uint8_t axis);
void disable_stepper(uint8_t axis);

void pause_jobs();
void resume_jobs();

bool queue_jobs(Jobs j);
void clear_job_queue();
void clear_jobs();
bool job_queue_open();

void set_max_acceleration(uint16_t accel);
void set_start_acceleration(uint16_t speed);

bool busy();

void machine_loop();
void machine_init();

int32_t steps_moved(uint8_t axis);

#endif
