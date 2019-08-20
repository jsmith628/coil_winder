
#include <TMC2130Stepper.h>
#include "machine.h"

typedef TMC2130Stepper Stepper;

typedef char i8;
typedef int i16;
typedef long i32;

typedef byte u8;
// typedef unsigned int u16;
typedef unsigned long u32;

Stepper clamp = Stepper(EN_CLAMP, DIR_CLAMP, STEP_CLAMP, CS_CLAMP);
Stepper feed = Stepper(EN_FEED, DIR_FEED, STEP_FEED, CS_FEED);

const byte prescaling_16_bit[6] = {1,3,3,2,2,0};
const byte prescaling_timer_2[8] = {1,3,2,1,1,1,2,0};

//A handy container object for all of the timer registers
typedef struct {
    const byte bits;
    const uint32_t mask;
    const byte * pre_mul;
    volatile uint8_t * tccrna;
    volatile uint8_t * tccrnb;
    volatile void * tcnt;
    volatile void * ocra;
    volatile uint8_t * timsk;
} Timer;

//all of the 16bit timers on the ATMEGA2560
Timer timers[4] = {
  {16, 0xFFFF0000, prescaling_16_bit, &TCCR3A, &TCCR3B, &TCNT3, &OCR3A, &TIMSK3},
  {16, 0xFFFF0000, prescaling_16_bit, &TCCR4A, &TCCR4B, &TCNT4, &OCR4A, &TIMSK4},
  {16, 0xFFFF0000, prescaling_16_bit, &TCCR5A, &TCCR5B, &TCNT5, &OCR5A, &TIMSK5},
  // {16, 0xFFFF0000, prescaling_16_bit, &TCCR1A, &TCCR1B, &TCNT1, &OCR1A, &TIMSK1},
  {8, 0xFFFFFF00, prescaling_timer_2, &TCCR2A, &TCCR2B, &TCNT2, &OCR2A, &TIMSK2},
  // {8, 0xFFFFFF00, prescaling_timer_2, &TCCR1A, &TCCR1A, &TCNT1, &OCR1A, &TIMSK1},
};

inline void set_timer_count(byte id, uint16_t val) {
  if(timers[id].bits==16)
    *((volatile uint16_t*)timers[id].tcnt) = val;
  else
    *((volatile uint8_t*)timers[id].tcnt) = (uint8_t) val;
}

inline void set_timer_period(byte id, uint16_t val) {
  if(timers[id].bits==16)
    *((volatile uint16_t*)timers[id].ocra) = val;
  else
    *((volatile uint8_t*)timers[id].ocra) = (uint8_t) val;
}

inline uint16_t get_timer_period(byte id) {
  if(timers[id].bits==16)
    return *((volatile uint16_t*)timers[id].ocra);
  else
    return *((volatile uint8_t*)timers[id].ocra);
}

uint16_t get_timer_period(byte id, uint16_t freq, byte* pre) {
  uint32_t period = (uint32_t) AVR_CLK_FREQ / (uint32_t) freq;//get the timer period
  byte prescaling = 1;

  while((period & timers[id].mask)&&(timers[id].pre_mul[prescaling])) {
    byte pre_mul = timers[id].pre_mul[prescaling];

    if(period & (1<<(pre_mul-1))) {
      period = (period>>pre_mul) + 1;
    } else {
      period = (period>>pre_mul);
    }

    prescaling++;
  }

  *pre = prescaling;
  return (uint16_t) period;
}

struct JobProgress {
  volatile bool running = false;
  volatile u32 remaining = 0;
  volatile EndConditionType end;
  void (* volatile callback)(const void*) = NULL;
  const void * volatile callback_args = NULL;
} current_jobs[SUBJOBS_PER_JOB];

#define STOP_JOB(id) { \
  current_jobs[id].running = false; \
  *timers[id].tccrnb = 0; \
  *timers[id].timsk = 0; \
}

// #define DO_STEP_FEED() (STEP_FEED_PORT ^= STEP_FEED_BIT)
#define DO_STEP_FEED() { \
    static byte step = 0; \
    step ^= STEP_FEED_BIT; \
    STEP_FEED_PORT = step; \
}

#define DO_STEP_CLAMP() (STEP_CLAMP_PORT ^= STEP_CLAMP_BIT)
#define DO_STEP_DRIVE() (STEP_DRIVE_PORT ^= STEP_DRIVE_BIT)
#define DO_STEP_NOOP()

//to be run in the ISRs
#define DO_JOB(id, step) { \
  step(); \
  if((--current_jobs[id].remaining) == 0) \
    STOP_JOB(id) \
}


#define TRIGGER_SG(id, pin) { \
  STOP_JOB(id); \
  detachInterrupt(digitalPinToInterrupt(pin)); \
}

void sg_0(void) {TRIGGER_SG(0, SG_FEED)}
void sg_1(void) {TRIGGER_SG(1, SG_CLAMP)}

ISR(TIMER2_COMPA_vect) { DO_JOB(3, DO_STEP_NOOP) }
// ISR(TIMER1_COMPA_vect) { DO_JOB(3, DO_STEP_NOOP) }
ISR(TIMER3_COMPA_vect) { DO_JOB(0, DO_STEP_FEED) }
ISR(TIMER4_COMPA_vect) { DO_JOB(1, DO_STEP_CLAMP) }
ISR(TIMER5_COMPA_vect) { DO_JOB(2, DO_STEP_DRIVE) }

Queue<Job,6> job_queue = Queue<Job,6>();
Queue<byte,6> job_size_queue = Queue<byte,6>();

int32_t drive_freq = 0;
bool drive_dir = false;

void clear_jobs() {
  cli();
  job_queue.clear();
  job_size_queue.clear();
  for(byte i=0; i<SUBJOBS_PER_JOB; i++) STOP_JOB(i);
  sei();
}

inline bool idempotent(Job j) {
  return j.en==KEEP && j.dir==KEEP &&
  (j.frequency==0 || j.end.ty==IMMEDIATE || (j.end.ty==COUNT&&j.end.cond==0)) &&
  j.callback == NULL;
}

bool queue_jobs(Jobs j) {
  byte count = 0;
  for(byte i=0; i<SUBJOBS_PER_JOB; i++) {
    if(!idempotent(j.jobs[i])) {
      count++;
      j.jobs[i].axis = i;
      if(!job_queue.push_bottom(j.jobs[i])) return false;
    }
  }

  return job_size_queue.push_bottom(count);
}

// void cancel_last_job() {
  // if(job_size_queue.count > 0) {
  //   for(byte i = job_size_queue.pop_bottom(); i>0; i--, job_queue.pop_bottom());
  // }
// }

byte open_jobs() {
  return (byte) min(min(job_queue.capacity() - job_queue.count(), job_size_queue.capacity() - job_size_queue.count()), 255);
}

bool job_done() {
  for(byte i=0; i<SUBJOBS_PER_JOB; i++){
    if(current_jobs[i].running) {
      return false;
    }
  }
  return true;
}

bool busy() { return job_queue.count()>0 || job_size_queue.count()>0 || job_done();}

void machine_loop() {

  for(byte i=0; i<SUBJOBS_PER_JOB; i++) {
    if(!current_jobs[i].running && current_jobs[i].callback!=NULL) {
      (*current_jobs[i].callback)(current_jobs[i].callback_args);
      current_jobs[i].callback = NULL;
    }
  }

  // static int last_time = millis();
  //
  // int time = millis();
  // if(time-last_time>1000) {
  //   Serial.print(clamp.TSTEP());
  //   Serial.print(" ");
  //   Serial.println(feed.TSTEP());
  //   last_time = time;
  // }

  //if the last command ended, advance the queue
  if(job_done()) {

    //enact the next job if there is one
    if(job_size_queue.count()>0){
      byte count = job_size_queue.pop_top();

      cli(); //make sure no random interrupt bs happens

        //loop over all the new jobs
        for(byte i=0; i<count; i++) {
          Job next = job_queue.pop_top();
          byte id = next.axis;

          if(next.dir!=KEEP) {
            switch(id) {
              case 0: digitalWrite(DIR_FEED, next.dir==SET ^ FEED_INVERT_DIR ? HIGH : LOW); break;
              case 1: digitalWrite(DIR_CLAMP, next.dir==SET ^ CLAMP_INVERT_DIR ? HIGH : LOW); break;
              case 2: digitalWrite(DIR_DRIVE, next.dir==SET ^ DRIVE_INVERT_DIR ? HIGH : LOW); break;
            }
          }

          if(next.en!=KEEP) {
            switch(id) {
              case 0: digitalWrite(EN_FEED, next.en==SET ^ FEED_INVERT_EN ? HIGH : LOW); break;
              case 1: digitalWrite(EN_CLAMP, next.en==SET ^ CLAMP_INVERT_EN ? HIGH : LOW); break;
              case 2: digitalWrite(EN_DRIVE, next.en==SET ^ DRIVE_INVERT_EN ? HIGH : LOW); break;
            }
          }

          //setup the end-condition, get if this job is already over
          //and setup how many cycles are remaining
          EndCondition end = next.end;
          current_jobs[id].end = end.ty;
          current_jobs[id].running = false;

          switch(end.ty) {
            case COUNT:
              if(end.cond > 0 && next.frequency > 0) {
                current_jobs[id].remaining = end.cond;
                current_jobs[id].running = true;
              }
              break;
            case STALL_GUARD:
              current_jobs[id].running = true;
              current_jobs[id].remaining = ~0;
              switch(id) {
                case 0:
                  feed.sgt(end.cond);
                  attachInterrupt(digitalPinToInterrupt(SG_FEED), sg_0, RISING);
                  break;
                case 1:
                  clamp.sgt(end.cond);
                  attachInterrupt(digitalPinToInterrupt(SG_CLAMP), sg_1, RISING);
                  break;
              }
              break;
            case FOREVER:
              current_jobs[id].running = true;
              current_jobs[id].remaining = ~0;
              break;
            case IMMEDIATE: break;
          }

          //setup the callback
          current_jobs[id].callback = next.callback;
          current_jobs[id].callback_args = next.callback_args;

          //setup the timers

          //clear the timer value
          set_timer_count(id,0);

          //clear the config
          //note we don't need prescaling because we wont really need events triggering
          //more than once a second
          *timers[id].tccrna = 0;

          if(current_jobs[id].running) {
            *timers[id].tccrnb = (1<<3); //clear the timer when it reaches OCRnA
            *timers[id].timsk = 2; //enable interrupt of OCRnA

            byte prescaling = 1;
            uint16_t period = get_timer_period(id, next.frequency, &prescaling);
            set_timer_period(id,period);

            *timers[id].tccrnb |= prescaling;

            // Serial.print(next.axis);
            // Serial.print(" ");
            // Serial.print(next.frequency);
            // Serial.print(" ");
            // Serial.print(period);
            // Serial.print(" ");
            // Serial.print(get_timer_period(id));
            // Serial.print(" ");
            // Serial.print(*timers[id].tccrnb,BIN);
            // Serial.print(" ");
            // Serial.print(end.cond);
            // Serial.print(" ");
            // Serial.println(current_jobs[id].remaining);
          } else {
            //disable the timer interrupt and clear the compare value
            *timers[id].tccrnb = 0;
            *timers[id].timsk = 0;
            set_timer_period(id,0);
          }

        }

      sei();
    } else {
      drive_freq = 0;
    }
  }
}

void machine_init() {

  SPI.begin();

  clamp.begin();
  clamp.rms_current(CLAMP_CURRENT);
  clamp.dedge(CLAMP_DEDGE);
  clamp.microsteps(CLAMP_MS);
  clamp.TCOOLTHRS(400);
  clamp.sgt(CLAMP_SGT);
  clamp.diag0_stall(true);

  feed.begin();
  feed.rms_current(FEED_CURRENT);
  feed.dedge(FEED_DEDGE);
  feed.microsteps(FEED_MS);
  feed.TCOOLTHRS(400);
  feed.sgt(FEED_SGT);
  feed.diag0_stall(true);

  pinMode(EN_DRIVE, OUTPUT);
  pinMode(STEP_DRIVE, OUTPUT);
  pinMode(DIR_DRIVE, OUTPUT);

  digitalWrite(EN_CLAMP, CLAMP_INVERT_EN ? HIGH : LOW);
  digitalWrite(EN_FEED, FEED_INVERT_EN ? HIGH : LOW);
  digitalWrite(EN_DRIVE, DRIVE_INVERT_EN ? HIGH : LOW);

 // digitalWrite(EN_CLAMP, LOW);
 // digitalWrite(EN_FEED, LOW);
 // digitalWrite(EN_DRIVE, LOW);
}
