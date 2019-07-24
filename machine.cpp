
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

//A handy container object for all of the timer registers
typedef struct {
    volatile uint16_t * tccr;
    volatile uint16_t * tcnt;
    volatile uint16_t * ocra;
    volatile uint16_t * ocrb;
    volatile uint16_t * ocrc;
    volatile uint8_t * timsk;
} Timer;

//all of the 16bit timers on the ATMEGA2560
Timer timers[4] = {
  {(volatile uint16_t*) &TCCR1A, &TCNT1, &OCR1A, &OCR1B, &OCR1C, &TIMSK1},
  {(volatile uint16_t*) &TCCR3A, &TCNT3, &OCR3A, &OCR3B, &OCR3C, &TIMSK3},
  {(volatile uint16_t*) &TCCR4A, &TCNT4, &OCR4A, &OCR4B, &OCR4C, &TIMSK4},
  {(volatile uint16_t*) &TCCR5A, &TCNT5, &OCR5A, &OCR5B, &OCR5C, &TIMSK5},
};

struct JobProgress {
  volatile bool running = false;
  volatile u32 remaining = 0;
  volatile EndConditionType end;
} current_jobs[SUBJOBS_PER_JOB];

//to be run in the ISRs
inline void do_job(byte id) {

  //dont do anything if we're done
  if(current_jobs[id].running){

    //step the proper stepper
    switch(id) {
      case 2: step_drive(); break;
      case 0: step_feed(); break;
      case 1: step_clamp(); break;
    }

    //check if we need to stop this job
    if(current_jobs[id].end == COUNT) {
      //update the progress, and check if we've hit 0
      if((--current_jobs[id].remaining) == 0) {
        //mark this job as done
        current_jobs[id].running = false;
        *timers[id].timsk = 0; //if we're done we don't need to run this interrupt again
      }
    }

  }

}


ISR(TIMER1_COMPA_vect) { do_job(0); }
ISR(TIMER3_COMPA_vect) { do_job(1); }
ISR(TIMER4_COMPA_vect) { do_job(2); }
ISR(TIMER5_COMPA_vect) { do_job(3); }

Queue<Jobs,4> job_queue = Queue<Jobs,4>();

bool queue_jobs(Jobs j) { return job_queue.push_bottom(j); }
void clear_jobs() {
  job_queue.clear();
  for(byte i=0; i<SUBJOBS_PER_JOB; current_jobs[i++].running = false);
}

bool job_done() {
  for(byte i=0; i<SUBJOBS_PER_JOB; i++){
    if(current_jobs[i].running) {
      return true;
    }
  }
  return false;
}

bool busy() { return job_queue.count()>0 || job_done();}

void machine_loop() {

  //if the last command ended, advance the queue
  if(job_done()) {

    //enact the next job if there is one
    if(job_queue.count()>0){
      Jobs j = job_queue.pop_top();
      Job next[SUBJOBS_PER_JOB] = {j.jobs[0],j.jobs[1],j.jobs[2],j.jobs[3]};

      //set the dir pins
      if(next[0].dir!=KEEP) digitalWrite(DIR_FEED, next[0].dir==SET ^ FEED_INVERT_DIR ? HIGH : LOW);
      if(next[1].dir!=KEEP) digitalWrite(DIR_CLAMP, next[1].dir==SET ^ CLAMP_INVERT_DIR ? HIGH : LOW);
      if(next[2].dir!=KEEP) digitalWrite(DIR_DRIVE, next[2].dir==SET ^ DRIVE_INVERT_DIR ? HIGH : LOW);

      //set the enable pins
      if(next[0].en!=KEEP) digitalWrite(EN_FEED, next[0].en ^ FEED_INVERT_EN ? HIGH : LOW);
      if(next[1].en!=KEEP) digitalWrite(EN_CLAMP, next[1].en ^ CLAMP_INVERT_EN ? HIGH : LOW);
      if(next[2].en!=KEEP) digitalWrite(EN_DRIVE, next[2].en ^ DRIVE_INVERT_EN ? HIGH : LOW);

      cli(); //make sure no random interrupt bs happens

        for(byte i=0; i<SUBJOBS_PER_JOB; i++) {

          //setup the end-condition, get if this job is already over
          //and setup how many cycles are remaining
          EndCondition end = next[i].end;
          current_jobs[i].end = end.ty;
          switch(end.ty) {
            case COUNT:
              if(end.cond > 0) {
                current_jobs[i].remaining = end.cond;
                current_jobs[i].running = true;
              }
              break;
            case STALL_GUARD:
              current_jobs[i].running = true;
              switch(i) {
                case 0: feed.sgt(end.cond); break;
                case 1: clamp.sgt(end.cond); break;
              }
            case FOREVER: current_jobs[i].running = true; break;
            case IMMEDIATE: current_jobs[i].running = false; break;
          }

          //setup the timers

          //clear the timer value
          *timers[i].tcnt = 0;

          //clear the config
          //note we don't need prescaling because we wont really need events triggering
          //more than once a second
          *timers[i].tccr = 0;

          if(current_jobs[i].running) {
            *timers[i].tccr |= (1<<WGM12); //clear the timer when it reaches OCRnA
            *timers[i].timsk = 2; //enable interrupt of OCRnA
            *timers[i].ocra = (AVR_CLK_FREQ / next[i].frequency);//get the timer period
          } else {
            //disable the timer interrupt and clear the compare value
            *timers[i].timsk = 0;
            *timers[i].ocra = 0;
          }

        }

      sei();
    }
  }
}

void machine_init() {

  SPI.begin();

  clamp.begin();
  clamp.rms_current(CLAMP_CURRENT);
  clamp.dedge(CLAMP_DEDGE);
  clamp.microsteps(CLAMP_MS);
  clamp.TCOOLTHRS(0xFFFFF);
  clamp.sgt(CLAMP_SGT);

  feed.begin();
  feed.rms_current(FEED_CURRENT);
  feed.dedge(FEED_DEDGE);
  feed.microsteps(FEED_MS);
  feed.TCOOLTHRS(0xFFFFF);
  feed.sgt(FEED_SGT);

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
