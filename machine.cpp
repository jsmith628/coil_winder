
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

Job::Job() {
  frequency = 0;

  for(byte i=0; i<3; i++) {
    dirs[i] = false;
    enabled[i] = false;
    ratio[i] = 0;

    end[i].ty = NONE;
    end[i].cond = 0;
    end[i].triggered = false;
  }
}

struct JobProgress {
  volatile bool running = false;
  volatile SyncEvents<3> sync;
  volatile EndCondition end[3];
  volatile u32 step_num = 0;
} current_job;

ISR(TIMER1_COMPA_vect) {

  static bool do_step[3] = {false,false,false};

  if(current_job.running) {

      current_job.sync.step(&do_step[0]);
      current_job.running = false;

      for(byte i=0; i<3; i++) {
        if(!current_job.end[0].triggered) {

          bool trig = true;
          if(do_step[i]){

            //step the proper stepper
            switch(i) {
              case 2: step_drive(); break;
              case 0: step_feed(); break;
              case 1: step_clamp(); break;
            }

            //test if this stepper needs to stop
            if(
              (current_job.end[i].ty == COUNT && current_job.step_num>current_job.end[i].cond) ||
              (current_job.end[i].ty == IMMEDIATE)
            ) {
              current_job.end[i].triggered = true;
              trig = false;
            }

          }

          if(trig) current_job.running = true;

        }
      }

      //we have done another step
      current_job.step_num++;

  }

}

Queue<Job,4> job_queue = Queue<Job,4>();

bool queue_job(Job j) { return job_queue.push_bottom(j); }
void clear_jobs() {
  job_queue.clear();
  current_job.running = false;
}

void machine_loop() {
  //if the last command ended, advance the queue
  if(!running) {

    TIMSK1 = 0; //disable timer interrupts

    //enact the next job if there is one
    if(job_queue.count()>0){
      cli(); //make sure no random interrupt bs happens
        Job next_job = job_queue.pop_top();

        //set the dir pins
        digitalWrite(DIR_FEED, next_job.dirs[0] ^ FEED_INVERT_DIR ?HIGH:LOW);
        digitalWrite(DIR_CLAMP, next_job.dirs[1] ^ CLAMP_INVERT_DIR ?HIGH:LOW);
        digitalWrite(DIR_DRIVE, next_job.dirs[2] ^ DRIVE_INVERT_DIR ?HIGH:LOW);

        //set the enable pins
        digitalWrite(EN_FEED, next_job.enabled[0] ^ FEED_INVERT_EN ?HIGH:LOW);
        digitalWrite(EN_CLAMP, next_job.enabled[1] ^ CLAMP_INVERT_EN ?HIGH:LOW);
        digitalWrite(EN_DRIVE, next_job.enabled[2] ^ DRIVE_INVERT_EN ?HIGH:LOW);

        current_job.step_num = 0;
        current_job.sync = Sync(next_job.sync);

        current_job.running = false;
        for(byte i=0; i<3; i++) {
          struct EndCondition end = next_job.end[i];
          current_job.end[i] = end;
          switch(end.ty) {
            case COUNT: if(end.cond > 0) running = true; break;
            case STALL_GUARD:
              switch(i) {
                case 0: feed.sgt(end.cond); break;
                case 1: clamp.sgt(end.cond); break;
              }
            case FOREVER: running = true; break;
            case IMMEDIATE: break;
          }
        }

        //timer config
        if(next_job.frequency==0) {
          //disable the timer
          TCCR1A = TCCR1B = OCR1A = TIMSK1 = 0;
        } else {
          //setup the timer
          TCCR1A = 0;

          TCCR1B = 1; //no prescaling
          TCCR1B |= (1<<WGM12); //clear the timer when it reaches OCR1A

          //the period (in clock cycles) of the step function
          OCR1A = (AVR_CLK_FREQ / next_job.frequency);

          TIMSK1 = 2; //enable timer interrupt when it reaches OCR1A
        }

      sei();
    }
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

  digitalWrite(EN_CLAMP, CLAMP_INVERT_EN ? HIGH : LOW);
  digitalWrite(EN_FEED, FEED_INVERT_EN ? HIGH : LOW);
  digitalWrite(EN_DRIVE, DRIVE_INVERT_EN ? HIGH : LOW);

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
