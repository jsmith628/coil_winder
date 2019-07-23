
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
  sync = SyncEvents<3>();
  steps = 0;

  for(byte i=0; i<3; i++) {
    dirs[i] = false;

    end[i].ty = NONE;
    end[i].cond = 0;
    end[i].triggered = false;
  }
}

volatile bool running = false;
Job current_job = Job();

ISR(TIMER1_COMPA_vect) {

  static bool do_step[3] = {false,false,false};

  if(running) {

      current_job.steps++;
      current_job.sync.step(&do_step[0]);

      running = false;

      for(byte i=0; i<3; i++) {
        if(!current_job.end[0].triggered) {

          running = true;
          if(do_step[i]){
            switch(i) {
              case 2: step_drive(); break;
              case 0: step_feed(); break;
              case 1: step_clamp(); break;
            }
            if(current_job.end[i].ty == COUNT && current_job.steps>current_job.end[i].cond) {
              current_job.end[i].triggered = true;
            }
          }

        }
      }

  }

}

Queue<Job,4> job_queue = Queue<Job,4>();

bool queue_job(Job j) { return job_queue.push_bottom(j); }

void machine_loop() {
  //if the last command ended, advance the queue
  if(!running && job_queue.count()>0) {
    cli(); //make sure no random interrupt bs happens
      current_job = job_queue.pop_top();

        if(current_job.frequency==0) {
          //disable the timer
          TCCR1A = 0;
          TCCR1B = 0;
          OCR1A = 0;
          TIMSK1 = 0;
          running = false;
        } else {
          //setup the timer
          TCCR1A = 0;
          TCCR1B = 1 | (1<<WGM12);

          OCR1A = (AVR_CLK_FREQ / current_job.frequency);

          TIMSK1 = 2;
          running = true;
        }


    sei();
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
