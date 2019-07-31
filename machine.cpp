
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
    const byte bits;
    const uint32_t mask;
    volatile uint8_t * tccrna;
    volatile uint8_t * tccrnb;
    volatile uint16_t * tcnt;
    volatile uint16_t * ocra;
    volatile uint16_t * ocrb;
    volatile uint16_t * ocrc;
    volatile uint8_t * timsk;
} Timer;

//all of the 16bit timers on the ATMEGA2560
Timer timers[4] = {
  {16, 0xFFFF0000, &TCCR3A, &TCCR3B, &TCNT3, &OCR3A, &OCR3B, &OCR3C, &TIMSK3},
  {16, 0xFFFF0000, &TCCR4A, &TCCR4B, &TCNT4, &OCR4A, &OCR4B, &OCR4C, &TIMSK4},
  {16, 0xFFFF0000, &TCCR5A, &TCCR5B, &TCNT5, &OCR5A, &OCR5B, &OCR5C, &TIMSK5},
  {8, 0xFFFFFF00, &TCCR1A, &TCCR1A, &TCNT1, &OCR1A, &OCR1B, &OCR1C, &TIMSK1},
  // {8, 0xFFFFFF00, &TCCR2A, &TCCR2A, &TCNT2, &OCR2A, &OCR2B, NULL, &TIMSK2},
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
        *timers[id].tccrnb = 0; //if we're done we don't need to run this interrupt again
        *timers[id].timsk = 0; //if we're done we don't need to run this interrupt again
      }
    }

  }

}

inline void trigger_sg(byte id) {
  current_jobs[id].running = false;
  *timers[id].tccrnb = 0;
  *timers[id].timsk = 0;
  switch(id) {
    case 0: detachInterrupt(digitalPinToInterrupt(SG_FEED)); break;
    case 1: detachInterrupt(digitalPinToInterrupt(SG_CLAMP)); break;
  }
}

void sg_0(void) {trigger_sg(0);}
void sg_1(void) {trigger_sg(1);}

ISR(TIMER1_COMPA_vect) { do_job(3); }
ISR(TIMER3_COMPA_vect) { do_job(0); }
ISR(TIMER4_COMPA_vect) { do_job(1); }
ISR(TIMER5_COMPA_vect) { do_job(2); }

Queue<Jobs,6> job_queue = Queue<Jobs,6>();

int32_t drive_freq = 0;
bool drive_dir = false;

void clear_jobs() {
  job_queue.clear();
  for(byte i=0; i<SUBJOBS_PER_JOB; current_jobs[i++].running = false);
}

bool queue_jobs(Jobs j) {

  return job_queue.push_bottom(j);
  // #define DRIVE_INDEX 2
  //
  // int32_t new_freq = j.jobs[2].frequency;
  // if(j.jobs[DRIVE_INDEX].dir==SET || j.jobs[DRIVE_INDEX].dir==KEEP&&drive_freq<0) new_freq*=-1;
  //
  // #define DRIVE_MAX_ACCEL_STEPS DRIVE_MAX_ACCEL*DRIVE_STEPS_PER_REV
  //
  // Serial.print(drive_freq);
  // Serial.print(" ");
  // Serial.println(new_freq);
  //
  // int32_t df = new_freq-drive_freq;
  // if(abs(df) > DRIVE_MAX_ACCEL_STEPS) {
  //   Jobs new_jobs = j;
  //
  //   //step up the acceleration
  //   if(df<0){
  //     drive_freq -= DRIVE_MAX_ACCEL_STEPS;
  //   } else {
  //     drive_freq += DRIVE_MAX_ACCEL_STEPS;
  //   }
  //   new_jobs.jobs[DRIVE_INDEX].frequency = abs(drive_freq);
  //
  //   //figure out how to scale everything else
  //   float factor = abs(drive_freq / (float) new_freq);
  //
  //
  //   //rescale and modify the duration of everything else
  //   for(byte i=0; i<SUBJOBS_PER_JOB; i++) {
  //     if(i!=DRIVE_INDEX) new_jobs.jobs[i].frequency *= factor;
  //     uint16_t dt = (uint16_t) ((float) DRIVE_ACCEL_RESOLUTION * new_jobs.jobs[i].frequency);
  //     if(j.jobs[i].end.ty == COUNT) {
  //       if(j.jobs[i].end.cond<dt){
  //         new_jobs.jobs[i].end.cond = j.jobs[i].end.cond;
  //         j.jobs[i].end.cond = 0;
  //       } else {
  //         new_jobs.jobs[i].end.cond = dt;
  //         j.jobs[i].end.cond -= dt;
  //       }
  //     }
  //     Serial.print(dt);
  //     Serial.print(" ");
  //   }
  //   Serial.println(factor);
  //
  //   if(job_queue.push_bottom(new_jobs)) {
  //     return queue_jobs(j);
  //   } else {
  //     return false;
  //   }
  // } else {
  //   drive_freq = new_freq;
  //   return job_queue.push_bottom(j);
  // }

}

bool job_done() {
  for(byte i=0; i<SUBJOBS_PER_JOB; i++){
    if(current_jobs[i].running) {
      return false;
    }
  }
  return true;
}

bool busy() { return job_queue.count()>0 || job_done();}

void machine_loop() {

  // static int last_time = millis();
  //
  // int time = millis();
  // if(time-last_time>1000) {
  //   Serial.print(clamp.sg_result());
  //   Serial.print(" ");
  //   Serial.println(feed.sg_result());
  //   last_time = time;
  // }

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
      if(next[0].en!=KEEP) digitalWrite(EN_FEED, next[0].en==SET ^ FEED_INVERT_EN ? HIGH : LOW);
      if(next[1].en!=KEEP) digitalWrite(EN_CLAMP, next[1].en==SET ^ CLAMP_INVERT_EN ? HIGH : LOW);
      if(next[2].en!=KEEP) digitalWrite(EN_DRIVE, next[2].en==SET ^ DRIVE_INVERT_EN ? HIGH : LOW);

      cli(); //make sure no random interrupt bs happens

        for(byte i=0; i<SUBJOBS_PER_JOB; i++) {

          //setup the end-condition, get if this job is already over
          //and setup how many cycles are remaining
          EndCondition end = next[i].end;
          current_jobs[i].end = end.ty;
          current_jobs[i].running = false;

          switch(end.ty) {
            case COUNT:
              if(end.cond > 0 && next[i].frequency > 0) {
                current_jobs[i].remaining = end.cond;
                current_jobs[i].running = true;
              }
              break;
            case STALL_GUARD:
              current_jobs[i].running = true;
              switch(i) {
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
            case FOREVER: current_jobs[i].running = true; break;
            case IMMEDIATE: break;
          }

          //setup the timers


          //clear the timer value
          *timers[i].tcnt = 0;

          //clear the config
          //note we don't need prescaling because we wont really need events triggering
          //more than once a second
          *timers[i].tccrna = 0;

          if(current_jobs[i].running) {
            *timers[i].tccrnb = (1<<3); //clear the timer when it reaches OCRnA
            *timers[i].timsk = 2; //enable interrupt of OCRnA
            uint32_t period = (uint32_t) AVR_CLK_FREQ / (uint32_t) next[i].frequency;//get the timer period
            uint32_t p = period;

            byte prescaling = 1;

            uint32_t mask = i<3 ? 0xFFFF0000 : 0xFFFFFF00;

            while((period & mask)&&prescaling<0b101) {
              if(prescaling<=2){
                if((period & 0b111)>=4){
                  period = (period>>3)+1;
                }else {
                  period = period>>3;
                }
              } else {
                if((period & 0b11)>=2){
                  period = (period>>2) + 1;
                } else {
                  period = period>>2;
                }
              }
              prescaling++;
            }

            *timers[i].tccrnb |= prescaling;
            *timers[i].ocra = (uint16_t) period;

            Serial.print(next[i].frequency);
            Serial.print(" ");
            Serial.print(p);
            Serial.print(" ");
            Serial.print(*timers[i].ocra);
            Serial.print(" ");
            Serial.print(*timers[i].tccrnb,BIN);
            Serial.print(" ");
            Serial.println(end.cond);
          } else {
            //disable the timer interrupt and clear the compare value
            *timers[i].tccrnb = 0; //clear the timer when it reaches OCRnA
            *timers[i].timsk = 0;
            *timers[i].ocra = 0;
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
  clamp.TCOOLTHRS(0xFFFFF);
  clamp.sgt(CLAMP_SGT);
  clamp.diag0_stall(true);

  feed.begin();
  feed.rms_current(FEED_CURRENT);
  feed.dedge(FEED_DEDGE);
  feed.microsteps(FEED_MS);
  feed.TCOOLTHRS(0xFFFFF);
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
