
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

uint16_t get_timer_period(uint16_t freq, byte* pre, const uint32_t mask) {
  uint32_t period = (uint32_t) AVR_CLK_FREQ / (uint32_t) freq;//get the timer period
  byte prescaling = 1;

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

  *pre = prescaling;
  return (uint16_t) period;
}

#define LUT_LENGTH 16

typedef struct {
  byte current = 0, num = 0;
  byte prescaling[LUT_LENGTH];
  uint16_t periods[LUT_LENGTH];
  uint16_t time = 0;
  uint16_t intervals[LUT_LENGTH];
} AccelsLUT;


struct JobProgress {
  volatile bool running = false;
  volatile u32 remaining = 0;
  volatile AccelsLUT accels;
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

    // byte lut = current_jobs[id].accels.current;
    // byte num = current_jobs[id].accels.num;
    //
    // if(lut < num) {
    //   if(--current_jobs[id].accels.time == 0) {
    //     if(++current_jobs[id].accels.current < num) {
    //       current_jobs[id].accels.time = current_jobs[id].accels.intervals[lut+1];
    //       *timers[id].tccrnb = (*timers[id].tccrnb & ~0b111) | current_jobs[id].accels.prescaling[lut+1];
    //       *timers[id].ocra = current_jobs[id].accels.periods[lut+1];
    //     }
    //   }
    // }

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
}

byte open_jobs() {
  return (byte) min(job_queue.capacity() - job_queue.count(), 255);
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
  //   Serial.print(clamp.TSTEP());
  //   Serial.print(" ");
  //   Serial.println(feed.TSTEP());
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

            // const byte max = 10;
            // current_jobs[i].accels.current = 0;
            // current_jobs[i].accels.num = max;
            //
            // uint16_t start = next[i].frequency/4;
            // uint16_t end = next[i].frequency;
            //
            // float duration = 1.0;
            //
            // for(byte j=0; j<max; j++) {
            //   byte prescaling = 1;
            //   uint16_t period = get_timer_period(start + j*(end-start)/max, &prescaling, timers[i].mask);
            //   current_jobs[i].accels.periods[j] = period;
            //   current_jobs[i].accels.prescaling[j] = prescaling;
            //   current_jobs[i].accels.intervals[j] = (uint16_t) ((duration * AVR_CLK_FREQ) / period);
            // }
            // current_jobs[i].accels.time = current_jobs[i].accels.intervals[0];
            //
            // *timers[i].ocra = current_jobs[i].accels.periods[0];
            // *timers[i].tccrnb |= current_jobs[i].accels.prescaling[0];

            byte prescaling = 1;
            *timers[i].ocra = get_timer_period(next[i].frequency, &prescaling, timers[i].mask);
            *timers[i].tccrnb |= prescaling;

            Serial.print(next[i].frequency);
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
