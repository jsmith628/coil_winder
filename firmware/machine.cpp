
#include <TMC2130Stepper.h>
#include "machine.h"

#define signum(x) ((x)<0 ? -1 : (x)==0 ? 0 : 1)

const uint8_t prescaling_16_bit[6] = {1,3,3,2,2,0};
const uint8_t prescaling_timer_2[8] = {1,3,2,1,1,1,2,0};

//A handy container object for all of the timer registers
template<typename T>
struct Timer {
    const byte * const prescaling;

    volatile T * const tcnt;
    volatile T * const ocra;
    volatile T * const ocrb;

    volatile uint8_t * const tccrna;
    volatile uint8_t * const tccrnb;
    volatile uint8_t * const timsk;
};

//all of the stepper timers
const Timer<uint16_t> timers[4] = {
  {prescaling_16_bit, &TCNT3, &OCR3A, &OCR3B, &TCCR3A, &TCCR3B, &TIMSK3},
  {prescaling_16_bit, &TCNT4, &OCR4A, &OCR4B, &TCCR4A, &TCCR4B, &TIMSK4},
  {prescaling_16_bit, &TCNT5, &OCR5A, &OCR5B, &TCCR5A, &TCCR5B, &TIMSK5},
  {prescaling_16_bit, &TCNT1, &OCR1A, &OCR1B, &TCCR1A, &TCCR1B, &TIMSK1}
};

const Timer<uint8_t> accel_timer = {
  prescaling_timer_2, &TCNT2, &OCR2A, &OCR2B, &TCCR2A, &TCCR2B, &TIMSK2
};

template<typename T>
struct Period {
  const uint8_t prescale;
  const T period;
};

template<typename T>
constexpr Period<T> period_from_frequency(uint16_t f, const uint8_t * const pre) {

  //if the frequency is 0, then the timer prescaler should be 0 to turn off the timer
  if(f==0) return {0,0};

  //compute the absolute period
  uint32_t period = (uint32_t) AVR_CLK_FREQ / (uint32_t) f;

  //construct the type mask
  uint32_t mask = ~0;
  for(T i=1; i!=0; i<<=1) mask &= (~((uint32_t) i));

  //now, loop through the timer prescale multipliers until the period is in range
  uint8_t prescaling = 1;
  while((period & mask) && pre[prescaling]!=0) {
    uint8_t pre_mul = pre[prescaling];

    if(period & (1<<(pre_mul-1))) {
      //round up
      period = (period>>pre_mul) + 1;
    } else {
      //round down
      period = (period>>pre_mul);
    }

    prescaling++;
  }

  return {prescaling, (T) period};
}

#define LUT1(f,step,ty)     (period_from_frequency<ty>(f, prescaling_16_bit))
#define LUT2(f,step,ty)     LUT1(f,step,ty),     LUT1((f+1*step),step,ty)
#define LUT4(f,step,ty)     LUT2(f,step,ty),     LUT2((f+2*step),step,ty)
#define LUT8(f,step,ty)     LUT4(f,step,ty),     LUT4((f+4*step),step,ty)
#define LUT16(f,step,ty)    LUT8(f,step,ty),     LUT8((f+8*step),step,ty)
#define LUT32(f,step,ty)    LUT16(f,step,ty),    LUT16((f+16*step),step,ty)
#define LUT64(f,step,ty)    LUT32(f,step,ty),    LUT32((f+32*step),step,ty)
#define LUT128(f,step,ty)   LUT64(f,step,ty),    LUT64((f+64*step),step,ty)
#define LUT256(f,step,ty)   LUT128(f,step,ty),   LUT128((f+128*step),step,ty)
#define LUT512(f,step,ty)   LUT256(f,step,ty),   LUT256((f+256*step),step,ty)
#define LUT1024(f,step,ty)  LUT512(f,step,ty),   LUT512((f+512*step),step,ty)
#define LUT2048(f,step,ty)  LUT1024(f,step,ty),  LUT1024((f+1024*step),step,ty)
#define LUT4096(f,step,ty)  LUT2048(f,step,ty),  LUT2048((f+2048*step),step,ty)
#define LUT8192(f,step,ty)  LUT4096(f,step,ty),  LUT4096((f+4096*step),step,ty)
#define LUT16384(f,step,ty) LUT8192(f,step,ty),  LUT8192((f+8192*step),step,ty)

#define _CAT(x,y) x ## y
#define LUT_INIT(start, step, size) \
  const PROGMEM Period<uint16_t> periodsLUT[size] = { \
    _CAT(LUT, size)(start, step, uint16_t) \
  };

LUT_INIT(PERIOD_LUT_FREQ_START, (1<<PERIOD_LUT_FREQ_STEP), PERIOD_LUT_SIZE)

struct JobProgress {

  volatile bool running = false;
  volatile bool accelerating = false;
  volatile bool paused = false;

  volatile uint32_t total = 0;
  volatile uint32_t remaining = 0;
  volatile  int32_t last = 0;

  volatile float target_frequency = 0;
  volatile float frequency = 0;
  volatile float acceleration = 0;

  volatile uint8_t paused_timsk = 0;
  volatile EndConditionType end = IMMEDIATE;

  void (* volatile callback)(const void*) = NULL;
  const void * volatile callback_args = NULL;

} current_jobs[SUBJOBS_PER_JOB];

volatile float max_acceleration = DEFAULT_MAX_ACCELERATION * DRIVE_STEPS_PER_REV * ACCEL_TIME_RESOLUTION;
volatile uint16_t start_acceleration = (uint16_t) (DEFAULT_BASE_SPEED * DRIVE_STEPS_PER_REV);

int32_t steps_moved(uint8_t axis) {return current_jobs[axis].last;}

#define STOP_JOB(id) { \
  current_jobs[id].running = false; \
  current_jobs[id].last = \
    (current_jobs[id].frequency < 0 ? -1 : 1) * \
    ((int32_t) current_jobs[id].total - (int32_t) current_jobs[id].remaining); \
  *timers[id].tccrnb = 0; \
  *timers[id].timsk = 0; \
}

#ifdef FEED_DEDGE
  #define DO_STEP_FEED() (STEP_FEED_PORT ^= 1<<STEP_FEED_BIT)
#else
  #define DO_STEP_FEED() (STEP_FEED_PORT |= 1<<STEP_FEED_BIT)
#endif

#ifdef CLAMP_DEDGE
  #define DO_STEP_CLAMP() (STEP_CLAMP_PORT ^= 1<<STEP_CLAMP_BIT)
#else
  #define DO_STEP_CLAMP() (STEP_CLAMP_PORT |= 1<<STEP_CLAMP_BIT)
#endif

#ifdef DRIVE_DEDGE
  #define DO_STEP_DRIVE() (STEP_DRIVE_PORT ^= 1<<STEP_DRIVE_BIT)
#else
  #define DO_STEP_DRIVE() (STEP_DRIVE_PORT |= 1<<STEP_DRIVE_BIT)
#endif

#define DO_STEP_NOOP()

inline bool stepper_dedge(uint8_t id) {
  switch(id) {
    #ifndef FEED_DEDGE
      case 0: return true;
    #endif
    #ifndef CLAMP_DEDGE
      case 1: return true;
    #endif
    #ifndef DRIVE_DEDGE
      case 2: return true;
    #endif
  }
  return false;
}


//to be run in the ISRs
#define DO_JOB(id, step) { \
  step(); \
  if((--current_jobs[id].remaining) == 0) \
    STOP_JOB(id) \
}


#define TRIGGER_SG(id, pin) { \
  if(!current_jobs[id].paused){ \
    STOP_JOB(id); \
    detachInterrupt(digitalPinToInterrupt(pin)); \
  } \
}

void sg_0(void) {TRIGGER_SG(0, SG_FEED)}
void sg_1(void) {TRIGGER_SG(1, SG_CLAMP)}

ISR(TIMER3_COMPA_vect) { DO_JOB(0, DO_STEP_FEED) }
ISR(TIMER4_COMPA_vect) { DO_JOB(1, DO_STEP_CLAMP) }
ISR(TIMER5_COMPA_vect) { DO_JOB(2,  DO_STEP_DRIVE) }
// ISR(TIMER1_COMPA_vect) { DO_JOB(3, DO_STEP_NOOP) }

#define DIRECT_REGISTER(port) (((uint8_t) port) <= 0x1F)

#ifndef FEED_DEDGE
  ISR(TIMER3_COMPB_vect, ISR_NAKED) { STEP_FEED_PORT &= ~(1<<STEP_FEED_BIT); reti(); }
#endif

#ifndef CLAMP_DEDGE
  ISR(TIMER4_COMPB_vect, ISR_NAKED) { STEP_CLAMP_PORT &= ~(1<<STEP_CLAMP_BIT); reti(); }
#endif

#ifndef DRIVE_DEDGE
  ISR(TIMER5_COMPB_vect, ISR_NAKED) { STEP_DRIVE_PORT &= ~(1<<STEP_DRIVE_BIT); reti(); }
#endif

void set_enable(uint8_t axis, bool set) {
  switch(axis) {
    case 0: digitalWrite(EN_FEED, (set ^ FEED_INVERT_EN) ? HIGH : LOW); break;
    case 1: digitalWrite(EN_CLAMP, (set ^ CLAMP_INVERT_EN) ? HIGH : LOW); break;
    case 2: digitalWrite(EN_DRIVE, (set ^ DRIVE_INVERT_EN) ? HIGH : LOW); break;
  }
}

void enable_stepper(uint8_t axis) { set_enable(axis, true); }
void disable_stepper(uint8_t axis) { set_enable(axis, false); }

void set_max_acceleration(uint16_t accel) {
  max_acceleration = ACCEL_TIME_RESOLUTION * (float) accel;
}

void set_start_acceleration(uint16_t speed) {
  start_acceleration = (float) speed;
}

inline void set_frequency(uint8_t i, float freq) {

  uint16_t f = (uint16_t) abs(freq);
  uint8_t prescale;
  uint16_t period;
  if(f >= PERIOD_LUT_FREQ_START) {
    uint16_t index = f - PERIOD_LUT_FREQ_START;
    prescale = pgm_read_byte_near(&periodsLUT[index].prescale);
    period = pgm_read_word_near(&periodsLUT[index].period);
  } else {
    prescale = period_from_frequency<uint16_t>(f, prescaling_16_bit).prescale;
    period = period_from_frequency<uint16_t>(f, prescaling_16_bit).period;
  }

  *timers[i].tccrnb = (*timers[i].tccrnb & (~0b111)) | prescale;
  *timers[i].ocra = period;
  *timers[i].ocrb = stepper_dedge(i) ? period >> 1 : 0;
}

ISR(TIMER2_COMPA_vect) {
  bool done = true;

  for(uint8_t i=0; i<SUBJOBS_PER_JOB; i++) {
    if(current_jobs[i].accelerating) {
      float target = current_jobs[i].target_frequency;
      float f = current_jobs[i].frequency;

      float sign = signum(target - f);
      if(!current_jobs[i].paused) f += sign * current_jobs[i].acceleration;

      if(sign != signum(target - f)) {
        f = target;
        current_jobs[i].accelerating = false;
      } else {
        done = false;
      }

      current_jobs[i].frequency = f;
      set_frequency(i, f);
    }
  }

  if(done) {
    *accel_timer.tccrnb = 0; \
    *accel_timer.timsk = 0; \
  }
}

void pause_jobs(){
  cli();
  for(byte i=0; i<4; i++) {
    if(!current_jobs[i].paused) {
      current_jobs[i].paused = true;
      current_jobs[i].paused_timsk = *timers[i].timsk;
      *timers[i].timsk = 0;
    }
  }
  sei();
}

void resume_jobs(){
  cli();
  for(byte i=0; i<4; i++) {
    if(!current_jobs[i].paused) {
      current_jobs[i].paused = false;
      *timers[i].timsk = current_jobs[i].paused_timsk;
    }
  }
  sei();
}

Queue<Job,JOB_QUEUE_ORDER> job_queue = Queue<Job,JOB_QUEUE_ORDER>();
Queue<byte,JOB_QUEUE_ORDER> job_size_queue = Queue<byte,JOB_QUEUE_ORDER>();

void clear_job_queue() {
  job_queue.clear();
  job_size_queue.clear();
}

void clear_jobs() {
  cli();
  clear_job_queue();
  for(byte i=0; i<SUBJOBS_PER_JOB; i++) STOP_JOB(i);
  sei();
}


inline bool idempotent(Job j) {
  return (j.frequency==0 || j.end.ty==IMMEDIATE || (j.end.ty==COUNT&&j.end.cond==0)) && j.callback == NULL;
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

bool job_queue_open() {
  return job_queue.available()>=4 && job_size_queue.available()>=2;
}

bool job_done() {
  for(byte i=0; i<SUBJOBS_PER_JOB; i++){
    if(current_jobs[i].running || current_jobs[i].callback!=NULL) {
      return false;
    }
  }
  return true;
}

bool paused() {
  for(byte i=0; i<SUBJOBS_PER_JOB; i++){
    if(current_jobs[i].paused) {
      return true;
    }
  }
  return false;
}

bool busy() { return job_queue.count()>0 || job_size_queue.count()>0 || !job_done();}

typedef TMC2130Stepper Stepper;

Stepper clamp = Stepper(EN_CLAMP, DIR_CLAMP, STEP_CLAMP, CS_CLAMP);
Stepper feed = Stepper(EN_FEED, DIR_FEED, STEP_FEED, CS_FEED);

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
  if(job_done() && !paused()) {

    //enact the next job if there is one
    if(job_size_queue.count()>0){
      byte count = job_size_queue.pop_top();

      //guarrantee that the timers are 100% turned off and not running just in case
      *accel_timer.timsk = 0;
      for(uint8_t i=0; i<SUBJOBS_PER_JOB; i++) {
        *timers[i].timsk = 0;
      }

      //NOTE: we cannot turn off interrupts in this sections because then it messes up
      //the Serial communication buffering for some reason

      //loop over all the new jobs
      for(uint8_t i=0; i<count; i++) {
        Job next = job_queue.pop_top();
        byte id = next.axis;

        //setup the end-condition, get if this job is already over
        //and setup how many cycles are remaining
        EndCondition end = next.end;
        current_jobs[id].end = end.ty;
        current_jobs[id].running = false;

        switch(end.ty) {
          case COUNT:
            if(end.cond > 0 && next.frequency != 0) {
              current_jobs[id].total = current_jobs[id].remaining = end.cond;
              current_jobs[id].running = true;
            }
            break;
          case STALL_GUARD:
            current_jobs[id].running = true;
            current_jobs[id].total = current_jobs[id].remaining = ~0;
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
            current_jobs[id].total = current_jobs[id].remaining = ~0;
            break;
          case IMMEDIATE: break;
        }

        //setup the callback
        current_jobs[id].callback = next.callback;
        current_jobs[id].callback_args = next.callback_args;

        //set the target speed
        if(current_jobs[id].running) {
          current_jobs[id].target_frequency = (float) next.frequency;
        } else {
          current_jobs[id].target_frequency = 0;
          current_jobs[id].frequency = 0;
          current_jobs[id].acceleration = 0;
          current_jobs[id].accelerating = false;
        }

      }

      //
      //Compute the acceleration of each axis
      //

      const uint8_t drive_axis = 2;

      float sign = signum(current_jobs[drive_axis].frequency);
      float mag = abs(current_jobs[drive_axis].frequency);
      float tar_sign = signum(current_jobs[drive_axis].target_frequency);
      float tar_mag = abs(current_jobs[drive_axis].target_frequency);
      float start_accel = (float) start_acceleration;

      float f_start[SUBJOBS_PER_JOB];
      float accel[SUBJOBS_PER_JOB];

      if((sign != tar_sign) || (mag <= start_accel) || (tar_mag <= start_accel)) {
        f_start[drive_axis] = (tar_sign * min(tar_mag, start_accel));
      } else {
        f_start[drive_axis] = current_jobs[drive_axis].frequency;
      }

      bool do_accel = current_jobs[drive_axis].target_frequency != f_start[drive_axis];
      accel[drive_axis] = do_accel ? max_acceleration : 0.0;

      //next rescale the start frequencies and accelerations such that
      //the ratios between the speeds are always the same
      for(uint8_t i=0; i<SUBJOBS_PER_JOB; i++){
        if(i!=drive_axis && current_jobs[i].running) {
          if(do_accel) {
            float factor = current_jobs[i].target_frequency / current_jobs[drive_axis].target_frequency;
            f_start[i] = factor * f_start[drive_axis];
            accel[i] = factor * accel[drive_axis];
          } else {
            f_start[i] = current_jobs[i].target_frequency;
            accel[i] = 0.0;
          }
        }
      }

      //setup the acceleration timer

      if(do_accel) {

        const uint16_t accel_freq = (uint16_t) (1.0 / ACCEL_TIME_RESOLUTION);
        const Period<uint8_t> accel_period = period_from_frequency<uint8_t>(
          accel_freq, accel_timer.prescaling
        );

        *accel_timer.tcnt = 0;
        *accel_timer.tccrna = _BV(WGM21);
        *accel_timer.tccrnb = accel_period.prescale;
        *accel_timer.ocra = accel_period.period;

        Serial.print("a ");
        Serial.print(accel_freq);
        Serial.print(" ");
        Serial.print(accel_period.period);
        Serial.print(" ");
        Serial.print(*accel_timer.ocra);
        Serial.print(" ");
        Serial.println(*accel_timer.tccrnb,BIN);
      }

      //finally, actually set the timers
      for(uint8_t id=0; id<SUBJOBS_PER_JOB; id++){
        //clear the timer value
        *timers[id].tcnt = 0;

        //clear the config
        *timers[id].tccrna = 0;

        if(current_jobs[id].running) {

          current_jobs[id].frequency = f_start[id];

          current_jobs[id].acceleration = accel[id];
          current_jobs[id].accelerating = current_jobs[id].acceleration != 0;

          *timers[id].tccrnb = _BV(WGM12); //clear the timer when it reaches OCRnA

          if(f_start[id]!=0) {
            switch(id) {
              case 0:
                digitalWrite(DIR_FEED, f_start[id]<0 ^ FEED_INVERT_DIR ? HIGH : LOW);
                break;
              case 1:
                digitalWrite(DIR_CLAMP, f_start[id]<0 ^ CLAMP_INVERT_DIR ? HIGH : LOW);
                break;
              case 2:
                digitalWrite(DIR_DRIVE, f_start[id]<0 ^ DRIVE_INVERT_DIR ? HIGH : LOW);
                break;
            }
          }

          set_frequency(id, f_start[id]);

          Serial.print(id);
          Serial.print(" ");
          Serial.print(f_start[id]);
          Serial.print(" ");
          Serial.print(current_jobs[id].acceleration);
          Serial.print(" ");
          Serial.print(current_jobs[id].target_frequency);
          Serial.print(" ");
          Serial.print(*timers[id].ocra);
          Serial.print(" ");
          Serial.print(*timers[id].tccrnb,BIN);
          Serial.print(" ");
          Serial.println(current_jobs[id].remaining);
        } else {
          //disable the timer interrupt and clear the compare value
          *timers[id].tccrnb = 0;
          *timers[id].timsk = 0;
          *timers[id].ocra = 0;
          *timers[id].ocrb = 0;
        }
      }

      cli();
        if(do_accel) *accel_timer.timsk = (1<<1);
        for(uint8_t i=0; i<SUBJOBS_PER_JOB; i++) {
          if(current_jobs[i].running) {
            *timers[i].timsk = (1<<1);
            if(stepper_dedge(i)) *timers[i].timsk |= (1<<2);
          }
        }
      sei();

    } else {
      //if the last job is done but nothing is queued, then we must assume that
      //the steppers are stopping, so we need to set the current frequency to 0
      for(uint8_t i=0; i<SUBJOBS_PER_JOB; i++)
        current_jobs[i].frequency = 0.0;
    }
  }

  //run the callbacks for any jobs that have completed
  for(byte i=0; i<SUBJOBS_PER_JOB; i++) {
    if(!current_jobs[i].running && current_jobs[i].callback!=NULL) {
      (*current_jobs[i].callback)(current_jobs[i].callback_args);
      current_jobs[i].callback = NULL;
    }
  }

}

void machine_init() {

  SPI.begin();

  clamp.begin();
  clamp.rms_current(CLAMP_CURRENT);

  #ifdef CLAMP_DEDGE
    clamp.dedge(true);
  #else
    clamp.dedge(false);
  #endif

  clamp.microsteps(CLAMP_MS);
  clamp.TCOOLTHRS(400);
  clamp.sgt(CLAMP_SGT);
  clamp.diag0_stall(true);

  feed.begin();
  feed.rms_current(FEED_CURRENT);

  #ifdef FEED_DEDGE
    feed.dedge(true);
  #else
    feed.dedge(false);
  #endif

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
