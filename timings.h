
#ifndef _TIMINGS_H_
#define _TIMINGS_H_

int gcd(int n, int m);
int lcm(int n, int m);

template<int N>
class SyncEvents {
  private:
    int timings[N];
    int current[N];

  public:
    SyncEvents();
    SyncEvents(int ratio[N]);
    void step(bool * results);
};


template<int N>
SyncEvents<N>::SyncEvents() {

  for(int i=0; i<N; i++) {
    this->current[i] = 0;
    this->timings[i] = 0;
  }

}

template<int N>
SyncEvents<N>::SyncEvents(int ratio[N]) {

  int total_cycles = ratio[0];
  for(int i=1; i<N; i++) total_cycles = lcm(total_cycles,ratio[i]);

  for(int i=0; i<N; i++) {
    this->current[i] = 0;
    this->timings[i] = total_cycles / ratio[i];
  }

}

template<int N>
void SyncEvents<N>::step(bool * results) {
  for(int i=0; i<N; i++) this->current[i]++;

  for(int i=0; i<N; i++, results++) {
    *results = this->current[i] >= this->timings[i];
    if(*results) this->current[i] = 0;
  }
}

#endif
