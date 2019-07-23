

#ifndef _QUEUE_H_
#define _QUEUE_H_

template<class T, byte P>
class Queue {

  private:
    T data[1 << P];
    unsigned int top, bottom;

    inline unsigned int mask();

  public:

    Queue();

    inline unsigned int capacity();
    unsigned int count();

    bool push_top(T val);
    bool push_bottom(T val);

    T peek_top();
    T peek_bottom();

    T pop_top();
    T pop_bottom();

};

template<class T, byte P>
Queue<T,P>::Queue() {
  top = 0;
  bottom = 0;
}

template<class T, byte P> inline unsigned int Queue<T,P>::capacity() { return 1 << P; }
template<class T, byte P> inline unsigned int Queue<T,P>::mask() { return (1 << P) - 1; }

template<class T, byte P>
unsigned int Queue<T,P>::count() {
  if(top > bottom) {
    return top - bottom;
  } else {
    return capacity() - (bottom - top);
  }
}

template<class T, byte P>
bool Queue<T,P>::push_top(T val) {
  unsigned int new_top = (top+1) & mask();
  if(new_top==bottom){
    return false;
  } else {
    data[top] = val;
    top = new_top;
    return true;
  }
}

template<class T, byte P>
bool Queue<T,P>::push_bottom(T val) {
  unsigned int new_bottom = (bottom-1) & mask();
  if(top==new_bottom){
    return false;
  } else {
    data[bottom] = val;
    bottom = new_bottom;
    return true;
  }
}

template<class T, byte P> T Queue<T,P>::peek_top() { return data[top]; }
template<class T, byte P> T Queue<T,P>::peek_bottom() { return data[bottom]; }

template<class T, byte P>
T Queue<T,P>::pop_top() {
  T val = peek_top();
  if(top!=bottom){
    top = (top-1) & mask();
  }
  return val;
}

template<class T, byte P>
T Queue<T,P>::pop_bottom() {
  T val = peek_bottom();
  if(top!=bottom){
    bottom = (bottom+1) & mask();
  }
  return val;
}


#endif
