

#ifndef _QUEUE_H_
#define _QUEUE_H_

template<class T, byte P>
class Queue {

  private:
    T data[1 << P];
    unsigned int top, bottom, size;

    inline unsigned int mask();

  public:

    Queue();

    inline unsigned int capacity();
    unsigned int count();
    unsigned int available();

    bool push_top(T val);
    bool push_bottom(T val);

    T peek_top();
    T peek_bottom();

    T pop_top();
    T pop_bottom();

    void clear();

};

template<class T, byte P> Queue<T,P>::Queue() { clear(); }

template<class T, byte P>
void Queue<T,P>::clear() {
  top = 0;
  bottom = 0;
  size = 0;
}

template<class T, byte P> inline unsigned int Queue<T,P>::capacity() { return (1 << P); }
template<class T, byte P> inline unsigned int Queue<T,P>::mask() { return (1 << P) - 1; }

template<class T, byte P> unsigned int Queue<T,P>::count() { return size; }
template<class T, byte P> unsigned int Queue<T,P>::available() { return capacity()-count(); }

template<class T, byte P>
bool Queue<T,P>::push_top(T val) {
  if(size==capacity()){
    return false;
  } else {
    data[top] = val;
    top = (top+1)&mask;
    size++;
    return true;
  }
}

template<class T, byte P>
bool Queue<T,P>::push_bottom(T val) {
  if(size==capacity()){
    return false;
  } else {
    data[bottom] = val;
    bottom = (bottom-1) & mask();
    size++;
    return true;
  }
}

template<class T, byte P> T Queue<T,P>::peek_top() { return data[top]; }
template<class T, byte P> T Queue<T,P>::peek_bottom() { return data[bottom]; }

template<class T, byte P>
T Queue<T,P>::pop_top() {
  T val = peek_top();
  if(size!=0){
    top = (top-1) & mask();
    size--;
  }
  return val;
}

template<class T, byte P>
T Queue<T,P>::pop_bottom() {
  T val = peek_bottom();
  if(size!=0){
    bottom = (bottom+1) & mask();
    size--;
  }
  return val;
}


#endif
