
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <time.h>

#include <termios.h>
#include <unistd.h>

#include <sys/types.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <pthread.h>


#define BUF_PAGE_SIZE 64

#define XON ((char) 17)
#define XOFF ((char) 19)

pthread_mutex_t flow_lock = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t flow_ready = PTHREAD_COND_INITIALIZER;
bool flow_on = true;

pthread_mutex_t cmd_lock = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t cmd_ready = PTHREAD_COND_INITIALIZER;
bool cmd_on = false;

pthread_mutex_t buf_lock = PTHREAD_MUTEX_INITIALIZER;

void lock_set(pthread_mutex_t* lock, bool* obj, bool val) {
  pthread_mutex_lock(lock);
  *obj = val;
  pthread_mutex_unlock(lock);
}

void wait_until(pthread_cond_t* cond, pthread_mutex_t* lock, bool* value) {
  pthread_mutex_lock(lock);
  while(!*value) pthread_cond_wait(cond, lock);
  pthread_mutex_unlock(lock);
}

void notify(pthread_cond_t* cond, pthread_mutex_t* lock, bool* value) {
  pthread_mutex_lock(lock);
  *value = true;
  pthread_cond_broadcast(cond);
  pthread_mutex_unlock(lock);
}

struct buf_list {
  char buf[BUF_PAGE_SIZE];
  size_t size;
  struct buf_list* next;
};
struct buf_list* cmd_buffer_back;
struct buf_list* cmd_buffer_front;

struct buf_list* init_page() {
  struct buf_list* page = malloc(sizeof(struct buf_list));
  page->next = NULL;
  page->size = 0;
  return page;
}

void *cmd_thread(void* arg) {

  //read from stdin until we get a new-line. Then, flush the buffer
  while(1) {

    //read the next character
    char x = fgetc(stdin);

    pthread_mutex_lock(&buf_lock);

    //add the next character to the buffer
    cmd_buffer_front->buf[cmd_buffer_front->size++] = x;

    //check if we've finished the line
    if(x=='\n' || cmd_buffer_front->size>=BUF_PAGE_SIZE) {

      //push the finished command onto the list
      cmd_buffer_front->next = init_page();
      cmd_buffer_front = cmd_buffer_front->next;

      //notify the write thread that there's more stuff
      notify(&cmd_ready, &cmd_lock, &cmd_on);
    }

    pthread_mutex_unlock(&buf_lock);

  }


}

void *write_thread(void* arg) {
  //get the file descriptor
  int dev = *(int*) arg;

  //setup the buffer
  pthread_mutex_lock(&buf_lock);

  cmd_buffer_back = init_page();
  cmd_buffer_front = cmd_buffer_back;

  pthread_mutex_unlock(&buf_lock);

  pthread_t t;
  pthread_create(&t, NULL, cmd_thread, NULL);

  while(1) {

    //wait until XON is sent
    wait_until(&flow_ready, &flow_lock, &flow_on);

    pthread_mutex_lock(&buf_lock);

    //check if there's data to send
    //we check next since there last element in the list is the read-buffer
    if(cmd_buffer_back->next==NULL){
      pthread_mutex_unlock(&buf_lock);

      //wait for there to be data to write
      lock_set(&cmd_lock, &cmd_on, false);
      wait_until(&cmd_ready, &cmd_lock, &cmd_on);
    } else {
      //write the line to the device
      // printf("Writing!\n");
      write(dev, cmd_buffer_back->buf, cmd_buffer_back->size);

      //pop the line off of the buffer
      struct buf_list* next = cmd_buffer_back->next;
      free(cmd_buffer_back);
      cmd_buffer_back = next;

      pthread_mutex_unlock(&buf_lock);

      // usleep(1000);
    }


  }

}

void *read_thread(void* arg) {

  //get the file-descriptor
  int dev = *(int*) arg;

  //read from stdin until we get a new-line. Then, flush the buffer
  while(1) {
    //read the next char from the device
    char x;
    size_t count = read(dev, &x, 1);

    if(count) {
      if(x==XON) {
        notify(&flow_ready, &flow_lock, &flow_on);
      } else if(x==XOFF) {
        lock_set(&flow_lock, &flow_on, false);
      } else {
        fputc(x, stdout);
      }
    }




  }
}

int main(int argc, char const *argv[]) {

  //make sure we got a path
  if(argc<=1) {
    printf("Please provide a serial port!\n");
    return 1;
  }

  //open the file
  int device = open(argv[1], O_RDWR | O_DSYNC | O_SYNC);
  if(device == -1){
    printf("Error opening device!\n");
    return 1;
  }

  //configure the serial port

  struct termios config;

  if(tcgetattr(device, &config)) {
    printf("Error getting serial port config!\n");
    return 1;
  }

  config.c_iflag &= ~(BRKINT | ICRNL | IMAXBEL | INLCR | IUTF8 | IXANY | IXON | IXOFF);
  config.c_iflag |= IGNBRK;

  config.c_oflag &= ~(OCRNL | ONLCR | NLDLY);
  config.c_oflag |= NL0 | OPOST;

  config.c_cflag &= ~(CRTSCTS | CSTOPB | PARENB | CSIZE | CBAUD);
  config.c_cflag |= CS8 | B115200;

  config.c_lflag &= ~(ECHO | ECHOE | ECHOK | ECHONL | ICANON | IEXTEN);
  config.c_lflag |= NOFLSH | ISIG;

  config.c_cc[VEOL] = '\n';
  config.c_cc[VSTART] = XON;
  config.c_cc[VSTOP] = XOFF;

  if(tcsetattr(device, TCSANOW, &config)) {
    printf("Error configuring serial port!\n");
    return 1;
  }

  pthread_t t1,t2;

  if(pthread_create(&t1, NULL, read_thread, &device) || pthread_create(&t2, NULL, write_thread, &device)){
    printf("Thread creation error!\n");
    return 1;
  }


  pthread_join(t1, NULL);
  pthread_join(t2, NULL);

  return 0;
}
