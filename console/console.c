
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <time.h>

#include <termios.h>
#include <unistd.h>
#include <signal.h>

#include <sys/types.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <pthread.h>

#include "../firmware/ascii_control.h"

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
  bool control;
  struct buf_list* next;
};
struct buf_list* cmd_buffer_back;
struct buf_list* cmd_buffer_front;

struct buf_list* init_page() {
  struct buf_list* page = malloc(sizeof(struct buf_list));
  page->next = NULL;
  page->size = 0;
  page->control = false;
  return page;
}

bool xflow = true;
bool echo = false;

pthread_mutex_t eof_lock = PTHREAD_MUTEX_INITIALIZER;
bool end_on_eof = false;


int device;


pthread_mutex_t stop_lock = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t stop_ready = PTHREAD_COND_INITIALIZER;
bool stop = false;

void *cmd_thread(void* arg) {

  //read from stdin until we get a new-line. Then, flush the buffer
  while(1) {

    //read the next character
    char x;
    size_t count = fread(&x, 1, 1, stdin);
    if(count==0 || x==EOT){
      x = EOT;
      lock_set(&eof_lock, &end_on_eof, true);
    }

    pthread_mutex_lock(&buf_lock);

    //add the next character to the buffer
    cmd_buffer_front->buf[cmd_buffer_front->size++] = x;

    if(x==DEL || x==BS){//do the backspace!
      cmd_buffer_front->size--;
      if(cmd_buffer_front->size==0) {
        printf("\a");
      } else {
        cmd_buffer_front->size--;
      }
    } else if(x<0x20&&x>=0 || cmd_buffer_front->size>=BUF_PAGE_SIZE-1) {//check if we've finished the line

      //send control characters immediately
      if(x==EOT && x!='\n'){
        cmd_buffer_front->buf[cmd_buffer_front->size++] = '\n';
        cmd_buffer_front->control = true;
      }

      //push the finished command onto the list
      cmd_buffer_front->next = init_page();
      cmd_buffer_front = cmd_buffer_front->next;

      //notify the write thread that there's more stuff
      notify(&cmd_ready, &cmd_lock, &cmd_on);
    }

    pthread_mutex_unlock(&buf_lock);

    //if stdin is closed, then we don't want to keep reading it
    if(x==EOT) break;

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

  //create the thread that reads stdin and constructs the buffer
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
      write(dev, cmd_buffer_back->buf, cmd_buffer_back->size);
      if(echo && !cmd_buffer_back->control) fwrite(cmd_buffer_back->buf, 1, cmd_buffer_back->size, stdout);

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

    if(count) { //make sure no error happened
      pthread_mutex_lock(&eof_lock);
      if(xflow && x==XON) { //unblock the writing thread if we get an XON
        notify(&flow_ready, &flow_lock, &flow_on);
      } else if(xflow && x==XOFF) { //block the writing thread if we get an XOFF
        lock_set(&flow_lock, &flow_on, false);
      } else if((end_on_eof && x==EOT) || x==ETX) {
        notify(&stop_ready, &stop_lock, &stop);
      } else { //else, just parrot to stdout
        fputc(x, stdout);
      }
      pthread_mutex_unlock(&eof_lock);
    }

  }
}

int init_device(int device) {
  //configure the serial port

  struct termios config;

  if(tcgetattr(device, &config)) {
    printf("Error getting serial port config!\n");
    return 1;
  }

  config.c_iflag &= ~(IGNBRK | ICRNL | IMAXBEL | INLCR | IUTF8 | IXANY | IUCLC | IXOFF | IXON);
  config.c_iflag |= BRKINT;

  config.c_oflag &= ~(OCRNL | ONLCR | NLDLY);
  config.c_oflag |= NL0 | OPOST | OLCUC;

  config.c_cflag &= ~(CSTOPB | PARENB | CSIZE | CBAUD);
  config.c_cflag |= CRTSCTS | CS8 | B115200;

  config.c_lflag &= ~(ECHO | ECHOCTL | ECHOE | ECHOK | ECHONL | ICANON | FLUSHO | NOFLSH | TOSTOP | ISIG | IEXTEN);
  config.c_lflag |= NOFLSH;

  // config.c_cc[VEOL] = '\n';
  config.c_cc[VSTART] = XON;
  config.c_cc[VSTOP] = XOFF;
  config.c_cc[VTIME] = 0;

  config.c_cc[VMIN] = 1;
  if(tcsetattr(device, TCSANOW, &config)) {
    printf("Error configuring serial port!\n");
    return 1;
  }



}

void signal_handler(int sig) {

  char msg[2] = "\0\n";
  switch(sig){
    case SIGINT: msg[0] = ETX; break;
    default: notify(&stop_ready, &stop_lock, &stop);
  }

  write(device, &msg, 2);
}

int main(int argc, char const *argv[]) {

  //parse the command line options
  char const * device_name = NULL;
  for(int i=1; i<argc; i++) {
    if(!strcmp(argv[i],"--echo")) echo=true;
    else if(!strcmp(argv[i],"--noecho")) echo=false;
    else if(!strcmp(argv[i],"--xflow")) xflow=true;
    else if(!strcmp(argv[i],"--noxflow")) xflow=false;
    else if(!strcmp(argv[i],"--eof")) end_on_eof=true;
    else if(!strcmp(argv[i],"--eof")) end_on_eof=false;
    else if(argv[i][0] == '-' && argv[i][1] != '-' && argv[i][1] != '\0') {
      for(char const * x = &argv[i][1]; *x!='\0'; x++) {
        if(*x=='e') echo=true;
        else if(*x=='E') echo=false;
        else if(*x=='x') xflow=true;
        else if(*x=='X') xflow=false;
        else if(*x=='d') end_on_eof=true;
        else if(*x=='D') end_on_eof=false;
        else {
          fprintf(stderr, "Invalid option \'%s\'\n", argv[i]);
          return 1;
        }
      }
    } else if(device_name==NULL) {
      device_name = argv[i];
    } else {
      fprintf(stderr, "Invalid option \"%s\"\n", argv[i]);
      return 1;
    }
  }

  //make sure we got a path
  if(device_name==NULL) {
    fprintf(stderr, "Please provide a serial port!\n");
    return 1;
  }

  //open the file
  device = open(device_name, O_RDWR | O_DSYNC | O_SYNC);
  if(device == -1){
    fprintf(stderr, "Error opening device!\n");
    return 1;
  }

  int err = init_device(device);
  if(err) return err;

  usleep(1000000);

  signal(SIGINT, signal_handler);

  pthread_t t1,t2;

  if(pthread_create(&t1, NULL, read_thread, &device) || pthread_create(&t2, NULL, write_thread, &device)){
    fprintf(stderr, "Thread creation error!\n");
    return 1;
  }

  wait_until(&stop_ready, &stop_lock, &stop);

  return 0;
}
