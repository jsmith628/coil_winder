
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

//controls if output is on or off given the flow control system
pthread_mutex_t flow_lock = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t flow_ready = PTHREAD_COND_INITIALIZER;
bool flow_on = true;

//notifies the threads that there is a new command to send
pthread_mutex_t cmd_lock = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t cmd_ready = PTHREAD_COND_INITIALIZER;
bool cmd_on = false;

//
//Helper methods for thread-stuff
//

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


//the mutex synchronizing buffer access
pthread_mutex_t buf_lock = PTHREAD_MUTEX_INITIALIZER;

//the buffer of commands
struct buf_list {
  char buf[BUF_PAGE_SIZE];
  size_t size;
  bool control;
  struct buf_list* next;
};
struct buf_list* cmd_buffer_back;
struct buf_list* cmd_buffer_front;

//mallocs up a new buffer for the next command line
struct buf_list* init_page() {
  struct buf_list* page = malloc(sizeof(struct buf_list));
  page->next = NULL;
  page->size = 0;
  page->control = false;
  return page;
}

bool xflow = true; //setting for if XON/XOFF works
bool echo = false; //setting for if things are spat back at stdout

//the setting for if an EOT from the device closes the application
//however, this needs to be able to change since ^D overrides this behavior
pthread_mutex_t eot_lock = PTHREAD_MUTEX_INITIALIZER;
bool end_on_eot = false;

//the fd of the device
int device;

//specifies if the application should stop
pthread_mutex_t stop_lock = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t stop_ready = PTHREAD_COND_INITIALIZER;
bool stop = false;

void *cmd_thread(void* arg) {

  //read from stdin until we get a new-line. Then, flush the buffer
  while(1) {

    //read the next character
    char x;
    size_t count = fread(&x, 1, 1, stdin);

    //if stdin was closed, set x to EOT
    if(count==0) x = EOT;

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
    } else if(x==CAN || x==EOT || x=='\n' || cmd_buffer_front->size>=BUF_PAGE_SIZE-1) {//check if we've finished the line

      //send control characters immediately
      if(x==EOT || x==CAN) cmd_buffer_front->control = true;

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

      //if we're sending an EOT, we need to be able to exit when we get a response
      if(cmd_buffer_back->control) lock_set(&eot_lock, &end_on_eot, true);

      //write the line to the device
      write(dev, cmd_buffer_back->buf, cmd_buffer_back->size);
      if(echo && !cmd_buffer_back->control) fwrite(cmd_buffer_back->buf, 1, cmd_buffer_back->size, stdout);


      //pop the line off of the buffer
      struct buf_list* next = cmd_buffer_back->next;
      free(cmd_buffer_back);
      cmd_buffer_back = next;

      pthread_mutex_unlock(&buf_lock);

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

    if(count) { //make sure the port is still open
      pthread_mutex_lock(&eot_lock);
      if(xflow && x==XON) { //unblock the writing thread if we get an XON
        notify(&flow_ready, &flow_lock, &flow_on);
      } else if(xflow && x==XOFF) { //block the writing thread if we get an XOFF
        lock_set(&flow_lock, &flow_on, false);
      } else if((end_on_eot && x==EOT) || x==INTERRUPT || x==QUIT) { //stop on an EOF, ^D, M20, or ^C
        notify(&stop_ready, &stop_lock, &stop);
      } else { //else, just parrot to stdout
        fputc(x, stdout);
      }
      pthread_mutex_unlock(&eot_lock);
    } else { //else, exit
      fprintf(stderr, "Serial port closed. Exiting\n");
      notify(&stop_ready, &stop_lock, &stop);
      break;
    }

  }
}

//configure the serial port
int init_device(int device, bool conv_lc) {

  //get the current config
  struct termios config;
  if(tcgetattr(device, &config)) {
    printf("Error getting serial port config!\n");
    return 1;
  }

  //don't convert anything and don't do XON/XOFF in the driver (since it doesn't work :( )
  config.c_iflag &= ~(IGNBRK | ICRNL | IMAXBEL | INLCR | IUTF8 | IXANY | IUCLC | IXOFF | IXON);
  config.c_iflag |= BRKINT;

  //don't convert any characters
  config.c_oflag &= ~(OCRNL | ONLCR | NLDLY | OLCUC);
  config.c_oflag |= NL0 | OPOST;

  //convert lower-case console input if desired
  if(conv_lc) config.c_oflag |= OLCUC;

  //115200 baud, 1 stop-bit, no-parity
  config.c_cflag &= ~(CSTOPB | PARENB | CSIZE | CBAUD);
  config.c_cflag |= CRTSCTS | CS8;

  switch(COM_BAUD) {
    case 0:      config.c_cflag |= B0;     break;
    case 50:     config.c_cflag |= B50;    break;
    case 75:     config.c_cflag |= B75;    break;
    case 110:    config.c_cflag |= B110;   break;
    case 134:    config.c_cflag |= B134;   break;
    case 150:    config.c_cflag |= B150;   break;
    case 200:    config.c_cflag |= B200;   break;
    case 300:    config.c_cflag |= B300;   break;
    case 600:    config.c_cflag |= B600;   break;
    case 1200:   config.c_cflag |= B1200;  break;
    case 1800:   config.c_cflag |= B1800;  break;
    case 2400:   config.c_cflag |= B2400;  break;
    case 4800:   config.c_cflag |= B4800;  break;
    case 9600:   config.c_cflag |= B9600;  break;
    case 19200:  config.c_cflag |= B19200;  break;
    case 38400:  config.c_cflag |= B38400;  break;
    case 57600:  config.c_cflag |= B57600;  break;
    case 115200: config.c_cflag |= B115200; break;
    case 230400: config.c_cflag |= B230400; break;
  }

  //don't echo anything since the output _from_ the device is considered the "input"
  //so echoing would cause all of the loging information to be fed right back to
  //the device as a command
  //also, no flushing, flushing bad
  config.c_lflag &= ~(ECHO | ECHOCTL | ECHOE | ECHOK | ECHONL | ICANON | FLUSHO | NOFLSH | TOSTOP | ISIG | IEXTEN);
  config.c_lflag |= NOFLSH;

  //this isn't really used since it doesn't seem to work :/
  config.c_cc[VSTART] = XON;
  config.c_cc[VSTOP] = XOFF;

  //no read timeout
  config.c_cc[VTIME] = 0;

  //turn on poll-based reads for the startup handshake
  config.c_cc[VMIN] = 0;
  if(tcsetattr(device, TCSANOW, &config)) {
    printf("Error configuring serial port!\n");
    return 1;
  }

  char x;

  //delay until we get a message or time-out to make sure we've properly connected
  for(int i=0, x=0; i<5000 && !read(device, &x, 1); i++) usleep(1000);

  //now, send an enquiry to make sure the device is properly setup
  x = ENQ;
  write(device, &x, 1);
  for(int i=0, x=0; i<5000 && !read(device, &x, 1); i++) usleep(1000);

  //turn on blocking-based reads
  config.c_cc[VMIN] = 1;
  if(tcsetattr(device, TCSANOW, &config)) {
    printf("Error configuring serial port!\n");
    return 1;
  }

}

int print_help_info(FILE * stream) {
  fprintf(stream,
    "\n"
    "Usage: console DEVICE [-e|--echo|-E|--noecho] [-x|--xflow|-X|--noxflow] [-d|--eot|-D|--noeot] [-u|--upper|-U|--noupper]\n"
    "   or: console --help\n"
    "\n"
    "Options:\n"
    "    DEVICE:\n"
    "        the serial port of the machine to open\n"
    "    -e, --echo\n"
    "        when on, all lines written to device are also written to stdout (off by default)\n"
    "    -E, --noecho:\n"
    "        turns off --echo\n"
    "    -x, --xflow:\n"
    "        turns on XON/XOFF flow control (on by default)\n"
    "    -X, --noxflow:\n"
    "        turns off --xflow\n"
    "    -d, --eot:\n"
    "        when on, the console will close when an ASCII value 0x04 (EOT) is read from the device \n"
    "    -D, --noeot:\n"
    "        turns off --eot\n"
    "    -u, --upper:\n"
    "        when on, lower-case input will automatically be converted to uppercase \n"
    "    -U, --noupper:\n"
    "        turns off --upper\n"
    "    --help:\n"
    "        displays this message\n"
    "\n"
  );
  return 0;
}

//captures the SIGINT signal and passes it onto the device
void signal_handler(int sig) {

  char msg = '\0';
  switch(sig){
    case SIGQUIT: msg = QUIT; break;
    case SIGINT: msg = INTERRUPT; break;
    default: notify(&stop_ready, &stop_lock, &stop);
  }

  write(device, &msg, 1);
}

int main(int argc, char const *argv[]) {

  //parse the command line options
  char const * device_name = NULL;
  bool conv_lc = false;
  for(int i=1; i<argc; i++) {
    if(!strcmp(argv[i],"--echo")) echo=true;
    else if(!strcmp(argv[i],"--noecho")) echo=false;
    else if(!strcmp(argv[i],"--xflow")) xflow=true;
    else if(!strcmp(argv[i],"--noxflow")) xflow=false;
    else if(!strcmp(argv[i],"--eot")) end_on_eot=true;
    else if(!strcmp(argv[i],"--noeot")) end_on_eot=false;
    else if(!strcmp(argv[i],"--upper")) conv_lc=true;
    else if(!strcmp(argv[i],"--noupper")) conv_lc=false;
    else if(!strcmp(argv[i],"--help")) return print_help_info(stdout);
    else if(argv[i][0] == '-' && argv[i][1] != '-' && argv[i][1] != '\0') {
      for(char const * x = &argv[i][1]; *x!='\0'; x++) {
        if(*x=='e') echo=true;
        else if(*x=='E') echo=false;
        else if(*x=='x') xflow=true;
        else if(*x=='X') xflow=false;
        else if(*x=='d') end_on_eot=true;
        else if(*x=='D') end_on_eot=false;
        else if(*x=='u') conv_lc=true;
        else if(*x=='U') conv_lc=false;
        else {
          fprintf(stderr, "Invalid option %s\n", argv[i]);
          fprintf(stderr, "Use --help for more information\n");
          return 1;
        }
      }
    } else if(device_name==NULL) {
      device_name = argv[i];
    } else {
      fprintf(stderr, "Invalid option \"%s\"\n", argv[i]);
      fprintf(stderr, "Use --help for more information\n");
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

  //initialize the termios settings a wait for the device to startup
  int err = init_device(device, conv_lc);
  if(err) return err;

  //register a function to handle system signals
  signal(SIGINT, signal_handler);
  signal(SIGQUIT, signal_handler);

  //started
  pthread_t t1,t2;
  if(pthread_create(&t1, NULL, read_thread, &device) || pthread_create(&t2, NULL, write_thread, &device)){
    fprintf(stderr, "Thread creation error!\n");
    return 1;
  }

  wait_until(&stop_ready, &stop_lock, &stop);

  return 0;
}
