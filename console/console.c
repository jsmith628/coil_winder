
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include <termios.h>
#include <unistd.h>
#include <signal.h>
#include <fcntl.h>

#include <pthread.h>

#include "../firmware/ascii_control.h"

//controls if output is on or off given the flow control system
pthread_mutex_t flow_lock = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t flow_ready = PTHREAD_COND_INITIALIZER;
bool flow_on = true;

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

bool xflow = true; //setting for if XON/XOFF works
bool echo = false; //setting for if things are spat back at stdout
bool allow_stop = false;

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

void *write_thread(void* arg) {
  //get the file descriptor
  int dev = *(int*) arg;

  //loop until stdin closes
  for(char x=0; x!=EOT;) {

    //read from stdin
    if(fread(&x, 1, 1, stdin)==0) x = EOT;

    //wait until XON is sent
    wait_until(&flow_ready, &flow_lock, &flow_on);

    //write to device
    write(dev, &x, 1);
    if(echo && x!=EOT) fwrite(&x, 1, 1, stdout);

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
      } else if(x==PAUSE && allow_stop) {
        raise(SIGSTOP);
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

  config.c_cflag &= ~(CSTOPB | CSIZE | CBAUD);
  // config.c_cflag |= CRTSCTS;

  if(COM_TWO_STOPB) config.c_cflag |= CSTOPB;

  switch(COM_PARITY&3) {
    case 0: config.c_cflag &= ~(PARENB); break;
    case 1: config.c_cflag |= PARENB | PARODD; break;
    case 2: config.c_cflag |= PARENB; config.c_cflag &= ~(PARODD); break;
  }

  switch(COM_DATA_BITS) {
    case 5: config.c_cflag |= CS5; break;
    case 6: config.c_cflag |= CS6; break;
    case 7: config.c_cflag |= CS7; break;
    case 8: default: config.c_cflag |= CS8; break;
  }

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
    "Usage: console DEVICE [-e|--echo|-E|--noecho] [-x|--xflow|-X|--noxflow] "
                          "[-d|--eot|-D|--noeot] [-u|--upper|-U|--noupper] "
                          "[-s|--stop|-S|--nostop]\n"
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
    "        when on, the console will close when an ASCII value 0x04 (EOT) is read from the device (off by default) \n"
    "    -D, --noeot:\n"
    "        turns off --eot\n"
    "    -u, --upper:\n"
    "        when on, lower-case input will automatically be converted to uppercase (off by default) \n"
    "    -U, --noupper:\n"
    "        turns off --upper\n"
    "    -s, --stop:\n"
    "        when a TSTP signal is received, the console will pause the device and suspend\n"
    "    -S, --nostop:\n"
    "        when a TSTP signal is received, the console will toggle the pause-state of the device\n"
    "    --help:\n"
    "        displays this message\n"
    "\n"
  );
  return 0;
}

//captures the SIGINT signal and passes it onto the device
void signal_handler(int sig) {

  static bool paused = false;

  char msg = '\0';
  switch(sig){
    case SIGQUIT: msg = QUIT; break;
    case SIGINT: msg = INTERRUPT; break;
    case SIGTSTP:
      msg = !paused||allow_stop ? PAUSE : RESUME;
      paused = msg==PAUSE;
      break;
    case SIGCONT:
      msg = RESUME;
      paused = false;
      break;
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
    else if(!strcmp(argv[i],"--stop")) allow_stop=true;
    else if(!strcmp(argv[i],"--nostop")) allow_stop=false;
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
        else if(*x=='s') allow_stop=true;
        else if(*x=='S') allow_stop=false;
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
  signal(SIGTSTP, signal_handler);
  signal(SIGCONT, signal_handler);

  //started
  pthread_t t1,t2;
  if(pthread_create(&t1, NULL, read_thread, &device) || pthread_create(&t2, NULL, write_thread, &device)){
    fprintf(stderr, "Thread creation error!\n");
    return 1;
  }

  wait_until(&stop_ready, &stop_lock, &stop);

  return 0;
}
