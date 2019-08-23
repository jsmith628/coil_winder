
#ifndef _ASCII_CONTROL_H_
#define _ASCII_CONTROL_H_

#define BUF_PAGE_SIZE 64

#define ETX ((char) 0x03)
#define EOT ((char) 0x04)
#define ENQ ((char) 0x05)
#define ACK ((char) 0x06)
#define BEL ((char) 0x07)
#define BS ((char) 0x08)
#define LF ((char) 0x0A)
#define DC1 ((char) 0x11)
#define DC2 ((char) 0x12)
#define DC3 ((char) 0x13)
#define DC4 ((char) 0x14)
#define NAK ((char) 0x15)
#define CAN ((char) 0x18)
#define SUB ((char) 0x1A)
#define FS ((char) 0x1C)
#define DEL ((char) 0x7F)

#define XON DC1
#define PAUSE DC2
#define XOFF DC3
#define RESUME DC4

#define INTERRUPT ETX
#define QUIT FS

#endif
