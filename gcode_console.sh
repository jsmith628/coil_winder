#!/bin/sh

#check to make sure a serial device was actually specified
if [ -z "$1" ]; then
    echo 'Please provide a serial port!'
else

    #set the tty settings
    stty -F $1 \
        cs8 115200 \
        ignbrk -brkint -icrnl -imaxbel -opost -onlcr -isig -icanon -iexten\
        -echo -echoe -echok -echoctl -echoke \
        noflsh -ixon -crtscts

    #output
    cat $1 &
    OUT_PID=$!


    #kill the output cat when the user does ^C
    trap "kill $OUT_PID" SIGINT

    #input
    cat > $1


fi
