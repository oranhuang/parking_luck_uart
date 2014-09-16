CC:=arm-linux-gnueabihf-gcc
#CC:=gcc

all: uart

uart.o : uart.c
	$(CC) -c uart.c 

uart : uart.o
	$(CC) -Wall -o2 -o uart uart.o -lm -lpthread


clean:
	-rm -f *.o
	-rm -f uart
