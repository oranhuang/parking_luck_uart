#include <stdio.h>
#include <unistd.h>		//Used for UART
#include <fcntl.h>		//Used for UART
#include <termios.h>		//Used for UART
#include <signal.h>
#include <errno.h>
#include <stdlib.h>
#include <pthread.h>

//char *str = "curl -H "Content-Type: application/json" -d '{"state":3}' http://parkingluck.cloudas.info/pl1/rest/parkings/012?token=MTEzMjU5OTkwOUNBQUNFZEVvc2UwY0JBSURaQW91UXpSWXVIek9zN2ZPeW9JczVVQVpDTk96UnJSSkI1cVc0Z1pBY1lCWkJSczVaQVpBUEtuYWZUUkhueXpwWFVLR1VaQ3VTSHlIbmVud09YcXk0WkIwWkI0eUptbFJoSlU1b1htODkzWGNMOG5IWTgwWEpSM0F6UlpCRFpBWkMxa1pBRGl3cVpBQ3lyWkE5ZnJBVEg4SHNqM1dUTkJKcGJJMzlDUzQ1c1hEeVdTb3FHUPARKINGLUCKLOVEPARKING";
//char *str = {"ad"b"};

volatile sig_atomic_t get_signal_int = 0;
int fd;
void get_sigint(int a)
{
	printf("^C caught\n");
	get_signal_int = 1;
}

void *thread_rev(){
	unsigned char rx_buffer[255];
	unsigned char rx_data;
	int rx_length, i=0, j=0;
	int sum, end_cnt, data_acquire;
	int current_st_is_ocupied = 0;

	printf("start receiving data....\n");

	while(!get_signal_int)
	{
		//sleep(3);
		// Read up to 255 characters from the port if they are there
		rx_length = read(fd, (void*)rx_buffer, 255);		//Filestream, buffer to store in, number of bytes to read (max)
		//rx_length = read(fd, &rx_data, 1);		//Filestream, buffer to store in, number of bytes to read (max)
		if (rx_length < 0)
		{
			//An error occured (will occur if there are no bytes)
			printf("error occured....%s\n", strerror(errno));
		}
		else if (rx_length == 0)
		{
			//No data waiting
			//fcntl(uart0_filestream, F_SETFL, 0);
			//printf("wait for data....\n");
			//printf("error occured....%s\n", strerror(errno));
		}
		else  
		{
			//Bytes received
			//rx_buffer[rx_length] = '\0';
			printf("%s\n", rx_buffer);				
			//printf("data=%d, rx len=%d\n", (int)(rx_buffer[0]*256) + (int)(rx_buffer[1]), rx_length);
			for(i=0;i<255;i++)
				rx_buffer[i] = 0;
			#if 0
			rx_buffer[i] = rx_data;
			i++;
			/*
			switch (rx_data){
				case 0xFF :
					end_cnt ++;
					if(end_cnt > 2)
						end_cnt = 1;
					break;
				case 0xA0 :
					if(end_cnt == 2)
						end_cnt++;
					if(end_cnt == 4){
						data_acquire = 1;
						end_cnt = 0;
					}
					break;
				default :				
				break; 
			}
			*/
			if(rx_data == 0x0a){
				printf("%s", rx_buffer);
				//for(j=0;j<2;j++)
				data_acquire = 0;
				//sum = (int)(rx_buffer[i-6]*256) + (int)(rx_buffer[i-5]);
				//printf("the abs value = %d\n",sum);
				//printf("magnetic abs value = %4.4fGa\n", (float)sum*0.92);
				i=0;
			#if 1
				if(sum > 5000 && !current_st_is_ocupied){
					current_st_is_ocupied = 1;
					//system("curl -H \"Content-Type: application/json\" -d '{\"state\":3}' http://parkingluck.cloudas.info/pl1/rest/parkings/012?token=MTEzMjU5OTkwOUNBQUNFZEVvc2UwY0JBSURaQW91UXpSWXVIek9zN2ZPeW9JczVVQVpDTk96UnJSSkI1cVc0Z1pBY1lCWkJSczVaQVpBUEtuYWZUUkhueXpwWFVLR1VaQ3VTSHlIbmVud09YcXk0WkIwWkI0eUptbFJoSlU1b1htODkzWGNMOG5IWTgwWEpSM0F6UlpCRFpBWkMxa1pBRGl3cVpBQ3lyWkE5ZnJBVEg4SHNqM1dUTkJKcGJJMzlDUzQ1c1hEeVdTb3FHUPARKINGLUCKLOVEPARKING");
					printf("send the state=3 to server\n");
				}else if (sum <= 5000 && current_st_is_ocupied){
					current_st_is_ocupied = 0;
					//system("curl -H \"Content-Type: application/json\" -d '{\"state\":1}' http://parkingluck.cloudas.info/pl1/rest/parkings/012?token=MTEzMjU5OTkwOUNBQUNFZEVvc2UwY0JBSURaQW91UXpSWXVIek9zN2ZPeW9JczVVQVpDTk96UnJSSkI1cVc0Z1pBY1lCWkJSczVaQVpBUEtuYWZUUkhueXpwWFVLR1VaQ3VTSHlIbmVud09YcXk0WkIwWkI0eUptbFJoSlU1b1htODkzWGNMOG5IWTgwWEpSM0F6UlpCRFpBWkMxa1pBRGl3cVpBQ3lyWkE5ZnJBVEg4SHNqM1dUTkJKcGJJMzlDUzQ1c1hEeVdTb3FHUPARKINGLUCKLOVEPARKING");
					printf("send the state=1 to server\n");
				}
					
			#endif
				}
			#endif 
		}
	}


	//----- CLOSE THE UART -----
	return ;
}

int main(int argc, char * argv){

	#if 1
	//-------------------------
	//----- SETUP USART 0 -----
		//-------------------------
	//At bootup, pins 8 and 10 are already set to UART0_TXD, UART0_RXD (ie the alt0 function) respectively	
	
	//OPEN THE UART
	//The flags (defined in fcntl.h):
	//	Access modes (use 1 of these):
	//		O_RDONLY - Open for reading only.
	//		O_RDWR - Open for reading and writing.
	//		O_WRONLY - Open for writing only.
	//
	//	O_NDELAY / O_NONBLOCK (same function) - Enables nonblocking mode. When set read requests on the file can return immediately with a failure status
	//											if there is no input immediately available (instead of blocking). Likewise, write requests can also return
	//											immediately with a failure status if the output can't be written immediately.
	//
	//	O_NOCTTY - When set and path identifies a terminal device, open() shall not cause the terminal device to become the controlling terminal for the process.
	fd = open("/dev/ttyAMA0", O_RDONLY | O_NOCTTY | O_NDELAY);		//Open in non blocking read/write mode
	if (fd == -1)
	{
		//ERROR - CAN'T OPEN SERIAL PORT
		printf("Error - Unable to open UART.  Ensure it is not in use by another application\n");
	}

	// Turn off blocking for reads, use (fd, F_SETFL, FNDELAY) if you want that
	/* This effectly clears all flags on the file descriptor */
	//fcntl(uart0_filestream, F_SETFL, FNDELAY);
	//fcntl(fd, F_SETFL, 0);
	//CONFIGURE THE UART
	//The flags (defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html):
	//	Baud rate:- B1200, B2400, B4800, B9600, B19200, B38400, B57600, B115200, B230400, B460800, B500000, B576000, B921600, B1000000, B1152000, B1500000, B2000000, B2500000, B3000000, B3500000, B4000000
	//	CSIZE:- CS5, CS6, CS7, CS8
	//	CLOCAL - Ignore modem status lines
	//	CREAD - Enable receiver
	//	IGNPAR = Ignore characters with parity errors
	//	ICRNL - Map CR to NL on input (Use for ASCII comms where you want to auto correct end of line characters - don't use for bianry comms!)
	//	PARENB - Parity enable
	//	PARODD - Odd parity (else even)
	struct termios options;
	tcgetattr(fd, &options);
	options.c_cflag &= ~CSTOPB;	// one stop bit
	//options.c_iflag = IGNPAR;	// enable parity check
	options.c_cflag &= ~(PARENB | PARODD);      // shut off parity
	options.c_cflag &= ~CRTSCTS;
	options.c_iflag = INPCK;	// disable parity check
	options.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
	options.c_oflag = 0;
	options.c_lflag = 0;
	//options.c_cc[VMIN] = 0;
	//options.c_cc[VTIME] = 1;
	options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;		//<Set baud rate
	tcflush(fd, TCIFLUSH);
	if(tcsetattr(fd, TCSANOW, &options) != 0)
		printf("comport setting error!!!!\n");

	signal(SIGINT, get_sigint);
	//----- CHECK FOR ANY RX BYTES -----
	#endif

	printf("comport setting finish!!!!\n");
	pthread_t id;	
	int ret = pthread_create(&id, NULL, thread_rev, NULL);

	while(!get_signal_int){};
	
	close(fd);
	printf("close uart port....\n");	
	return 0;
}
