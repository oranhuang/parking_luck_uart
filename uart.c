#include <stdio.h>
#include <unistd.h>		//Used for UART
#include <fcntl.h>		//Used for UART
#include <termios.h>		//Used for UART
#include <signal.h>
#include <errno.h>
#include <stdlib.h>
#include <pthread.h>
#include <sys/select.h>
#include <string.h>

#define USED		1
#define AVAILABLE	0
#define TRUE		1
#define FALSE		0
//char *str = "curl -H "Content-Type: application/json" -d '{"state":3}' http://parkingluck.cloudas.info/pl1/rest/parkings/012?token=MTEzMjU5OTkwOUNBQUNFZEVvc2UwY0JBSURaQW91UXpSWXVIek9zN2ZPeW9JczVVQVpDTk96UnJSSkI1cVc0Z1pBY1lCWkJSczVaQVpBUEtuYWZUUkhueXpwWFVLR1VaQ3VTSHlIbmVud09YcXk0WkIwWkI0eUptbFJoSlU1b1htODkzWGNMOG5IWTgwWEpSM0F6UlpCRFpBWkMxa1pBRGl3cVpBQ3lyWkE5ZnJBVEg4SHNqM1dUTkJKcGJJMzlDUzQ1c1hEeVdTb3FHUPARKINGLUCKLOVEPARKING";
//char *str = {"ad"b"};

char *str_1 = "curl -H \"Content-Type: application/json\" -d '{\"availablecar\":\"";
char *str_2 = "\",\"reservedcar\":\"";
char *str_3 = "\"}' \"http://parkingluck.cloudas.info/pl1/rest/management/PKL1000?id=10203591950055986&token=MTEzMjU5OTkwOUNBQUNFZEVvc2UwY0JBSURaQW91UXpSWXVIek9zN2ZPeW9JczVVQVpDTk96UnJSSkI1cVc0Z1pBY1lCWkJSczVaQVpBUEtuYWZUUkhueXpwWFVLR1VaQ3VTSHlIbmVud09YcXk0WkIwWkI0eUptbFJoSlU1b1htODkzWGNMOG5IWTgwWEpSM0F6UlpCRFpBWkMxa1pBRGl3cVpBQ3lyWkE5ZnJBVEg4SHNqM1dUTkJKcGJJMzlDUzQ1c1hEeVdTb3FHUPARKINGLUCKLOVEPARKING\"";

volatile sig_atomic_t get_signal_int = 0;
int TOTAL_PARKING_SPACE = 0;
int avai_parking_space;
char str_pool[512];
typedef struct _Parking_lot_status{	
	int parking_NO;
	unsigned char status;
	unsigned char ack;
}_parking_lot_status;

_parking_lot_status *pk_lot;

void get_sigint(int a)
{
	printf("^C caught\n");
	get_signal_int = 1;
}

void str_concate(char *str1){
	strcat(str_pool, str1);
	return ;
}

void *uart_rev(){
	#if 1
	//-------------------------
	//----- SETUP USART 0 -----
		//-------------------------
	//At bootup, pins 8 and 10 are already set to UART0_TXD, UART0_RXD (ie the alt0 function) respectively
	int uart0_filestream = -1;
	
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
	uart0_filestream = open("/dev/ttyAMA0", O_RDONLY | O_NOCTTY);// | O_NDELAY);		//Open in non blocking read/write mode
	if (uart0_filestream == -1)
	{
		//ERROR - CAN'T OPEN SERIAL PORT
		printf("Error - Unable to open UART.  Ensure it is not in use by another application\n");
	}

	// Turn off blocking for reads, use (fd, F_SETFL, FNDELAY) if you want that
	/* This effectly clears all flags on the file descriptor */
	//fcntl(uart0_filestream, F_SETFL, FNDELAY);
	fcntl(uart0_filestream, F_SETFL, 0);
	//CONFIGURE THE UART
	//The flags (defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html):
	//	Baud rate:- B1200, B2400, B4800, B9600, B19200, B38400, B57600, B115200, B230400, B460800, B500000, B576000, B921600, B1000000, B1152000, B1500000, B2000000, B2500000, B3000000, B3500000, B4000000
	//	CSIZE:- CS5, CS6, CS7, CS8
	//	CLOCAL -tatus lines
	//	CREAD - Enable receiver
	//	IGNPAR = Ignore characters with parity errors
	//	ICRNL - Map CR to NL on input (Use for ASCII comms where you want to auto correct end of line characters - don't use for bianry comms!)
	//	PARENB - Parity enable
	//	PARODD - Odd parity (else even)
	/*
	struct termios options;	
	tcgetattr(uart0_filestream, &options);
	options.c_cflag &= ~CSTOPB;	// one stop bit
	options.c_iflag = IGNPAR;	// enable parity check
	options.c_cflag &= ~(PARENB | PARODD);      // shut off parity
	options.c_cflag &= ~CRTSCTS;
	options.c_iflag = INPCK;	// disable parity check
	options.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
	options.c_oflag = 0;
	options.c_lflag = 0;
	//options.c_cc[VMIN] = 0;
	//options.c_cc[VTIME] = 1;
	options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;		//<Set baud rate
	tcflush(uart0_filestream, TCIFLUSH);
	if(tcsetattr(uart0_filestream, TCSANOW, &options) != 0)
		printf("comport setting error!!!!\n");
	*/
	struct termios tty;
	int spd = B115200;
	cfsetospeed(&tty, (speed_t)spd);
	cfsetispeed(&tty, (speed_t)spd);

        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;

        tty.c_cflag &= ~CRTSCTS;    /* no HW flow control? */
        tty.c_cflag |= CLOCAL | CREAD;

        tty.c_iflag |= IGNPAR | IGNCR;
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_lflag |= ICANON;
        tty.c_oflag &= ~OPOST;

        int rc = tcsetattr(uart0_filestream, TCSANOW, &tty);
        if (rc < 0) {
         /* handle error */
        }
	//signal(SIGINT, get_sigint);
	//----- CHECK FOR ANY RX BYTES -----
	#endif
	unsigned char rx_buffer[255];
	unsigned char rx_data;
	int rx_length, i=0, ret;
	int end_cnt=0, data_acquire=0;
	char str_available[8], str_reserved[8];
	memset(str_available, 0x0, 8);
	memset(str_reserved, 0x0, 8);

	#if 0
	int uart0_filestream, len;
	uart0_filestream = open("/dev/ttyAMA0", O_RDWR |  S_IRUSR | S_IWUSR);
	fcntl(uart0_filestream, F_SETFL, 0);
	char temp;
	#endif
	fd_set readfds;
	if (uart0_filestream != -1)
	{
		printf("start receiving data....\n");
		while(1)
		{
			usleep(1);
			FD_ZERO(&readfds);
			FD_SET(uart0_filestream, &readfds);
			ret = select(uart0_filestream+1, &readfds, NULL, NULL, NULL);		
	
			if(ret > 0){
			// Read up to 255 characters from the port if they are there
			//rx_length = read(uart0_filestream, (void*)rx_buffer, 255);		//Filestream, buffer to store in, number of bytes to read (max)
			rx_length = read(uart0_filestream, &rx_data, sizeof(rx_data));		//Filestream, buffer to store in, number of bytes to read (max)	
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
				//printf("%s\n", rx_buffer);				
				//printf("data=%d, rx len=%d\n", (int)(rx_buffer[0]*256) + (int)(rx_buffer[1]), rx_length);
				//for(i=0;i<255;i++)
				//	rx_buffer[i] = 0;
				#if 1
				rx_buffer[i] = rx_data;
				//fprintf(stderr, "data = %x, index=%d\n", rx_data, i);
				i++;				
				switch (rx_data){
					/* check the tail of the data , that's the 0xF0 0xF0 0xA0 0xA0*/
					case 0xF0 :
						end_cnt ++;					
						if(end_cnt > 2)
							end_cnt = 1;
						break;
					case 0xA0 :
						if(end_cnt == 2){							
							end_cnt++;
						}
						if(end_cnt == 3){
							data_acquire = 1;
							end_cnt = 0;
						}
						break;
					default :				
					break;
				}
				if(data_acquire){
					//printf("%s", rx_buffer);
					//for(j=0;j<2;j++)
					data_acquire = 0;
					//sum = (int)(rx_buffer[i-5]*256) + (int)(rx_buffer[i-4]);
					//printf("the abs value = %d, len=%d, %x %x, \n",sum ,i, rx_buffer[i-5],rx_buffer[i-4]);//,rx_buffer[i-7], rx_buffer[i-5], rx_buffer[i-4],rx_buffer[i-3] ,rx_buffer[i-2],rx_buffer[i-1]);
					//printf("magnetic abs value = %4.4fGa\n", (float)sum*0.92);
					//i=0;
				
					if(rx_buffer[i-4] == 0x01){
						//fprintf(stderr, "NO.%d parking lot, Is Used Now \n", rx_buffer[i-5]);
						if(!(pk_lot + rx_buffer[i-5])->ack){
							(pk_lot + rx_buffer[i-5])->status = USED;
							(pk_lot + rx_buffer[i-5])->ack = TRUE;
							avai_parking_space--;
							fprintf(stderr, "NO.%d parking lot, Is Used Now \n", rx_buffer[i-5]);
						}
					}
					else{
						if((pk_lot + rx_buffer[i-5])->ack){
							(pk_lot + rx_buffer[i-5])->status = AVAILABLE;
							(pk_lot + rx_buffer[i-5])->ack = FALSE;
						}
						fprintf(stderr, "NO.%d parking lot, Is Available Now \n", rx_buffer[i-5]);
						avai_parking_space++;
						if(avai_parking_space > TOTAL_PARKING_SPACE)
							avai_parking_space = TOTAL_PARKING_SPACE;						
					}
					i = 0;
	
						memset(str_pool, 0x0, sizeof(str_pool));
						sprintf(str_available, "%d", avai_parking_space);
						sprintf(str_reserved, "%d", TOTAL_PARKING_SPACE - avai_parking_space);
						str_concate(str_1);
						str_concate(str_available);
						str_concate(str_2);
						str_concate(str_reserved);
						str_concate(str_3);
						//fprintf(stderr, "%s\n", str_pool);
						system(str_pool);
						//system ("curl -H \"Content-Type: application/json\" -d '{\"availablecar\":%t_available,\"reservedcar\":\"50\"}' \"http://parkingluck.cloudas.info/pl1/rest/management/PKL1000?id=10203591950055986&token=MTEzMjU5OTkwOUNBQUNFZEVvc2UwY0JBSURaQW91UXpSWXVIek9zN2ZPeW9JczVVQVpDTk96UnJSSkI1cVc0Z1pBY1lCWkJSczVaQVpBUEtuYWZUUkhueXpwWFVLR1VaQ3VTSHlIbmVud09YcXk0WkIwWkI0eUptbFJoSlU1b1htODkzWGNMOG5IWTgwWEpSM0F6UlpCRFpBWkMxa1pBRGl3cVpBQ3lyWkE5ZnJBVEg4SHNqM1dUTkJKcGJJMzlDUzQ1c1hEeVdTb3FHUPARKINGLUCKLOVEPARKING\"");
						fprintf(stderr, "\n");

						//system("curl -H \"Content-Type: application/json\" -d '{\"availabecar\":str_available, \"reservedcar\",str_reserved}' \"http://parkingluck.cloudas.info/pl1/rest/management/PKL1000?id=10203591950055986&token=MTEzMjU5OTkwOUNBQUNFZEVvc2UwY0JBSURaQW91UXpSWXVIek9zN2ZPeW9JczVVQVpDTk96UnJSSkI1cVc0Z1pBY1lCWkJSczVaQVpBUEtuYWZUUkhueXpwWFVLR1VaQ3VTSHlIbmVud09YcXk0WkIwWkI0eUptbFJoSlU1b1htODkzWGNMOG5IWTgwWEpSM0F6UlpCRFpBWkMxa1pBRGl3cVpBQ3lyWkE5ZnJBVEg4SHNqM1dUTkJKcGJJMzlDUzQ1c1hEeVdTb3FHUPARKINGLUCKLOVEPARKING\"");
				#if 0
							
					if(sum > 4000 && !current_st_is_ocupied){
						current_st_is_ocupied = 1;
						system("curl -H \"Content-Type: application/json\" -d '{\"state\":3}' http://parkingluck.cloudas.info/pl1/rest/parkings/012?token=MTEzMjU5OTkwOUNBQUNFZEVvc2UwY0JBSURaQW91UXpSWXVIek9zN2ZPeW9JczVVQVpDTk96UnJSSkI1cVc0Z1pBY1lCWkJSczVaQVpBUEtuYWZUUkhueXpwWFVLR1VaQ3VTSHlIbmVud09YcXk0WkIwWkI0eUptbFJoSlU1b1htODkzWGNMOG5IWTgwWEpSM0F6UlpCRFpBWkMxa1pBRGl3cVpBQ3lyWkE5ZnJBVEg4SHNqM1dUTkJKcGJJMzlDUzQ1c1hEeVdTb3FHUPARKINGLUCKLOVEPARKING");
						printf("send the state=3 to server, car is here\n");
					}else if (sum <= 4000 && current_st_is_ocupied){
						current_st_is_ocupied = 0;
						system("curl -H \"Content-Type: application/json\" -d '{\"state\":1}' http://parkingluck.cloudas.info/pl1/rest/parkings/012?token=MTEzMjU5OTkwOUNBQUNFZEVvc2UwY0JBSURaQW91UXpSWXVIek9zN2ZPeW9JczVVQVpDTk96UnJSSkI1cVc0Z1pBY1lCWkJSczVaQVpBUEtuYWZUUkhueXpwWFVLR1VaQ3VTSHlIbmVud09YcXk0WkIwWkI0eUptbFJoSlU1b1htODkzWGNMOG5IWTgwWEpSM0F6UlpCRFpBWkMxa1pBRGl3cVpBQ3lyWkE5ZnJBVEg4SHNqM1dUTkJKcGJJMzlDUzQ1c1hEeVdTb3FHUPARKINGLUCKLOVEPARKING");
						printf("send the state=1 to server, car is moving out\n");
					}
					
				#endif

				}
				#endif 
			}
			} // if(ret)
		}
	}

	//----- CLOSE THE UART -----
	close(uart0_filestream);
	printf("close uart port....\n");	
	return 0;
}

*void queue_task(){

	while(1){
		if(queue_task_count){
			
			
			queue_task_count--;
		}
		usleep(1);
	}
}

int main(int argc, char* argv[]){
	int ret;

	pthread_t id_uart_rev, id_queue_task;

	if(argc < 3){
		if(argc < 2)
			fprintf(stderr, "PLEASE enter the numbers of parking space with \"-n\" \n");
		else
			fprintf(stderr, "PLEASE input the numbers of parking spaces AGAIN\n");
		
		return 0;
	}

	if(strcmp(argv[1], "-n") == 0){		
		TOTAL_PARKING_SPACE = atoi(argv[2]);
		if(TOTAL_PARKING_SPACE == 0){
			fprintf(stderr, "PLEASE input the numbers of parking spaces AGAIN\n");
			return 0;
		}
	}
		
	avai_parking_space = TOTAL_PARKING_SPACE;
	pk_lot = malloc(sizeof(_parking_lot_status)*TOTAL_PARKING_SPACE);
	fprintf(stderr, "total available parking lots are %d\n", avai_parking_space);

	ret = pthread_create(&id_uart_rev, NULL, uart_rev, NULL);
	if(ret != 0){
	        fprintf(stderr, "socket_recv thread creation fail!!!\n");
        	return -1;
        }
	ret = pthread_create(&id_queue_task, NULL, queue_task, NULL);
	if(ret != 0){
	        fprintf(stderr, "queue task thread creation fail!!!\n");
        	return -1;
        }

	signal(SIGINT, get_sigint);
	while(!get_signal_int){
		usleep(1);
	}
	
	free(pk_lot);
	//pthread_join(id_uart_rev, NULL);

	return 0;

}
