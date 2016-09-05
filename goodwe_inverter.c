// ---------------------------------------------------------------------
// FROM: https://en.wikibooks.org/wiki/Serial_Programming/termios
// ---------------------------------------------------------------------

// GOOD-WE INVERTER DATA PACKET
//
// HEADER_MSB		0xAA								- 01 bytes
// HEADER_LSB		0x55								- 01 bytes
// ORIG_ADDRESS		0xAA 								- 01 bytes
// DEST_ADDRESS		0xAA								- 01 bytes
// CONTROL_CODE		0x00 for register and 0x01 for read	- 01 bytes
// FUNCTION_CODE	0xAA								- 01 bytes
// DATA_LENGTH		0xAA								- 01 bytes
// SN + ADDRESS		0x00, 0x01, 0x02, ... 0x0F 0x10   	- 17 bytes
// CHECK_SUM_MSB	0xCS								- 01 bytes
// CHECK_SUM_LSB	0xCS								- 01 bytes
//													------------------+				
// TOTAL BYTES											- 41 bytes
	
// FUNCTION CODES for REGISTER process CONTROL_CODE = 0x00
//
//  AP ADDRESS = 0x88
//  ASSIGNED INV ADDRESS = 0x11, 0x12, 0x13, etc...
//	PAYLOAD DATA = SERIAL NUMBER + ADDRESS 
//  SERIAL NUMBER length =  16 bytes 
//	ADDRESS length = 1 byte
//
// 	AP 	-->	INV 	off_line_query 		FC=0x00, AP=0x88, INV=0x7F
//  header(2),  orig, dest, CC,   FC,   LEN,  checksum(2) 
//	0xAA, 0x55, 0x88, 0x7F, 0x00, 0x00, 0x00, 0xCS, 0xCS
//
// 	INV	-->	AP	 	register_request	FC=0x80, INV=0x7F, AP=0x88 + SN
//  header(2),  orig, dest, CC,   FC,   LEN,  DATA ......... (16), checksum(2) 
//	0xAA, 0x55, 0x7F, 0x88, 0x00, 0x80, 0x10, 0x01, 0x02, .. 0x10, 0xCS, 0xCS
//
// 	AP 	-->	INV 	assign_address 		FC=0x01, AP=0x88, INV=0x7F + SN + ADDRESS=0x11
//  header(2),  orig, dest, CC,   FC,   LEN,  DATA ......... (16), checksum(2) 
//	0xAA, 0x55, 0x88, 0x7F, 0x00, 0x01, 0x11, 0x01, 0x02, .. 0x11, 0xCS, 0xCS
//
// 	INV	-->	AP	 	address_confirm		FC=0x81, INV=ADDRESS=0x11, AP=0x88
//  header(2),  orig, dest, CC,   FC,   LEN,  DATA ......... (16), checksum(2) 
//	0xAA, 0x55, 0x11, 0x88, 0x00, 0x81, 0x00, 0x01, 0x02, .. 0x10, 0xCS, 0xCS
//
//
// 	AP 	-->	INV 	remove_register		FC=0x02
//  header(2),  orig, dest, CC,   FC,   LEN,  checksum(2) 
//	0xAA, 0x55, 0x88, 0x11, 0x00, 0x02, 0x00, 0xCS, 0xCS
//
// 	INV	-->	AP	 	remove_confirm		FC=0x82
//  header(2),  orig, dest, CC,   FC,   LEN,  checksum(2) 
//	0xAA, 0x55, 0x11, 0x88, 0x00, 0x82, 0x10, 0xCS, 0xCS
//


// FUNCTION CODES for READ process CONTROL_CODE = 0x01
//
//  AP ADDRESS = 0x88
//  ASSIGNED INV ADDRESS = 0x11, 0x12, 0x13, etc...
//	PAYLOAD INFO length = 32 x 2 bytes = 64 bytes 
//  SERIAL NUMBER length =  16 bytes 
//	ADDRESS length = 1 byte
//
// 	AP 	-->	INV 	query_running_info 		FC=0x00, AP=0x88, INV=0x7F
//  header(2),  orig, dest, CC,   FC,   LEN,  checksum(2) 
//	0xAA, 0x55, 0x88, 0x7F, 0x00, 0x00, 0x00, 0xCS, 0xCS
//
// 	INV	-->	AP	 	response_running_info	FC=0x80, INV=0x7F, AP=0x88 + SN
//  header(2),  orig, dest, CC,   FC,   LEN,  DATA ......... (16), checksum(2) 
//	0xAA, 0x55, 0x7F, 0x88, 0x00, 0x80, 0x10, 0x01, 0x02, .. 0x10, 0xCS, 0xCS




#include <stdio.h>
#include <stdlib.h>
#include <string.h>  		/* String function definitions */
#include <netdb.h>
#include <unistd.h>  		/* UNIX standard function definitions */
#include <fcntl.h>   		/* File control definitions */
#include <netinet/in.h>
#include <errno.h>   		/* Error number definitions */
#include <termios.h> 		/* POSIX terminal control definitions */
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <time.h> 			/* POSIX time control definitions */


// States for tty state machine
#define		STATE_TTY_HEADER_AA				 0	// Statemachine for read tty data 
#define		STATE_TTY_HEADER_55				 1	// Statemachine for read tty data 
#define		STATE_TTY_ORIG_ADDR				 2	// Statemachine for read tty data 
#define		STATE_TTY_DEST_ADDR  			 3	// Statemachine for read tty data 
#define		STATE_TTY_CONTROL_CODE  		 4	// Statemachine for read tty data 
#define		STATE_TTY_FUNCTION_CODE  		 5	// Statemachine for read tty data 
#define		STATE_TTY_LENGTH  				 6	// Statemachine for read tty data 
#define		STATE_TTY_PAYLOAD  				 7	// Statemachine for read tty data 
#define		STATE_TTY_CS_MSB  				 8	// Statemachine for read tty data 
#define		STATE_TTY_CS_LSB  				 9	// Statemachine for read tty data 
#define		STATE_TTY_10  					10	// Statemachine for read tty data 
#define		STATE_TTY_11  					11	// Statemachine for read tty data 
#define		STATE_TTY_12  					12	// Statemachine for read tty data 
#define		STATE_TTY_13  					13	// Statemachine for read tty data 
#define		STATE_TTY_14  					14	// Statemachine for read tty data 
#define		STATE_TTY_15  					15	// Statemachine for read tty data 

// State Machine states
#define	STATE_TTY_00  	0
#define	STATE_TTY_01  	1
#define	STATE_TTY_02  	2
#define	STATE_TTY_03  	3
#define	STATE_TTY_04  	4
#define	STATE_TTY_05  	5
#define	STATE_TTY_06  	6
#define	STATE_TTY_07  	7
#define	STATE_TTY_08  	8
#define	STATE_TTY_09  	9


// State Machine states
#define		STATE_INIT_TTY 				 0	// Init state 
#define		STATE_WAIT_FOR_QUERY  		 1  // Wait for query from GoodWe AP
#define		STATE_REGISTER_REQUEST	 	 2	// Send: register request state 
#define		STATE_CONFIRM_ADDRESS		 3	// Send: confirm address state 
#define		STATE_CONFIRM_REMOVE		 4	// Send: confirm remove state 
#define		STATE_RESPONSE_INFO 		 5	// Send: response_info state 

#define		TTY_BUFF_SIZE	 		  2048	// Buffer size for storing the TTY (P1 telegram) data
#define		RECV_TIME_OUT 		  	  6000	// HTTP Receive time-out in mSec 

// ---------------------------------------------------------------------------- 
// ---------------------------------------------------------------------------- 


int		rx_tty_wait_for_query(char *str_tty_data, int tty_data);
int		rx_tty_response_info(char *str_tty_data, int tty_data);

int		validate_checksum(char *str_goodwe_message);
int 	append_checksum(char *str_goodwe_message, int length);

void 	hexdump(char *desc, void *addr, int len);

// ---------------------------------------------------------------------------- 
// ---------------------------------------------------------------------------- 


int main(int argc,char** argv)
{
	struct termios options;
	int tty_fd = -1;
	int retval = -1;
	int	state_machine = STATE_INIT_TTY;
	unsigned char tty_char = 'D';
	
	// 	AP 	-->	INV 	off_line_query 		FC=0x00, AP=0x88, INV=0x7F
	//  header(2),  orig, dest, CC,   FC,   LEN,  checksum(2) 
	//	0xAA, 0x55, 0x88, 0x7F, 0x00, 0x00, 0x00, 0xCS, 0xCS
	//  But can also be:
	// 	AP 	-->	INV 	assign_address 		FC=0x01, AP=0x88, INV=0x7F + SN + ADDRESS=0x11
	//  header(2),  orig, dest, CC,   FC,   LEN,  DATA ......... (16), checksum(2) 
	//	0xAA, 0x55, 0x88, 0x7F, 0x00, 0x01, 0x10, 0x01, 0x02, .. 0x11, 0xCS, 0xCS
	char str_incoming_query[80] ;
	
	// 	INV	-->	AP	 	register_request	FC=0x80, INV=0x7F, AP=0x88 + SN
	//  header(2),  orig, dest, CC,   FC,   LEN,  DATA ......... (16), checksum(2) 
	//  SN = 13000SSU11000008
	char str_register_request[] = {0xAA, 0x55, 0x7F, 0x88, 0x00, 0x80, 0x10,
		 '0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F', 0xCD, 0xEF};

	// 	INV	-->	AP	 	address_confirm		FC=0x81, INV=ADDRESS=0x11, AP=0x88
	//  						  header(2),  orig, dest, CC,   FC,   LEN, checksum(2) 
	char str_address_confirm[] = {0xAA, 0x55, 0xAD, 0x88, 0x00, 0x81, 0x00, 0xCD, 0xEF};

	// 	INV	-->	AP	 	remove_confirm		FC=0x82
	//  header(2),  orig, dest, CC,   FC,   LEN,  checksum(2) 
	char str_remove_confirm[] = {0xAA, 0x55, 0x7F, 0x88, 0x00, 0x82, 0x00, 0xCD, 0xEF};

	// 	INV	-->	AP	 	response_running_info	FC=0x80, INV=0x7F, AP=0x88 + SN
	//  header(2),  orig, dest, CC,   FC,   LEN,  DATA ......... (16), checksum(2) 
	char str_response_info[] = {0xAA, 0x55, 0x7F, 0x88, 0x01, 0x81, 0x40, 
		 '0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F',
		 '0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F',
		 '0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F',
		 '0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F', 
		 0xAB, 0xCD};
			
	
	//-----------------------------------------------------------------------
	// printf("Starting %s\n", argv[0]);
	//-----------------------------------------------------------------------


	printf("----------------------------------------------\n");
	printf(" GoodWe RS485 INVERTER SIMULATOR        v0.1  \n");
	printf("----------------------------------------------\n");

	
	if(append_checksum(str_register_request, sizeof(str_register_request)) == -1) 
	{
		printf("Incorrect - str_register_request.\n");
		exit(-1);
	}
	else
	{
		hexdump("str_register_request", str_register_request, sizeof(str_register_request));
	}

	if(append_checksum(str_address_confirm, sizeof(str_address_confirm)) == -1)
	{
		printf("Incorrect - str_address_confirm.\n");
		exit(-1);
	}
	else
	{
		hexdump("str_address_confirm", str_address_confirm, sizeof(str_address_confirm));	
	}

	if(append_checksum(str_remove_confirm, sizeof(str_remove_confirm)) == -1)
	{
		printf("Incorrect - str_remove_confirm.\n");
		exit(-1);
	}
	else
	{
		hexdump("str_remove_confirm", str_remove_confirm, sizeof(str_remove_confirm));
	}

	if(append_checksum(str_response_info, sizeof(str_response_info)) == -1)
	{
		printf("Incorrect - str_response_info.\n");
		exit(-1);
	}
	else
	{
		hexdump("str_response_info", str_response_info, sizeof(str_response_info));
	}


	bzero(str_incoming_query, sizeof(str_incoming_query));				// Clear buffer for incoming messages
	
	while (1)
	{
		switch (state_machine)
		{
			case STATE_INIT_TTY:										// Initialize state machine

				memset(&options, 0, sizeof(options));
				options.c_iflag = 0;
				options.c_oflag = 0;
				//options.c_cflag = CS7 | CREAD | CLOCAL | PARENB;  	// 7n1, see termios.h for more information
				//options.c_cflag = CS8 | CREAD | CLOCAL | PARENB;  	// 8n1, see termios.h for more information
				options.c_cflag = CS8 | CREAD | CLOCAL ;  				// 8n1, see termios.h for more information
				options.c_lflag = 0;
				
				// Warning: The VMIN and VTIME flags are ignored because the O_NONBLOCK flag is set.
				options.c_cc[VMIN] = 1;
				options.c_cc[VTIME] = 5;

				printf("\n");
				printf("termios.c_iflag    : %x\n", options.c_iflag);
				printf("termios.c_oflag    : %x\n", options.c_oflag);
				printf("termios.c_cflag    : %x\n", options.c_cflag);
				printf("termios.c_lflag    : %x\n\n", options.c_lflag);

				//tty_fd = open(argv[1], O_RDWR | O_NONBLOCK);      
				tty_fd = open("/dev/ttyUSB1", O_RDWR | O_NONBLOCK);      
				if (tty_fd == -1)
				{
					printf("ERROR connecting to /dev/ttyUSB1 \n");
					exit(-1);
				}
				else
				{
					printf("Device /dev/ttyUSB1 connected on fd: %d \n", tty_fd);
					tcsetattr(tty_fd, TCSANOW, &options);				// Set tty properties
				}
				
				printf(">> STATE_WAIT_FOR_QUERY:\n");
				state_machine = STATE_WAIT_FOR_QUERY;
				
				break; 


			case STATE_WAIT_FOR_QUERY:									// Wait for incoming query
				
				retval = read(tty_fd, &tty_char, 1);
				if(retval > 0) 
				{
					retval = rx_tty_wait_for_query(str_incoming_query, tty_char);
					if(retval > STATE_WAIT_FOR_QUERY)
					{
						state_machine = retval;							// Next State depening on incoming request
					}
				}
				else
				{
					// printf(".");
				}
				break;

			case STATE_REGISTER_REQUEST:
				
				hexdump("<< STATE_REGISTER_REQUEST", str_register_request, sizeof(str_register_request));
				sleep(1); 			// DEBUG DELAY TO TEST TIME_OUT BEHAVIOUR
				write(tty_fd, str_register_request, sizeof(str_register_request)); 
				printf(">> STATE_WAIT_FOR_QUERY:\n");
				state_machine = STATE_WAIT_FOR_QUERY;
				break;

			case STATE_CONFIRM_ADDRESS:

				hexdump("<< STATE_CONFIRM_ADDRESS", str_address_confirm, sizeof(str_address_confirm));
				write(tty_fd, str_address_confirm, sizeof(str_address_confirm)); 
				printf(">> STATE_WAIT_FOR_QUERY:\n");
				state_machine = STATE_WAIT_FOR_QUERY;
				break;

			case STATE_CONFIRM_REMOVE:

				hexdump("<< STATE_CONFIRM_REMOVE", str_remove_confirm, sizeof(str_remove_confirm));
				write(tty_fd, str_remove_confirm, sizeof(str_remove_confirm)); 
				printf(">> STATE_WAIT_FOR_QUERY:\n");
				state_machine = STATE_WAIT_FOR_QUERY;
				break;

			case STATE_RESPONSE_INFO:
			
				hexdump("<< STATE_RESPONSE_INFO", str_response_info, sizeof(str_response_info));
				write(tty_fd, str_response_info, sizeof(str_response_info)); 
				printf(">> STATE_WAIT_FOR_QUERY:\n");
				state_machine = STATE_WAIT_FOR_QUERY;
				break;

		}
	}
	
	close(tty_fd);
	return 0;
}




/*
 * rx_tty_response_info:
 * Read all the datafields from response info.
 *********************************************************************************
 */
							
int	rx_tty_wait_for_query(char *str_tty_data, int tty_data)
{
	int				retval = -1;
	static int		tty_state_machine = STATE_TTY_HEADER_AA;
	static int		pntr_buffer_tty = 0;

	static int		int_control_code = -1;
	static int		int_function_code = -1;
	static int		int_length = -1;
	
	// 	AP 	-->	INV 	off_line_query 		FC=0x00, AP=0x88, INV=0x7F
	//  header(2),  orig, dest, CC,   FC,   LEN,  checksum(2) 
	//	0xAA, 0x55, 0x88, 0x7F, 0x00, 0x00, 0x00, ...... , 0xAB, 0xCD
	
	switch (tty_state_machine)
	{
		case STATE_TTY_HEADER_AA:										// check for Goodwe header 1

			if (tty_data == (int) 0xAA)									// GoodWe header1 0xAA
			{
				pntr_buffer_tty = 0;
				str_tty_data[pntr_buffer_tty] = tty_data;
				pntr_buffer_tty++;
				printf("Received header (0xAA) : 0x%x \n", tty_data);
				tty_state_machine = STATE_TTY_HEADER_55;
			}
			else
			{
				printf("Waiting for header (0xAA). Received: 0x%x \n", tty_data);
				tty_state_machine = STATE_TTY_HEADER_AA;
			}
			break;
			
		case STATE_TTY_HEADER_55: 										// Check for GoodWe header 2 

			if (tty_data == (int) 0x55)									// GoodWe header2 0x55
			{
				str_tty_data[pntr_buffer_tty] = tty_data;
				pntr_buffer_tty++;
				printf("Received header (0x55) : 0x%x \n", tty_data);
				tty_state_machine = STATE_TTY_ORIG_ADDR;
			}
			else
			{
				printf("Received 0x%x \n", tty_data);
				tty_state_machine = STATE_TTY_HEADER_AA;
				retval = -2;											// Incorrect data received
			}
			break;

		case STATE_TTY_ORIG_ADDR: 										// continue reading 

			str_tty_data[pntr_buffer_tty] = tty_data;					// store byte
			pntr_buffer_tty++;
			printf("Received originator address: 0x%x \n", tty_data);
			tty_state_machine = STATE_TTY_DEST_ADDR;
			break;

		case STATE_TTY_DEST_ADDR: 										// continue reading 

			str_tty_data[pntr_buffer_tty] = tty_data;					// store byte 
			pntr_buffer_tty++;
			printf("Received destination address: 0x%x \n", tty_data);
			tty_state_machine = STATE_TTY_CONTROL_CODE;
			break;

		case STATE_TTY_CONTROL_CODE: 									// continue reading 

			int_control_code = tty_data;
			str_tty_data[pntr_buffer_tty] = tty_data;					// store byte 
			pntr_buffer_tty++;
			printf("Received control code: 0x%x \n", tty_data);
			tty_state_machine = STATE_TTY_FUNCTION_CODE;
			break;

		case STATE_TTY_FUNCTION_CODE: 									// continue reading 

			int_function_code = tty_data;
			str_tty_data[pntr_buffer_tty] = tty_data;					// store byte 
			pntr_buffer_tty++;
			printf("Received function code: 0x%x \n", tty_data);
			tty_state_machine = STATE_TTY_LENGTH;
			break;

		case STATE_TTY_LENGTH: 											// continue reading 

			int_length = tty_data;										// copy length for next state
			str_tty_data[pntr_buffer_tty] = tty_data;// store byte 
			pntr_buffer_tty++;
			printf("Received length field: 0x%x \n", int_length);
			if(int_length == 0)
			{
				tty_state_machine = STATE_TTY_CS_MSB;
			}
			else
			{
				tty_state_machine = STATE_TTY_PAYLOAD;
			}
			break;

		case STATE_TTY_PAYLOAD: 										// Reading multiple payload data bytes 

			str_tty_data[pntr_buffer_tty] = tty_data;					// store data bytes according to length
			pntr_buffer_tty++;
			printf("Received payload data byte: 0x%x \n", tty_data);
			int_length--;												// store payload bytes (loop)
			if(int_length == 0)
			{
				tty_state_machine = STATE_TTY_CS_MSB;
			}
			break;

		case STATE_TTY_CS_MSB: 											// continue reading 

			str_tty_data[pntr_buffer_tty] = tty_data;					// store byte in struct
			pntr_buffer_tty++;
			printf("Received checksum (msb): 0x%x \n", tty_data);
			tty_state_machine = STATE_TTY_CS_LSB;
			break;

		case STATE_TTY_CS_LSB: 											// continue reading 

			str_tty_data[pntr_buffer_tty] = tty_data;					// store byte 
			pntr_buffer_tty++;
			printf("Received checksum (lsb): 0x%x \n", tty_data);
			if(validate_checksum(str_tty_data) == 0)
			{
				
				tty_state_machine = STATE_TTY_HEADER_AA;				// Done and go wait for new response

				if((int_function_code == 0) && (int_control_code == 0))
				{
					retval = STATE_REGISTER_REQUEST;
				}
				else if((int_function_code == 1) && (int_control_code == 0))
				{
					retval = STATE_CONFIRM_ADDRESS;
				}
				
				else if((int_function_code == 2) && (int_control_code == 0))
				{
					retval = STATE_CONFIRM_REMOVE;
				}
				
				else if((int_function_code == 1) && (int_control_code == 1))
				{
					retval = STATE_RESPONSE_INFO;
				}
				else
				{
					retval = STATE_WAIT_FOR_QUERY;						// Received unsupported fc or cc
				}
			}
			else
			{
				tty_state_machine = STATE_TTY_HEADER_AA;				// Done and go wait for new response			
				retval = -2;											// Indicate that the checksum is incorrect
			}
			break;

		default:
		
			printf("ERROR this state does not exist %d: default \n", tty_state_machine);
			tty_state_machine = STATE_TTY_HEADER_AA;
			retval = -1;
			break;

	}
	
	return retval;
	
}



// ---------------------------------------------------------------------------- 
// ---------------------------------------------------------------------------- 




/*
   Params:
      fd       	- (int) socket file descriptor
      buffer 	- (char*) buffer to hold data
      len     	- (int) maximum number of bytes to recv()
      flags   	- (int) flags (as the fourth param to recv() )
      to       	- (int) timeout in milliseconds
   Results:
      int     	- The same as recv, but -2 == TIMEOUT
   Notes:
      You can only use it on file descriptors that are sockets!
      'to' must be different to 0
      'buffer' must not be NULL and must point to enough memory to hold at least 'len' bytes
*/
int recv_to(int fd, char *buffer, int len, int flags, int time_out) 
{

   fd_set readset;
   int result, iof = -1;
   struct timeval tv;

   // Initialize the set
   FD_ZERO(&readset);
   FD_SET(fd, &readset);
   
   // Initialize time out struct
   tv.tv_sec = 0;
   tv.tv_usec = time_out * 1000;
   
   // select()
   result = select(fd+1, &readset, NULL, NULL, &tv);

   // Check status
   if (result < 0)
   {
      return -1;
   }
   else if (result > 0 && FD_ISSET(fd, &readset)) 
   {
      // Set non-blocking mode
      if ((iof = fcntl(fd, F_GETFL, 0)) != -1)
      {
         fcntl(fd, F_SETFL, iof | O_NONBLOCK);
	  }
      
      // receive
      result = recv(fd, buffer, len, flags);
      
      // set as before
      if (iof != -1)
      {
         fcntl(fd, F_SETFL, iof);
	  }
      return result;
   }
   return -2;
}

// ---------------------------------------------------------------------------- 
// ---------------------------------------------------------------------------- 

/*
 * append_checksum:
 * Append the checksum for any type of request or response 
 *********************************************************************************
 */

int append_checksum(char *str_goodwe_message, int int_length)
{
	int		pntr_buffer_tty = 0;
	int		int_checksum = 0;
	int		data_length = 0;
	
	//printf("append_checksum - length field = %d\n", str_goodwe_message[6]);
	
	if((str_goodwe_message[6] + 9) != int_length) 						// header (7) + datalength (?) + checksum (2) == int_length
	{
		printf("error: append_checksum - incorrect length = %d, (0x%x)\n", int_length, int_length);
		return -1;
	}
	
	// header msb
	int_checksum = int_checksum + (int) str_goodwe_message[pntr_buffer_tty];
	pntr_buffer_tty++;
	// header lsb
	int_checksum = int_checksum + (int) str_goodwe_message[pntr_buffer_tty];
	pntr_buffer_tty++;
	// source address
	int_checksum = int_checksum + (int) str_goodwe_message[pntr_buffer_tty];
	pntr_buffer_tty++;
	// destination address
	int_checksum = int_checksum + (int) str_goodwe_message[pntr_buffer_tty];
	pntr_buffer_tty++;
	// control code
	int_checksum = int_checksum + (int) str_goodwe_message[pntr_buffer_tty];
	pntr_buffer_tty++;
	// function code
	int_checksum = int_checksum + (int) str_goodwe_message[pntr_buffer_tty];
	pntr_buffer_tty++;
	// data length
	int_checksum = int_checksum + (int) str_goodwe_message[pntr_buffer_tty];
	data_length = str_goodwe_message[pntr_buffer_tty];
	pntr_buffer_tty++;

	//printf("append_checksum - data_length field = %d\n", data_length);
		
	while(data_length > 0)
	{
		int_checksum = int_checksum + (int) str_goodwe_message[pntr_buffer_tty];
		pntr_buffer_tty++;
		data_length--;
	}  

	// Convert int to checksum_msb and checksum_lsb
	str_goodwe_message[pntr_buffer_tty] = (char) (int_checksum >> 8);			// Shift bits 8 positions to the right 
	pntr_buffer_tty++;
	str_goodwe_message[pntr_buffer_tty] = (char) int_checksum & 0x00FF;			// Mask off all more significant bytes
	
	//hexdump("append_checksum - str_goodwe_message", str_goodwe_message, int_length);  

	return 0;
}



/*
 * validate_checksum:
 * Calculate the checksum for any type of request or response 
 *********************************************************************************
 */

int validate_checksum(char *str_goodwe_message)
{
	int		pntr_buffer_tty = 0;
	int		int_checksum = 0;
	int		data_length = 0;
	char 	checksum_msb;
	char 	checksum_lsb;
	
	// header msb
	int_checksum = int_checksum + (int) str_goodwe_message[pntr_buffer_tty];
	pntr_buffer_tty++;
	// header lsb
	int_checksum = int_checksum + (int) str_goodwe_message[pntr_buffer_tty];
	pntr_buffer_tty++;
	// source address
	int_checksum = int_checksum + (int) str_goodwe_message[pntr_buffer_tty];
	pntr_buffer_tty++;
	// destination address
	int_checksum = int_checksum + (int) str_goodwe_message[pntr_buffer_tty];
	pntr_buffer_tty++;
	// control code
	int_checksum = int_checksum + (int) str_goodwe_message[pntr_buffer_tty];
	pntr_buffer_tty++;
	// function code
	int_checksum = int_checksum + (int) str_goodwe_message[pntr_buffer_tty];
	pntr_buffer_tty++;
	// data length
	int_checksum = int_checksum + (int) str_goodwe_message[pntr_buffer_tty];
	data_length = str_goodwe_message[pntr_buffer_tty];
	pntr_buffer_tty++;
	
	while(data_length > 0)
	{
		int_checksum = int_checksum + (int) str_goodwe_message[pntr_buffer_tty];
		pntr_buffer_tty++;
		data_length--;
	}  

	checksum_msb = (char) (int_checksum >> 8);			// Shift bits 8 positions to the right 
	checksum_lsb = (char) int_checksum & 0x00FF;		// Mask off all more significant bytes

	//printf("checksum msb in string = %d, 0x%x \n", str_goodwe_message[str_goodwe_message[6] + 7], (int) *cs_msb);
	//printf("checksum lsb in string = %d, 0x%x \n", str_goodwe_message[str_goodwe_message[6] + 8], (int) *cs_lsb);
	
	// check if the checksum in the string is correct
	if((str_goodwe_message[str_goodwe_message[6] + 7] == checksum_msb) && (str_goodwe_message[str_goodwe_message[6] + 8] == checksum_lsb))
	{
		return 0;	// checksum is ok!
	}
	else
	{
		return -1;	// checksum is NOT ok!
	}
}


// ---------------------------------------------------------------------------- 
// ---------------------------------------------------------------------------- 



void hexdump(char *desc, void *addr, int len) 
{
    int i;
    unsigned char buff[17];
    unsigned char *pc = (unsigned char*)addr;

    // Output description if given.
    if (desc != NULL)
        printf ("%s:\n", desc);

    if (len == 0) 
    {
        printf("  ZERO LENGTH\n");
        return;
    }
    if (len < 0) 
    {
        printf("  NEGATIVE LENGTH: %i\n",len);
        return;
    }

    // Process every byte in the data.
    for (i = 0; i < len; i++) 
    {
        // Multiple of 16 means new line (with line offset).

        if ((i % 16) == 0) 
        {
            // Just don't print ASCII for the zeroth line.
            if (i != 0)
            {
                printf ("  %s\n", buff);
			}
            // Output the offset.
            printf ("  %04x ", i);
        }

        // Now the hex code for the specific character.
        printf (" %02x", pc[i]);

        // And store a printable ASCII character for later.
        if ((pc[i] < 0x20) || (pc[i] > 0x7e))
        {
            buff[i % 16] = '.';
		}
        else
        {
            buff[i % 16] = pc[i];
        }    
        buff[(i % 16) + 1] = '\0';
    }

    // Pad out last line if not exactly 16 characters.
    while ((i % 16) != 0) 
    {
        printf ("   ");
        i++;
    }

    // And print the final ASCII bit.
    printf ("  %s\n", buff);
}


