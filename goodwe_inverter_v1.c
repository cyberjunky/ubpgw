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


// 	AP 	-->	INV 	off_line_query 		FC=0x00, AP=0x88, INV=0x7F
//  header(2),  orig, dest, CC,   FC,   LEN,  checksum(2) 
//	0xAA, 0x55, 0x88, 0x7F, 0x00, 0x00, 0x00, 0xCS, 0xCS
//
// 	INV	-->	AP	 	register_request	FC=0x80, INV=0x7F, AP=0x88 + SN
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


#define		TTY_BUFF_SIZE	 		  2048	// Buffer size for storing the TTY (P1 telegram) data
#define		RECV_TIME_OUT 		  	  6000	// HTTP Receive time-out in mSec 

// ---------------------------------------------------------------------------- 
// ---------------------------------------------------------------------------- 


int		rx_tty_off_line_query(char *str_tty_data, int tty_char);
int 	recv_to(int fd, char *buffer, int len, int flags, int time_out);
void 	hexdump(char *desc, void *addr, int len);

// ---------------------------------------------------------------------------- 
// ---------------------------------------------------------------------------- 


int main(int argc,char** argv)
{
	struct termios options;
	int tty_fd;
	int retval;
	
	//char p_tty_data[TTY_BUFF_SIZE];
	// char p_tty_data[TTY_BUFF_SIZE] = {0x55, 0x88, 0x7F, 0x00, 0x00, 0x00, 0xAB, 0xCD};
	
	// 	AP 	-->	INV 	off_line_query 		FC=0x00, AP=0x88, INV=0x7F
	//  header(2),  orig, dest, CC,   FC,   LEN,  checksum(2) 
	//	0xAA, 0x55, 0x88, 0x7F, 0x00, 0x00, 0x00, 0xCS, 0xCS
	char str_off_line_query[] = {0xAA, 0x55, 0x88, 0x7F, 0x00, 0x00, 0x00, 0xAB, 0xCD};
	
	// 	INV	-->	AP	 	register_request	FC=0x80, INV=0x7F, AP=0x88 + SN
	//  header(2),  orig, dest, CC,   FC,   LEN,  DATA ......... (16), checksum(2) 
	//  SN = 13000SSU11000008
	//	0xAA, 0x55, 0x7F, 0x88, 0x00, 0x80, 0x10, 0x01, 0x02, .. 0x10, 0xCS, 0xCS
	//char str_register_request[] = {0xAA, 0x55, 0x7F, 0x88, 0x00, 0x80, 0x10, 
	//	  '0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F', 0xAB, 0xCD};
	char str_register_request[] = {0xAA, 0x55, 0x88, 0x7F, 0xCC, 0xFC, 0x1E, 0xAB, 0xCD};
			
	unsigned char tty_char = 'D';


	printf("Starting %s\n", argv[0]);

	//printf("str_off_line_query = %s length: %d \n", str_off_line_query, sizeof(str_off_line_query));
	//printf("str_register_request = %s length: %d \n", str_register_request, sizeof(str_register_request));

	hexdump("str_off_line_query:", str_off_line_query, sizeof(str_off_line_query));
	hexdump("str_register_request:", str_register_request, sizeof(str_register_request));

	//bzero(str_off_line_query, sizeof(str_off_line_query));
	//bzero(str_register_request, sizeof(str_register_request));

	//printf("str_off_line_query = %s length: %d \n", str_off_line_query, sizeof(str_off_line_query));
	//printf("str_register_request = %s length: %d \n", str_register_request, sizeof(str_register_request));

	//exit(0);
	
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
		tcsetattr(tty_fd, TCSANOW, &options);							// Set tty propoerties
	}

	cfsetospeed(&options, B9600);            							// 9600 baud
	cfsetispeed(&options, B9600);            							// 9600 baud

	tcsetattr(tty_fd, TCSANOW, &options);
	
	while (1)
	{
		retval = read(tty_fd, &tty_char, 1);
		if(retval > 0) 
		{
			//write(STDOUT_FILENO, &tty_char, 1);        				// if new data is available on the serial port, print it out
			
			retval = rx_tty_off_line_query(str_off_line_query, tty_char);
			if(retval == 0)
			{
				// hex dump request from AP  
				hexdump("> RECEIVED - str_tty_off_line_query:", str_off_line_query, sizeof(str_off_line_query));
				// hex dump response to AP 
				hexdump("> SENDING - str_register_request:", str_register_request, sizeof(str_register_request));
				// send data
				write(tty_fd, str_register_request, sizeof(str_register_request)); 
			}
		}
		else
		{
			// printf("Serial line read time-out occured...\n");
		}
	}
	
	close(tty_fd);
	
	return 0;
}




/*
 * rx_tty_off_line_query:
 * Read all the datafields from the P1 telegram.
 *********************************************************************************
 */
							
int	rx_tty_off_line_query(char *str_tty_data, int tty_char)
{
	int				retval = -1;
	static int		tty_state_machine = STATE_TTY_00;
	static int		pntr_buffer_tty = 0;
	
	// 	AP 	-->	INV 	off_line_query 		FC=0x00, AP=0x88, INV=0x7F
	//  header(2),  orig, dest, CC,   FC,   LEN,  checksum(2) 
	//	0xAA, 0x55, 0x88, 0x7F, 0x00, 0x00, 0x00, 0xCS, 0xCS
	
	switch (tty_state_machine)
	{
		case STATE_TTY_00:												// check for Goodwe header 1

			if (tty_char == (char) 0xAA)								// GoodWe header1 0xAA
			{
				pntr_buffer_tty = 0;
				bzero(str_tty_data, TTY_BUFF_SIZE);
				str_tty_data[pntr_buffer_tty] = tty_char;
				pntr_buffer_tty++;
				printf("Received GoodWe header (0xAA) : %x \n", tty_char);
				tty_state_machine = STATE_TTY_01;
			}
			else
			{
				printf("Received %x \n", tty_char);
				tty_state_machine = STATE_TTY_00;
			}
			retval = -1;
			break;
			
		case STATE_TTY_01: 												// Check for GoodWe header 2 

			if (tty_char == (char) 0x55)								// GoodWe header2 0x55
			{
				str_tty_data[pntr_buffer_tty] = tty_char;
				pntr_buffer_tty++;
				printf("Received GoodWe header (0x55) : %x \n", tty_char);
				tty_state_machine = STATE_TTY_02;
			}
			else
			{
				printf("Received %x \n", tty_char);
				tty_state_machine = STATE_TTY_00;
			}
			retval = -1;
			break;

		case STATE_TTY_02: 												// continue reading 

			str_tty_data[pntr_buffer_tty] = tty_char;					// store byte
			pntr_buffer_tty++;
			printf("Received originator address: %x \n", tty_char);
			tty_state_machine = STATE_TTY_03;
			retval = -1;
			break;

		case STATE_TTY_03: 												// continue reading 

			str_tty_data[pntr_buffer_tty] = tty_char;					// store byte 
			pntr_buffer_tty++;
			printf("Received destination address: %x \n", tty_char);
			tty_state_machine = STATE_TTY_04;
			retval = -1;
			break;

		case STATE_TTY_04: 												// continue reading 

			str_tty_data[pntr_buffer_tty] = tty_char;					// store byte 
			pntr_buffer_tty++;
			printf("Received control code: %x \n", tty_char);
			tty_state_machine = STATE_TTY_05;
			retval = -1;
			break;

		case STATE_TTY_05: 												// continue reading 

			str_tty_data[pntr_buffer_tty] = tty_char;					// store byte 
			pntr_buffer_tty++;
			printf("Received function code: %x \n", tty_char);
			tty_state_machine = STATE_TTY_06;
			retval = -1;
			break;

		case STATE_TTY_06: 												// continue reading 

			str_tty_data[pntr_buffer_tty] = tty_char;					// store byte 
			pntr_buffer_tty++;
			printf("Received length field: %x \n", tty_char);
			tty_state_machine = STATE_TTY_07;
			retval = -1;
			break;

		case STATE_TTY_07: 												// continue reading 

			str_tty_data[pntr_buffer_tty] = tty_char;					// store byte 
			pntr_buffer_tty++;
			printf("Received checksum (msb): %x \n", tty_char);
			tty_state_machine = STATE_TTY_08;
			retval = -1;
			break;

		case STATE_TTY_08: 												// continue reading 

			str_tty_data[pntr_buffer_tty] = tty_char;					// store byte 
			pntr_buffer_tty++;
			printf("Received checksum (lsb): %x \n", tty_char);
			tty_state_machine = STATE_TTY_00;
			retval = 0;
			break;

		default:
		
			printf("ERROR this state does not exist %d: default \n", tty_state_machine);
			tty_state_machine = STATE_TTY_00;
			retval = -1;
			break;

	}
	
	return retval;
	
}


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


