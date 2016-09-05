/***********************************************************************
 * 
 *  Project: Upp Box Pro - GoodWe inverter interface:
 * 
 *  Release notes:
 * 
 *  V0.1 - UBP GoodWe propriatary interface over RS485
 *  - Ref specification: 
 *  - DATA COMM: 9600 BAUD, 8 bits, no parity, 1 stopbit
 *  
 **********************************************************************/


/***************************************************************************
 * 
 *  Geany compile and build directives  
 *  
 *  Compile: 	gcc -Wall -g -O -o "%e" "%f" "ini.c" "jsmn.c" "-lm" "-lcurl" "-lwiringPi"
 *  Build: 		gcc -Wall -g -O -o "%e" "%f" "ini.c" "jsmn.c" "-lcurl" "-lwiringPi"
 *
 ***************************************************************************/

/* *********************************************************************
 * 
 *	  https://projects.drogon.net/raspberry-pi/wiringpi/
 *
 *    wiringPi is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    wiringPi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public License
 *    along with wiringPi.  If not, see <http://www.gnu.org/licenses/>.
 * 
 ***********************************************************************
 * 
 *    For pin assignment please refer to:
 * 
 *    https://learn.sparkfun.com/tutorials/raspberry-gpio/gpio-pinout
 * 
 **********************************************************************/
 

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
//	
//
//     Registering new Inverter (control code == 0)
//
//  AP   off-line-query									   INV
//	| --------------------------------------------------->> |
//	
//     									 register_request
//	| <<--------------------------------------------------- |
//	
//       assign_address
//	| --------------------------------------------------->> |
//	
//     									 confirm_address
//	| <<--------------------------------------------------- |
//	
//       remove_register
//	| --------------------------------------------------->> |
//	
//     									 confirm_remove
//	| <<--------------------------------------------------- |
//	
//
//
//    Reading data from Inverter (control code == 1)
//
//  AP   request_info									   INV
//	| --------------------------------------------------->> |
//	
//     										 response_info
//	| <<--------------------------------------------------- |
//	
//	
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
// 	INV	-->	AP	 	confirm_address		FC=0x81, INV=ADDRESS=0x11, AP=0x88
//  header(2),  orig, dest, CC,   FC,   LEN,  DATA ......... (16), checksum(2) 
//	0xAA, 0x55, 0x11, 0x88, 0x00, 0x81, 0x00, 0x01, 0x02, .. 0x10, 0xCS, 0xCS
//
//  Remove :
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
//	0xAA, 0x55, 0x88, 0x11, 0x01, 0x00, 0x00, 0xCS, 0xCS
//
// 	INV	-->	AP	 	response_running_info	FC=0x80, INV=0x7F, AP=0x88 + SN
//  header(2),  orig, dest, CC,   FC,   LEN,  DATA ......... (64), checksum(2) 
//	0xAA, 0x55, 0x11, 0x88, 0x01, 0x80, 0x40, 0x01, 0x02, .. 0x3F, 0xCS, 0xCS





/***********************************************************************
* --- JSON GoodWe PAYLOAD EXAMPLE --------------------------------------
************************************************************************
*
* {
*   "boxUuId":"16606002",
*	"sensors":
*	[
*		{"sensorUuId":"GWPT0001", "timestamp":"2016-08-08T11:21:54.000Z", "load": 0.000000, "total": 0}
*		{"sensorUuId":"GWPT0002", "timestamp":"2016-08-08T11:21:54.000Z", "load": 0.000000, "total": 0}
*	]
* }
*
***********************************************************************/


/***********************************************************************
 * 
 * http://uppenergy.com/public/api/store/measurement/16607001 
 *
 **********************************************************************/


/***********************************************************************
 * 
 * # Config file for UBPMB
 * 
 * [config]               			; S0 sensor configuration
 * UppBoxID 	= 16607001       	; UppBox ID YYMDDnnn where M is Month in Hex
 * 
 * # Input device
 * Device		= /dev/ttyUSB0		; S0 tty input port				
 * Baudrate		= 9600				; Baudrate of the tty device connected
 * 
 * # URI parameters
 * Hostname		= 192.168.2.4
 * Port			= 8080
 * Directory	= /public/api/store/measurement/
 * 
 * 
 **********************************************************************/

  
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
#include <wiringPi.h>		/* wiringPi library to control GPIO pins */
#include "ini.h" 			/* inih - INI not invented here library */
#include "jsmn.h"			/* jsmn - Jasmin JSON parser library */	

// #include "struct.h" 		/* Single structure that hoolds all S0 related data */


// States for Main state machine
#define		STATE_INIT_TTY 					 1	// Init state 
#define		STATE_OFF_LINE_QUERY			 2	// Send: off_line_query state 
#define		STATE_REGISTER_REQUEST	 		 3	// Receive: register_request state 
#define		STATE_ASSIGN_ADDRESS		 	 4	// Send: assign_address state 
#define		STATE_CONFIRM_ADDRESS			 5	// Receive: confirm_address state 
#define		STATE_REQUEST_INFO				 6	// Send: read_info state 
#define		STATE_RESPONSE_INFO 			 7	// Receive: response_info state 
#define		STATE_WAIT_INTERVAL 			 8	// Receive: response_info state 
#define		STATE_OFF_REMOVE_ALL 			 9	// Receive: response_info state 

#define		STATE_SEND_HTTP	  				10	// Send http state 
#define		STATE_HTTP_RESPONSE				11	// Respond to http response state 
#define		STATE_SLEEP_LED_OFF				12	// Wait 1 sec and turn LED's off

// States for receiving GooWe messages tty state machine
#define		STATE_TTY_HEADER_AA				 0	// State for reading tty data 
#define		STATE_TTY_HEADER_55				 1	// State for reading tty data 
#define		STATE_TTY_ORIG_ADDR				 2	// State for reading tty data 
#define		STATE_TTY_DEST_ADDR  			 3	// State for reading tty data 
#define		STATE_TTY_CONTROL_CODE  		 4	// State for reading tty data 
#define		STATE_TTY_FUNCTION_CODE  		 5	// State for reading tty data 
#define		STATE_TTY_LENGTH  				 6	// State for reading tty data 
#define		STATE_TTY_PAYLOAD  				 7	// State for reading tty data 
#define		STATE_TTY_CS_MSB  				 8	// State for reading tty data 
#define		STATE_TTY_CS_LSB  				 9	// State for reading tty data 
#define		STATE_TTY_10  					10	// State for reading tty data 
#define		STATE_TTY_11  					11	// State for reading tty data 
#define		STATE_TTY_12  					12	// State for reading tty data 
#define		STATE_TTY_13  					13	// State for reading tty data 
#define		STATE_TTY_14  					14	// State for reading tty data 
#define		STATE_TTY_15  					15	// State for reading tty data 

//----------------------------------------------------------------------
// Constant definitions
//----------------------------------------------------------------------

#define		GPIO_PIN_RESET					29	// SWITCH INTerrupt wiringPi pin 29
#define		GPIO_PIN_23						 4	// SWITCH INTerrupt wiringPi pin 4
#define		GPIO_PIN_24						 5	// SWITCH INTerrupt wiringPi pin 5
#define		GPIO_PIN_25						 6	// SWITCH INTerrupt wiringPi pin 6

#define		LED_RED							26	// LED RED 		--> wiringPi pin 1 
#define		LED_YELLOW						22	// LED YELLOW 	--> wiringPi pin 4
#define		LED_GREEN						21	// LED GREEN 	--> wiringPi pin 5
#define		LED_FLASH					   300	// LED flash interval HELLO
#define		LED_FLASH_ON			   	   100	// LED flash ON interval in mSec
#define		MAIN_LOOP_DELAY			   	  1000	// MAIN LOOP DELAY in mSec

#define		MAX_NUMBER_OF_INVERTERS			10	// Max number of inverters
#define		SENSOR_BUFF_SIZE			 	 9	// Hold JSON str name for sensor (last char is unique) zero terminated
#define 	SN_BUFF_SIZE	 			    17	// Buffer size for serial number

#define		HTTP_PAYLOAD_SIZE			  2048	// Buffer size for storing the HTTP POST payload
#define		HTTP_HEADER_SIZE			  2048	// Buffer size for storing the HTTP POST payload
#define		HTTP_REQUEST_SIZE 			  2048	// Buffer size for storing the HTTP request from UppEnergy
#define		HTTP_RESPONSE_SIZE 			  2048	// Buffer size for storing the HTTP response from UppEnergy
#define		MODBUS_RESPONSE_SIZE		  2048	// Buffer size for storing the Modbus response from Modbus Client
#define		TTY_BUFF_SIZE	 			    80	// Buffer size for storing the TTY (P1 telegram) data
#define		MB_DATA_FIELD	 			    64	// Buffer size for storing individual P1 data fields

#define		SIZE_REGISTER_REQUEST		   256	// Buffer size for GoodWe register request
#define		SIZE_REGISTER_RESPONSE		   256	// Buffer size for GoodWe register response
#define		SIZE_READ_INFO_REQUEST		   256	// Buffer size for GoodWe register request
#define		SIZE_READ_INFO_RESPONSE		   256	// Buffer size for GoodWe register response

#define		RECV_TIME_OUT 		  		  6000	// HTTP Receive time-out in mSec 
#define		MY_ADDRESS					  0x88	// MY ADDRESS used for orig and expected for receipient


// ---------------------------------------------------------------------
// --- NEW typedev struct for SENSORS
// ---------------------------------------------------------------------


typedef struct
{
	
	// Indicate if this entry in the address table is used by a registered for GoodWe protocol
	// 0x00 indicates an empty slot and 0x01 indicated that a invertered is registered at this entry 
	char registered; 													// Indicates if inverter is registered and active
	
	// Asigned address for GoodWe protocol
	// The maximum number of inverters on a single bus is limited to 10.
	// This is because of dynamic JSON sensor ID assignment 
	// The last digit is dynamically assigned and in the range of ASCII '0'-'9' 
	char identifier; 													// Assigned address ('0'-'9')
	
	// Stores the serial number of the inverter
	char serial_number[SN_BUFF_SIZE]; 									// Serial number from inverter
	
	// JSON name of the sensor (= inverter) comes from config.ini 
	// The last char uniquely identifies the inverter on the RS485 bus 
	// For this implementation it is limited to 10 inverters on a bus
	// The last digit is dynamically assigned and is linked to the table entry
    char str_ini_sensor_id[SENSOR_BUFF_SIZE];							// Store the JSON name of the sensor (= inverter) comes from config.ini 

	// Holds the number of time-outs in a row to detect if an existing inverter has been disabled
	int timeout;

	// Store latest values for load (power) and total (energy) 
	int average_power;												// calculated average power in Watt
	int total_energy;												// calculated total enegry in Wh 
	
} t_UBP_SENSOR_DATA;



// ---------------------------------------------------------------------
// --- NEW struct for UPP BOX
// ---------------------------------------------------------------------



typedef struct
{
	// Store parameters from config.ini
    char *str_ini_upp_box_id;					// S0 tty input device name e.g. /dev/ttyACM0
	char *str_ini_sensor_id;					// GoodWe sensor ID template (last digit is dynamic)

	char *str_ini_hostname;						// Hostname
	char *str_ini_directory;					// Resource locator
	char *str_ini_portno;						// Port number
	int   int_ini_portno;						// Integer value of port number

	// GoodWe RS485 response buffers 
	char str_tty_response[TTY_BUFF_SIZE];		// One size fits all
	
	// Holds the new_serial_number of the undiscovered inverter (or discovered but reset inverter) 
	char new_serial_number[SN_BUFF_SIZE]; 		// Hold the newly discovered serial number
	
	// Holds the new entry that is provided by itable_new_entry()
	uint8_t new_entry_number;
	
	// Buffer size for HTTP POST request and response 
	char str_http_header[HTTP_HEADER_SIZE];
	char str_http_payload[HTTP_PAYLOAD_SIZE];
	char str_http_request[HTTP_REQUEST_SIZE];

	// HTTP response data from Upp Energy platform 
	char str_http_response[HTTP_RESPONSE_SIZE]; 
			
	t_UBP_SENSOR_DATA sensor[MAX_NUMBER_OF_INVERTERS];
	
} t_UBP_DATA;



// ---------------------------------------------------------------------
// --- declare prototypes ----------------------------------------------
// ---------------------------------------------------------------------



void 	create_http_payload(t_UBP_DATA *pUBP_DATA);

int		rx_tty_register_request(t_UBP_DATA *pUBP_DATA, int tty_char);
int		rx_tty_confirm_address(t_UBP_DATA *pUBP_DATA, int tty_char);
int		rx_tty_response_info(t_UBP_DATA *pUBP_DATA, int tty_data);

void 	init_wiringPi (t_UBP_DATA *pUBP_DATA);
void	init_config(t_UBP_DATA *pUBP_DATA);

int 	recv_to(int fd, char *buffer, int len, int flags, int time_out);

void 	flash_GREEN_LED(void);
void	flash_LEDs(void);

int 	ttyOpen (const char *device, const int baud);
void  	ttyClose(const int fd);
void  	ttyFlush(const int fd);
void  	ttyPutchar(const int fd, const unsigned char c);
void  	ttyPuts(const int fd, const char *s);
int   	ttyDataAvail(const int fd);
int   	ttyGetchar(const int fd);

void 	hexdump(char *desc, void *addr, int len);
int 	append_checksum(char *str_goodwe_message, int length);
int		validate_checksum(char *str_goodwe_message);


uint8_t	itable_initialize(t_UBP_DATA *pUBP_DATA);
uint8_t	itable_find_entry(t_UBP_DATA *pUBP_DATA);
uint8_t	itable_add_entry(t_UBP_DATA *pUBP_DATA, uint8_t entry);
uint8_t	itable_delete_entry(t_UBP_DATA *pUBP_DATA, uint8_t entry);


void bincpyx(char *dest, char *src, unsigned int x);	// Copy binary string



// ---------------------------------------------------------------------
// --- declare global variables ----------------------------------------
// ---------------------------------------------------------------------


long 	lng_TOTAL_PULSES_S0M1;											// Total number of S0 pulses received since start
long 	lng_TOTAL_PULSES_S0M2;											// Total number of S0 pulses received since start
long 	lng_TOTAL_PULSES_S0M3;											// Total number of S0 pulses received since start


// ---------------------------------------------------------------------
// wiringPi call back routine for switch interrupts   
// ---------------------------------------------------------------------


void call_back_wiringPi_GPIO_PIN_RESET(void) 
{ 
	printf("Performing a gracefull shutdown... \n");
	digitalWrite(LED_RED, HIGH);										// LED RED ON
	sleep(2); 
	//system("sudo shutdown -r now"); 
	system("sudo shutdown -h now"); 
}


void call_back_wiringPi_GPIO_PIN_23(void) 
{ 
	//pullUpDnControl(GPIO_PIN_23, PUD_DOWN);							// Disable pull-up
	lng_TOTAL_PULSES_S0M1++;											// Increment S0 pulses for M1
	digitalWrite(LED_GREEN, HIGH);										// LED ON
	printf("TOTAL S0 impulses on GPIO 23: %ld\n", lng_TOTAL_PULSES_S0M1);
	delay(LED_FLASH_ON);
	digitalWrite(LED_GREEN, LOW);										// LED OFF
	//pullUpDnControl(GPIO_PIN_23, PUD_UP);								// Enable pull-up
}


void call_back_wiringPi_GPIO_PIN_24(void) 
{ 
	//pullUpDnControl(GPIO_PIN_24, PUD_DOWN);							// Disable pull-up
	lng_TOTAL_PULSES_S0M2++;											// Increment S0 pulses for M2
	digitalWrite(LED_GREEN, HIGH);										// LED ON
	printf("TOTAL S0 impulses on GPIO 24: %ld\n", lng_TOTAL_PULSES_S0M2);
	delay(LED_FLASH_ON);
	digitalWrite(LED_GREEN, LOW);										// LED OFF
	//pullUpDnControl(GPIO_PIN_24, PUD_UP);								// Enable pull-up
}


void call_back_wiringPi_GPIO_PIN_25(void) 
{ 
	//pullUpDnControl(GPIO_PIN_25, PUD_DOWN);							// Disable pull-up
	lng_TOTAL_PULSES_S0M3++;											// Increment S0 pulses for M3
	digitalWrite(LED_GREEN, HIGH);										// LED ON
	printf("TOTAL S0 impulses on GPIO 25: %ld\n", lng_TOTAL_PULSES_S0M3);
	delay(LED_FLASH_ON);
	digitalWrite(LED_GREEN, LOW);										// LED OFF
	//pullUpDnControl(GPIO_PIN_25, PUD_UP);								// Enable pull-up
}


// ---------------------------------------------------------------------
// The ini_call_back_handler is called from the inih.c lib to store data  
// ---------------------------------------------------------------------

static int inih_call_back_handler(void* user, const char* section, const char* name, const char* value)
{
    t_UBP_DATA* pUBP_DATA = (t_UBP_DATA*)user;

    #define MATCH(s, n) strcmp(section, s) == 0 && strcmp(name, n) == 0
    
	// Initialize all UBP BOX related config parameters
	
    if (MATCH("config", "UppBoxID")) 
    {
        pUBP_DATA->str_ini_upp_box_id = strdup(value);
    } 
    else if (MATCH("config", "SensorID")) 
    {
        pUBP_DATA->str_ini_sensor_id = strdup(value);
    } 
    else if (MATCH("config", "Hostname")) 
    {
        pUBP_DATA->str_ini_hostname = strdup(value);
    } 
    else if (MATCH("config", "Directory")) 
    {
        pUBP_DATA->str_ini_directory = strdup(value);
    } 
    else if (MATCH("config", "Port")) 
    {
        pUBP_DATA->str_ini_portno = strdup(value);
        pUBP_DATA->int_ini_portno = atoi(pUBP_DATA->str_ini_portno);
    } 
    else 
    {
        return 0;  /* unknown section or name, error */
    }
    return 1;
}


// ---------------------------------------------------------------------
// main 
// ---------------------------------------------------------------------


int main(int argc, char *argv[])
{

	// socket variables for HTTP communication 
	int http_fd = -1; 		// UppEnergy file descriptor
	struct sockaddr_in backend_addr;
	struct hostent *backend_server;

	// tty variables for RS485 communication
	int	tty_fd = -1;				// ttyUSBx file descriptor
	char tty_char;					// tty input

	// struct termios options;	
	
	// Generic variables
	int	retval, i;		

	// For keeping statistics on the backend_server response
	int iCount_HTTP_OK = 0;
	int iCount_HTTP_NOK = 0;
	int iCount_TIMEOUT = 0;
	int iCount_MBTIMEOUT = 0;

	// state machine switch
	uint8_t state_machine = STATE_INIT_TTY;
	   
	// Main struct that holds all the data   
	t_UBP_DATA UBP_DATA;

	// for analysing the http response
	char *pJSON_payload;						// Used for strstr compare 		
	char str_compare[32];
	
	// variables for jsmn JSON parsing
	int jsmn_tokens;							// Number of tokens parsed by jsmn_parse
	jsmn_parser jsmn_p;							// Create jsmn parser struct
	jsmntok_t jsmn_token_array[128]; 			// We expect no more than json 128 tokens 

	char str_jsmn_t1[256];						// Used to store the JSON HTTP response field
	char str_jsmn_t2[256];						// Used to store the JSON HTTP response field
	char str_jsmn_t3[256];						// Used to store the JSON HTTP response field
	char str_jsmn_t4[256];						// Used to store the JSON HTTP response field
	char str_jsmn_t5[256];						// Used to store the JSON HTTP response field
	char str_jsmn_t6[256];						// Used to store the JSON HTTP response field
	char str_jsmn_t7[256];						// Used to store the JSON HTTP response field
	char str_jsmn_t8[256];						// Used to store the JSON HTTP response field

	long lTotal;
	char *ptr;


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


	// Request offline query:    header(2),  orig, dest, CC,   FC,   LEN,  checksum(2)    
	char str_off_line_query[] = {0xAA, 0x55, 0x88, 0x7F, 0x00, 0x00, 0x00, 0x02, 0x06};
	
	// Request assign address:   header(2),  orig, dest, CC,   FC,   LEN,  DATA(17), checksum(2)    
	char str_assign_address[] = {0xAA, 0x55, 0x88, 0x7F, 0x00, 0x01, 0x11, 0xD0, 0xD1, 0xD2, 0xD3, 0xD4, 0xD5, 0xD6, 0xD7, 
		 0xD8, 0xD9, 0xDA, 0xDB, 0xDC, 0xDD, 0xDE, 0xDF, 0xAD, 0x10, 0x3D};

	// Request remove address:   header(2),  orig, dest, CC,   FC,   LEN,  checksum(2)    
	char str_remove_address[] = {0xAA, 0x55, 0x88, 0xAD, 0x00, 0x02, 0x00, 0x02, 0x36};

	// Request request_info:   header(2),  orig, dest, CC,   FC,   LEN,  checksum(2)    
	char str_request_info[] = {0xAA, 0x55, 0x88, 0xAD, 0x01, 0x01, 0x00, 0x02, 0x36};

	// FAKE JSON STRING :
	//const char *JSON_STRING = "{\"type\":\"newSensorTotal\",\"boxUuId\":\"16606002\",\"sensorUuId\":\"S0M10001\",\"newTotal\":12345}";
	//const char *JSON_STRING = "{\"type\":\"newSensorTotal\",\"boxUuId\":\"16606002\",\"sensorUuId\":\"S0PT0009\",\"newTotal\":1234567890}";

	char	checksum_msb = 0;
	char	checksum_lsb = 0;

		
	// --- START OF CODE -----------------------------------------------

	

	if(validate_checksum(str_off_line_query) == 0) 	{
		printf("str_off_line_query checksum is ok.\n");
	}
	else {
		printf("error str_off_line_query : checksum_msb = %d (0x%x), checksum_lsb = %d (0x%x)\n", checksum_msb, checksum_msb, checksum_lsb, checksum_lsb);
	}
	if(validate_checksum(str_assign_address) == 0) {
		printf("str_assign_address checksum is ok.\n");
	}
	else {
		printf("error str_assign_address : checksum_msb = %d (0x%x), checksum_lsb = %d (0x%x)\n", checksum_msb, checksum_msb, checksum_lsb, checksum_lsb);
	}
	if(validate_checksum(str_remove_address) == 0) 	{
		printf("str_remove_address checksum is ok.\n");
	}
	else {
		printf("error str_remove_address : checksum_msb = %d (0x%x), checksum_lsb = %d (0x%x)\n", checksum_msb, checksum_msb, checksum_lsb, checksum_lsb);
	}
	if(validate_checksum(str_request_info) == 0) 	{
		printf("str_request_info checksum is ok.\n");
	}
	else {
		printf("error str_request_info: checksum_msb = %d (0x%x), checksum_lsb = %d (0x%x)\n", checksum_msb, checksum_msb, checksum_lsb, checksum_lsb);
	}
	
	
	
	if(append_checksum(str_off_line_query, sizeof(str_off_line_query))) {
		printf("error in string: str_off_line_query.\n");
	}
	else {
		hexdump("str_off_line_query", str_off_line_query, sizeof(str_off_line_query));
	}
	if(append_checksum(str_assign_address, sizeof(str_assign_address))) {
		printf("error in string: str_assign_address.\n");
	}
	else {
		hexdump("str_assign_address", str_assign_address, sizeof(str_assign_address));
	}
	if(append_checksum(str_remove_address, sizeof(str_remove_address))) {
		printf("error in string: str_remove_address.\n");
	} 
	else {
		hexdump("str_remove_address", str_remove_address, sizeof(str_remove_address));
	}
	if(append_checksum(str_request_info, sizeof(str_request_info))) {
		printf("error in string: str_request_info.\n");
	}
	else {
		hexdump("str_request_info", str_request_info, sizeof(str_request_info));
	}

	// printf("string: %s - sizeof: %d\n", str_off_line_query, sizeof(str_off_line_query));
	// printf("string: %s - strlen: %d\n", str_off_line_query, strlen(str_off_line_query));

	
	// Initialize jsmn parser
	jsmn_init(&jsmn_p);

	// Read config files 
	init_config(&UBP_DATA);
	
	// Initialize wiringPi routines
	init_wiringPi(&UBP_DATA);
	
	// Clear buffers 
	bzero(UBP_DATA.str_http_header, HTTP_HEADER_SIZE-1);
	bzero(UBP_DATA.str_http_payload, HTTP_PAYLOAD_SIZE-1);
	bzero(UBP_DATA.str_http_request, HTTP_REQUEST_SIZE-1);
	bzero(UBP_DATA.str_tty_response, TTY_BUFF_SIZE-1);
	
//  RELATED TO S0 INTERRUPT LINES (code from S0 project)
//	if (UBP_DATA.sensor[1].uint_sensor_enabled == 1)
//	{
//		printf("Enabling interrupt for GPIO_PIN_23\n");
//		pullUpDnControl(GPIO_PIN_23, PUD_UP);
//		wiringPiISR(GPIO_PIN_23, INT_EDGE_FALLING, &call_back_wiringPi_GPIO_PIN_23);
//	}
//	/if (UBP_DATA.sensor[2].uint_sensor_enabled == 1)
//	{
//		printf("Enabling interrupt for GPIO_PIN_24\n");
//		pullUpDnControl(GPIO_PIN_24, PUD_UP);
//		wiringPiISR(GPIO_PIN_24, INT_EDGE_FALLING, &call_back_wiringPi_GPIO_PIN_24);
//	}
//	if (UBP_DATA.sensor[3].uint_sensor_enabled == 1)
//	{
//		printf("Enabling interrupt for GPIO_PIN_25\n");
//		pullUpDnControl(GPIO_PIN_25, PUD_UP);
//		wiringPiISR(GPIO_PIN_25, INT_EDGE_FALLING, &call_back_wiringPi_GPIO_PIN_25);
//	}

	printf("\n");

	printf("----------------------------------------------\n");
	printf(" Upp Box Pro GoodWe RS485 BUS Interface v0.1  \n");
	printf("----------------------------------------------\n\n");

	
	// Flash LED's to indicate startup	
	flash_LEDs();

	printf("Initializing all sensor id strings \n");
	retval = itable_initialize(&UBP_DATA);
	printf("UBP_DATA.sensor[0].str_ini_sensor_id = %s\n", UBP_DATA.sensor[0].str_ini_sensor_id);
	printf("UBP_DATA.sensor[1].str_ini_sensor_id = %s\n", UBP_DATA.sensor[1].str_ini_sensor_id);
	printf("UBP_DATA.sensor[2].str_ini_sensor_id = %s\n", UBP_DATA.sensor[2].str_ini_sensor_id);
	printf("UBP_DATA.sensor[3].str_ini_sensor_id = %s\n", UBP_DATA.sensor[3].str_ini_sensor_id);
	printf("UBP_DATA.sensor[4].str_ini_sensor_id = %s\n", UBP_DATA.sensor[4].str_ini_sensor_id);
	printf("UBP_DATA.sensor[5].str_ini_sensor_id = %s\n", UBP_DATA.sensor[5].str_ini_sensor_id);
	printf("UBP_DATA.sensor[6].str_ini_sensor_id = %s\n", UBP_DATA.sensor[6].str_ini_sensor_id);
	printf("UBP_DATA.sensor[7].str_ini_sensor_id = %s\n", UBP_DATA.sensor[7].str_ini_sensor_id);
	printf("UBP_DATA.sensor[8].str_ini_sensor_id = %s\n", UBP_DATA.sensor[8].str_ini_sensor_id);
	printf("UBP_DATA.sensor[9].str_ini_sensor_id = %s\n", UBP_DATA.sensor[9].str_ini_sensor_id);  

	
	
	// --- MAIN LOOP ---------------------------------------------------	

	
	while (1)
	{
		switch (state_machine)
		{
			case STATE_INIT_TTY:										// Initialize state machine

				tty_fd = ttyOpen("/dev/ttyUSB0", 9600);
				if (tty_fd == -1)
				{
					printf("ERROR connecting to /dev/ttyUSB0 \n");
					digitalWrite (LED_RED, HIGH) ;						// LED ERROR ON
					sleep(1);
					digitalWrite (LED_RED, LOW) ;						// LED ERROR OFF
					sleep(2);
				}
				else
				{
					printf("Device /dev/ttyUSB0 connected on fd: %d \n", tty_fd);
					//state_machine = STATE_OFF_REMOVE_ALL;
					state_machine = STATE_OFF_LINE_QUERY;
				}
				break; 

			case STATE_OFF_LINE_QUERY:									// Send: off_line_query
			
				printf("<< STATE_OFF_LINE_QUERY:\n");
				hexdump("sending str_off_line_query", str_off_line_query, sizeof(str_off_line_query));
				sleep(2);
				write(tty_fd, str_off_line_query, sizeof(str_off_line_query)); 
				printf(">> STATE_REGISTER_REQUEST:\n");
				state_machine = STATE_REGISTER_REQUEST;
				break;


			case STATE_REGISTER_REQUEST:								// Receive: register request
				
				retval = read(tty_fd, &tty_char, 1);
				if(retval > 0) 
				{
					retval = rx_tty_register_request(&UBP_DATA, tty_char);
					if(retval >= 0)
					{
						hexdump("> RECEIVED - tty_register_request:", 
						     &UBP_DATA.str_tty_response, 
						      sizeof(UBP_DATA.str_tty_response));

						// Check if source address == 0x7F
						// 0x7F is specifically reserved for unregistered inverters
						if (UBP_DATA.str_tty_response[2] != 0x7F)
						{
							printf("Incorrect source address for unregistered inverter... \n");
							state_machine = STATE_OFF_LINE_QUERY;
						}
						else
						{
							// Copy the serial number
							strncpy(UBP_DATA.new_serial_number, &UBP_DATA.str_tty_response[7], 16);
							strcpy(UBP_DATA.sensor[0].serial_number, "ABCDEFGHIJKLMOPQ");

							printf("new_sn : %s\n", UBP_DATA.new_serial_number);
							printf("sn[0] : %s\n", UBP_DATA.sensor[0].serial_number);
						
							state_machine = STATE_ASSIGN_ADDRESS;

						}
					}
				}
				else
				{
					// A time-out has occured
					// No new (unregistered) inverteres found
					printf("No new (unregistered) inverteres found...\n");
					// There are no new inverters found
					state_machine = STATE_REQUEST_INFO;
				}
				break;
				

			case STATE_ASSIGN_ADDRESS:									// Wait for ttyUSB data

				printf("<< STATE_ASSIGN_ADDRESS:\n");
				hexdump("sending str_assign_address", str_assign_address, sizeof(str_assign_address));
				sleep(3);
				// ToDo copy serial number from register request
				// ToDo assign a real address from address table
				write(tty_fd, str_assign_address, sizeof(str_assign_address)); 
				printf(">> STATE_CONFIRM_ADDRESS:\n");
				state_machine = STATE_CONFIRM_ADDRESS;
				break;


			case STATE_CONFIRM_ADDRESS:

				retval = read(tty_fd, &tty_char, 1);
				if(retval > 0) 
				{
					retval = rx_tty_confirm_address(&UBP_DATA, tty_char);
					if(retval >= 0)
					{
						// ToDo store serial number in the inverter address table
						// ToDo assign a real address from address table
						// hexdump("> RECEIVED - tty_confirm address:", &UBP_DATA.str_tty_response, sizeof(UBP_DATA.str_tty_response));
						state_machine = STATE_REQUEST_INFO;
					}
				}
				else
				{
					// printf("Serial line read time-out occured...\n");
				}
				break;


			case STATE_REQUEST_INFO:
			
				printf("<< STATE_REQUEST_INFO:\n");
				// ToDo REQUEST INFO FROM ALL ASSIGNED INVERTERS (IMPLEMENT LOOP)
				hexdump("sending str_request_info", str_request_info, sizeof(str_request_info));
				sleep(3);
				write(tty_fd, str_request_info, sizeof(str_request_info)); 
				printf(">> STATE_RESPONSE_INFO:\n");
				state_machine = STATE_RESPONSE_INFO;
				break;


			case STATE_RESPONSE_INFO:
			
				retval = read(tty_fd, &tty_char, 1);
				if(retval > 0) 
				{
					retval = rx_tty_response_info(&UBP_DATA, tty_char);
					if(retval >= 0)
					{
						// hexdump("> RECEIVED - tty_response_info:", &UBP_DATA.str_tty_response, sizeof(UBP_DATA.str_tty_response));
						state_machine = STATE_WAIT_INTERVAL;
					}
				}
				else
				{
					// printf("Serial line read time-out occured...\n");
				}
				break;

			case STATE_WAIT_INTERVAL:

				printf(">< STATE_WAIT_INTERVAL:\n");
				sleep(3);
				state_machine = STATE_OFF_LINE_QUERY;
				break;


			case STATE_OFF_REMOVE_ALL:									// Send: remove all existing connections

				printf("Sending disconnect signal to all inverters.\n");
				// Request remove address:   header(2),  orig, dest, CC,   FC,   LEN,  checksum(2)    
				// char str_remove_address[] = {0xAA, 0x55, 0x88, 0xAD, 0x00, 0x02, 0x00, 0x02, 0x36};
				str_remove_address[3] = 0x02;
				hexdump("BEFORE str_remove_address", str_remove_address, sizeof(str_remove_address));
				append_checksum(str_remove_address, sizeof(str_remove_address));
				hexdump("AFTER str_remove_address", str_remove_address, sizeof(str_remove_address));
				write(tty_fd, str_remove_address, sizeof(str_remove_address)); 
				// sleep(1);
				str_remove_address[3] = 0x01;
				append_checksum(str_remove_address, sizeof(str_remove_address));
				write(tty_fd, str_remove_address, sizeof(str_remove_address)); 
				//sleep(1);
				str_remove_address[3] = 0x02;
				append_checksum(str_remove_address, sizeof(str_remove_address));
				write(tty_fd, str_remove_address, sizeof(str_remove_address)); 
				//sleep(1);
				str_remove_address[3] = 0x03;
				append_checksum(str_remove_address, sizeof(str_remove_address));
				write(tty_fd, str_remove_address, sizeof(str_remove_address)); 
				//sleep(1);
				str_remove_address[3] = 0x04;
				append_checksum(str_remove_address, sizeof(str_remove_address));
				write(tty_fd, str_remove_address, sizeof(str_remove_address)); 
				//sleep(1);
				str_remove_address[3] = 0x05;
				append_checksum(str_remove_address, sizeof(str_remove_address));
				write(tty_fd, str_remove_address, sizeof(str_remove_address)); 
				//sleep(1);
				str_remove_address[3] = 0x06;
				append_checksum(str_remove_address, sizeof(str_remove_address));
				write(tty_fd, str_remove_address, sizeof(str_remove_address)); 
				//sleep(1);
				str_remove_address[3] = 0x07;
				append_checksum(str_remove_address, sizeof(str_remove_address));
				write(tty_fd, str_remove_address, sizeof(str_remove_address)); 
				//sleep(1);
				str_remove_address[3] = 0x08;
				append_checksum(str_remove_address, sizeof(str_remove_address));
				write(tty_fd, str_remove_address, sizeof(str_remove_address)); 
				//sleep(1);
				str_remove_address[3] = 0x09;
				append_checksum(str_remove_address, sizeof(str_remove_address));
				write(tty_fd, str_remove_address, sizeof(str_remove_address)); 
				//sleep(1);
				
				state_machine = STATE_OFF_LINE_QUERY;
				
				break;

							
/*************************************************************************/							
							
							
			case STATE_SEND_HTTP:										// Send HTTP

				create_http_payload(&UBP_DATA);							// Create HTTP POST payload
				
				// --- CREATE HTTP HEADER --------------------------------------------
				// POST / HTTP/1.1
				// User-Agent: devicexs-agent/1.0
				// Host: localhost:8080
				// Accept: */*
				// Content-Type: application/json
				// Content-Length: 554	
				// ------------------------------------------------------------------

				sprintf(UBP_DATA.str_http_header, "POST %s%s HTTP/1.1\r\nUser-Agent: devicexs-agent/1.0\r\nHost: uppenergy.com\r\nAccept: */*\r\nContent-Type: application/json\r\nContent-Length: %d\r\n\r\n", UBP_DATA.str_ini_directory, UBP_DATA.str_ini_upp_box_id, strlen(UBP_DATA.str_http_payload));
				sprintf(UBP_DATA.str_http_request, "%s%s", UBP_DATA.str_http_header, UBP_DATA.str_http_payload);

				printf("\n>> HTTP HEADER:\n%s\n", UBP_DATA.str_http_header);
				printf("\n>> HTTP PAYLOAD:\n%s\n", UBP_DATA.str_http_payload);
				//printf("\n>> HTTP REQUEST:\n%s\n", UBP_DATA.str_http_request);
				
				// Create new TCP socket
				http_fd = socket(AF_INET, SOCK_STREAM, 0);
				if(http_fd < 0) 
				{
				  perror("HTTP ERROR: Opening socket");
				  close(http_fd);
				  state_machine = STATE_SLEEP_LED_OFF;
				  break;
				}
				else
				{
					printf("HTTP: file descriptor = %d\n", http_fd);
				}
				
				// Resolve backend_server hostname
				backend_server = gethostbyname(UBP_DATA.str_ini_hostname);
				if (backend_server == NULL) 
				{
				  fprintf(stderr,"HTTP ERROR: No such host\n");
				  close(http_fd);
				  state_machine = STATE_SLEEP_LED_OFF;
				  break;
				}

				// Create backend_server address
				bzero((char *) &backend_addr, sizeof(backend_addr));
				backend_addr.sin_family = AF_INET;
				bcopy((char *)backend_server->h_addr, (char *)&backend_addr.sin_addr.s_addr, backend_server->h_length);
				backend_addr.sin_port = htons(UBP_DATA.int_ini_portno);

				// Connect to the HTTP backend_server 
				if (connect(http_fd, (struct sockaddr*)&backend_addr, sizeof(backend_addr)) < 0) 
				{
				  //fprintf(stderr,"ERROR: Connecting\n");	
				  printf("HTTP ERROR: Connecting\n");	
				  close(http_fd);
				  state_machine = STATE_SLEEP_LED_OFF;
				  break;
				}

				// Send message to the HTTP backend_server 
				retval = send(http_fd, UBP_DATA.str_http_request, strlen(UBP_DATA.str_http_request), 0);
				if (retval == 0) 
				{
				  perror("HTTP ERROR writing to socket");
				  //printf("ERROR: Writing to socket\n");
				  close(http_fd);
				  state_machine = STATE_SLEEP_LED_OFF;
				  break;
				}
				else
				{
					//printf("HTTP WRITE: wrote %d bytes to socket\n", retval);
					digitalWrite (LED_YELLOW, HIGH) ;					// LED HTTP ON
				}

				// Read backend_server response 
				bzero(UBP_DATA.str_http_response, HTTP_RESPONSE_SIZE-1);
				retval = recv_to(http_fd, UBP_DATA.str_http_response, HTTP_RESPONSE_SIZE, 0, RECV_TIME_OUT);
				
				// Analyse recv response
				if (retval == -1) 
				{
					perror("HTTP RECV error: ");
				}
				else if (retval == -2) 
				{
					iCount_TIMEOUT++;
					printf("HTTP TIME-OUT while waiting for a response.\n");
				}
				else
				{
					// Copy the first 16 char from HTTP response
					//strncpy(str_compare, UBP_DATA.str_http_response, 16);
					//printf("\n>> SERVER RESPONSE:\n%s\n", str_compare);
					printf("\n>> HTTP SERVER RESPONSE:\n%s\n", UBP_DATA.str_http_response);
				}

				// Close socket 
				close(http_fd);										// After send/receive always close
				
				state_machine = STATE_HTTP_RESPONSE;

				break;
				
			case STATE_HTTP_RESPONSE:									// Respond to HTTP response
			
				// Compare first x bytes of HTTP response 
				// Check for "HTTP/1.1 200 OK"
				strcpy(str_compare, "HTTP/1.1 200 OK");
				if(memcmp(str_compare, UBP_DATA.str_http_response, strlen(str_compare)) != 0)
				{
					digitalWrite(LED_RED, HIGH);						// LED RED HTTP ERROR response ON  
					iCount_HTTP_NOK++;
					printf("\n>> HTTP STATISTICS:\n");				
					printf("HTTP OK = %d, HTTP NOK = %d, HTTP TO = %d, MB TO = %d\n\n", iCount_HTTP_OK, iCount_HTTP_NOK, iCount_TIMEOUT, iCount_MBTIMEOUT);
					state_machine = STATE_SLEEP_LED_OFF;
					break;												// NOT 200 OK
				}
				else
				{
					iCount_HTTP_OK++;
				    printf("\n>> HTTP STATISTICS:\n");				
				    printf("HTTP OK = %d, HTTP NOK = %d, HTTP TO = %d, MB TO = %d\n\n", iCount_HTTP_OK, iCount_HTTP_NOK, iCount_TIMEOUT, iCount_MBTIMEOUT);
				}
		
		
				// Check for "measurement stored" and break 
				strcpy(str_compare, "measurement stored");
				pJSON_payload = strstr(UBP_DATA.str_http_response, str_compare);
				if(pJSON_payload != NULL) 
				{
				  state_machine = STATE_SLEEP_LED_OFF;
				  break;
				}
			
				// Check for "newSensorTotal"
				strcpy(str_compare, "newSensorTotal");
				pJSON_payload = strstr(UBP_DATA.str_http_response, str_compare);
				// if "newSensorTotal" is not found exit
				if (pJSON_payload == NULL) 
				{
				  state_machine = STATE_SLEEP_LED_OFF;
				  break;
				}

				// Check for the correct <UppBoxID> value 
				pJSON_payload = strstr(UBP_DATA.str_http_response, UBP_DATA.str_ini_upp_box_id);
				// if correct UppBoxID is found continue
				if (pJSON_payload == NULL) 
				{
				  state_machine = STATE_SLEEP_LED_OFF;
				  break;
				}

				// Locate payload by checking for "\r\n\r\n"
				strcpy(str_compare, "\r\n\r\n");
				pJSON_payload = strstr(UBP_DATA.str_http_response, str_compare);
				
				// if "\r\n\r\n is found --> start of payload
				if (pJSON_payload == NULL) 
				{
				  state_machine = STATE_SLEEP_LED_OFF;
				  break;
				}
				else
				{
					pJSON_payload = pJSON_payload + 4;					// Increment pointer to skip "\r\n\r\n"
					printf(">> PAYLOAD: %s\n", pJSON_payload);
				}
				 
				// INSERT FAKE JSON PAYLOAD
				// strcpy(pJSON_payload, JSON_STRING);
				// printf("NEW JSON string: \n%s\n", JSON_STRING);

				// Now parse the response payload string through jsmn json parser
				jsmn_tokens = jsmn_parse(&jsmn_p, pJSON_payload, strlen(pJSON_payload), jsmn_token_array, sizeof(jsmn_token_array)/sizeof(jsmn_token_array[0]));
				if (jsmn_tokens < 0) 
				{
					if (jsmn_tokens == -1) printf("Error parsing JSON string: %d ==> Not enough tokens were provided\n", jsmn_tokens);
					if (jsmn_tokens == -2) printf("Error parsing JSON string: %d ==> Invalid character inside JSON string\n", jsmn_tokens);
					if (jsmn_tokens == -3) printf("Error parsing JSON string: %d ==> The string is not a full JSON packet\n", jsmn_tokens);
					state_machine = STATE_SLEEP_LED_OFF;
					break;
				}
				else
				{
					printf("Number of JSON tokens received: %d\n", jsmn_tokens -1);
				}

				/* Assume the top-level element is an object */
				if (jsmn_tokens < 1 || jsmn_token_array[0].type != JSMN_OBJECT) 
				{
					printf("Not a valid JSON string \n");
					state_machine = STATE_SLEEP_LED_OFF;
					break;
				}

				// Store tokens in separate strings
				sprintf(str_jsmn_t1, "%.*s", jsmn_token_array[1].end-jsmn_token_array[1].start, pJSON_payload + jsmn_token_array[1].start);
				sprintf(str_jsmn_t2, "%.*s", jsmn_token_array[2].end-jsmn_token_array[2].start, pJSON_payload + jsmn_token_array[2].start);
				sprintf(str_jsmn_t3, "%.*s", jsmn_token_array[3].end-jsmn_token_array[3].start, pJSON_payload + jsmn_token_array[3].start);
				sprintf(str_jsmn_t4, "%.*s", jsmn_token_array[4].end-jsmn_token_array[4].start, pJSON_payload + jsmn_token_array[4].start);
				sprintf(str_jsmn_t5, "%.*s", jsmn_token_array[5].end-jsmn_token_array[5].start, pJSON_payload + jsmn_token_array[5].start);
				sprintf(str_jsmn_t6, "%.*s", jsmn_token_array[6].end-jsmn_token_array[6].start, pJSON_payload + jsmn_token_array[6].start);
				sprintf(str_jsmn_t7, "%.*s", jsmn_token_array[7].end-jsmn_token_array[7].start, pJSON_payload + jsmn_token_array[7].start);
				sprintf(str_jsmn_t8, "%.*s", jsmn_token_array[8].end-jsmn_token_array[8].start, pJSON_payload + jsmn_token_array[8].start);
				
				lTotal = strtol(str_jsmn_t8, &ptr, 10);

				printf("JSON object pair 1: %s : %s\n", str_jsmn_t1, str_jsmn_t2);
				printf("JSON object pair 2: %s : %s\n", str_jsmn_t3, str_jsmn_t4);
				printf("JSON object pair 3: %s : %s\n", str_jsmn_t5, str_jsmn_t6);
				printf("JSON object pair 4: %s : %s\n", str_jsmn_t7, str_jsmn_t8);
				printf("New total for sensor %s = %ld\n\n", str_jsmn_t6, lTotal);
			
				// Look for matching sensor ID and update new total for that sensor
				for (i = 1; i < MAX_NUMBER_OF_INVERTERS; i++)
				{
					printf("Comparing against sensor id's from config.ini: %s\n", UBP_DATA.sensor[i].str_ini_sensor_id);
					if (strcmp(str_jsmn_t6, UBP_DATA.sensor[i].str_ini_sensor_id) == 0)
					{
						// Update calculated total
						UBP_DATA.sensor[i].total_energy = (double) lTotal;
						
						// Update respective global totals
						if (i == 1) UBP_DATA.sensor[1].total_energy = (double) lTotal;
						if (i == 2) UBP_DATA.sensor[2].total_energy = (double) lTotal;
						if (i == 3) UBP_DATA.sensor[3].total_energy = (double) lTotal;
						
						// Notify with update to screen
						printf(">> UPDATING sensor \"%s\" total to %d\n\n", UBP_DATA.sensor[i].str_ini_sensor_id, UBP_DATA.sensor[i].total_energy);
						
						break;
					}
				}
				
				state_machine = STATE_SLEEP_LED_OFF;

				break;

			case STATE_SLEEP_LED_OFF:									// Sleep 1 sec and turm LED's off
				
				delay(LED_FLASH);										// Delay for LED's to be visible
				
				digitalWrite (LED_GREEN, LOW);							// LED TTY OFF
				digitalWrite (LED_YELLOW, LOW);							// LED HTTP OFF
				digitalWrite (LED_RED, LOW);							// LED ERROR OFF

				//state_machine = STATE_READ_TTY;
				state_machine = STATE_REGISTER_REQUEST;
				
				sleep(3);
				
				break;													// NOT 200 OK
				
			default:
			
				printf("ERROR - default STATE MACHINE \n");
				break;
		}	
	}
}


// ---------------------------------------------------------------------------- 
// ---------------------------------------------------------------------------- 
// ---------------------------------------------------------------------------- 
// ---------------------------------------------------------------------------- 
// ---------------------------------------------------------------------------- 
// ---------------------------------------------------------------------------- 
// ---------------------------------------------------------------------------- 
// ---------------------------------------------------------------------------- 


/*
 * itable_find_entry:
 * Find an entry in the table for a new Inverter 
 *********************************************************************************
 * First go through the table and look for already registered serial numbers
 * If not yet registered then find the first empty entry in the table
 */


uint8_t	itable_find_entry(t_UBP_DATA *pUBP_DATA)
{
	uint8_t entry = 0;
	int retval = 0;
		
	// First scan full table for already existing entries based on serial number
	for(entry = 0; entry < MAX_NUMBER_OF_INVERTERS; entry++)
	{
		// Only compare the entries that are 'registered' 
		if(pUBP_DATA->sensor[entry].registered != 0)
		{
			// Do the actual string comparrison
			retval = strcmp(pUBP_DATA->sensor[entry].serial_number,  pUBP_DATA->new_serial_number);
			if(retval == 0)
			{
				exit(entry);	// Return table entry if found
			}
		}
	}

	// Find first empty entry 
	for(entry = 0; entry < MAX_NUMBER_OF_INVERTERS; entry++)
	{
		// Look for first 'empty' entry that is not yet registered  
		if(pUBP_DATA->sensor[entry].registered == 0)
		{
			exit(entry);	// Return table entry if found
		}
	}

	// Table is full 
	exit(-1);  	// Return error
}


/*
 * itable_add_entry:
 * Create a new entry in the table for a new Inverter 
 *********************************************************************************
 * Copy the serial number into the new (or already existing) table entry
 * Set table entry to registered
 * Clear all other variables
 */


uint8_t	itable_add_entry(t_UBP_DATA *pUBP_DATA, uint8_t entry)
{
	// Initialize all the variables
	pUBP_DATA->sensor[entry].registered = 1;
	pUBP_DATA->sensor[entry].timeout = 0;
	pUBP_DATA->sensor[entry].average_power = 0;
	pUBP_DATA->sensor[entry].total_energy = 0;
	
	// Copy serial number string 
	bincpyx(pUBP_DATA->sensor[entry].serial_number,  pUBP_DATA->new_serial_number, SN_BUFF_SIZE);

	exit(0);  	// Return 0
}



/*
 * itable_delete_entry:
 * Clear a specific table entry (inverter) 
 *********************************************************************************
 */


uint8_t itable_delete_entry(t_UBP_DATA *pUBP_DATA, uint8_t entry)
{

	if(entry >= MAX_NUMBER_OF_INVERTERS)
	{
		exit(-1);		// inidicate error
	}
	else
	{
		pUBP_DATA->sensor[entry].registered = 0;
		pUBP_DATA->sensor[entry].timeout = 0;
		pUBP_DATA->sensor[entry].average_power = 0;
		pUBP_DATA->sensor[entry].total_energy = 0;
		bzero(pUBP_DATA->sensor[entry].serial_number, SN_BUFF_SIZE);
		
		// Never clear the identifier and sensor entries
	}
	exit(0);
}


/*
 * itable_initialize:
 * Clear a specific table entry (inverter) 
 *********************************************************************************
 */


uint8_t itable_initialize(t_UBP_DATA *pUBP_DATA)
{
	uint8_t entry = 0;

	// Reset main parameters
	pUBP_DATA->new_entry_number = 0;
	bzero(pUBP_DATA->new_serial_number, SN_BUFF_SIZE);
	 
	// Initialize all variables in the table
	for (entry = 0; entry < MAX_NUMBER_OF_INVERTERS; entry++)
	{
		pUBP_DATA->sensor[entry].registered = 0;
		pUBP_DATA->sensor[entry].identifier = entry + 0x30; 	// Hex offset to ASCII '0'
		pUBP_DATA->sensor[entry].timeout = 0;
		pUBP_DATA->sensor[entry].average_power = 0;
		pUBP_DATA->sensor[entry].total_energy = 0;

		bzero(pUBP_DATA->sensor[entry].serial_number, SN_BUFF_SIZE);
		bzero(pUBP_DATA->sensor[entry].str_ini_sensor_id, SENSOR_BUFF_SIZE);
		
		// Generate the sensor ID's for all table entries
		strcpy(pUBP_DATA->sensor[entry].str_ini_sensor_id, pUBP_DATA->str_ini_sensor_id);
		// bincpyx(pUBP_DATA->sensor[entry].str_ini_sensor_id, pUBP_DATA->str_ini_sensor_id, 8);
				
		// Update last char of the sensor id pUBP_DATA->sensor[entry].str_ini_sensor_id
		pUBP_DATA->sensor[entry].str_ini_sensor_id[7] = (char) (entry + 0x30);  // Add ASCII offset
	}
	
	return 0;
}




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





/*
 * rx_tty_register_request:
 * Read all the datafields from the P1 telegram.
 *********************************************************************************
 */

int	rx_tty_register_request(t_UBP_DATA *pUBP_DATA, int tty_data)
{
	int				retval = -1;
	static int		tty_state_machine = STATE_TTY_HEADER_AA;
	static int		pntr_buffer_tty = 0;

	//static int		int_control_code = -1;
	//static int		int_function_code = -1;
	static int		int_length = -1;
	//static int		int_checksum_msb = -1;
	//static int		int_checksum_lsb = -1;	
	
	// 	AP 	-->	INV 	off_line_query 		FC=0x00, AP=0x88, INV=0x7F
	//  header(2),  orig, dest, CC,   FC,   LEN,  checksum(2) 
	//	0xAA, 0x55, 0x88, 0x7F, 0x00, 0x00, 0x00, ...... , 0xAB, 0xCD
	
	switch (tty_state_machine)
	{
		case STATE_TTY_HEADER_AA:										// check for Goodwe header 1

			if (tty_data == (int) 0xAA)									// GoodWe header1 0xAA
			{
				pntr_buffer_tty = 0;
				bzero(pUBP_DATA->str_tty_response, TTY_BUFF_SIZE);
				pUBP_DATA->str_tty_response[pntr_buffer_tty] = tty_data;
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
				pUBP_DATA->str_tty_response[pntr_buffer_tty] = tty_data;
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

			pUBP_DATA->str_tty_response[pntr_buffer_tty] = tty_data;	// store byte
			pntr_buffer_tty++;
			printf("Received originator address: 0x%x \n", tty_data);
			tty_state_machine = STATE_TTY_DEST_ADDR;
			break;

		case STATE_TTY_DEST_ADDR: 										// continue reading 

			// Correct receipient address (addressed to me?)
			if(tty_data != MY_ADDRESS)
			{
				printf("Received incorrect destination address: 0x%x \n", tty_data);
				tty_state_machine = STATE_TTY_HEADER_AA;
				retval = -3;											// Incorrect data received
			}
			else
			{
				pUBP_DATA->str_tty_response[pntr_buffer_tty] = tty_data;// store byte 
				pntr_buffer_tty++;
				printf("Received destination address: 0x%x \n", tty_data);
				tty_state_machine = STATE_TTY_CONTROL_CODE;
			}
			break;

		case STATE_TTY_CONTROL_CODE: 									// continue reading 

			// Check control code == 0x00
			if(tty_data != (int) 0x00) 
			{
				printf("Received incorrect control code: 0x%x \n", tty_data);
				tty_state_machine = STATE_TTY_HEADER_AA;
			}
			else
			{
				//int_control_code = tty_data;
				pUBP_DATA->str_tty_response[pntr_buffer_tty] = tty_data;// store byte 
				pntr_buffer_tty++;
				printf("Received control code: 0x%x \n", tty_data);
				tty_state_machine = STATE_TTY_FUNCTION_CODE;
			}
			break;

		case STATE_TTY_FUNCTION_CODE: 									// continue reading 

			// Check function code = 0x80
			if(tty_data != (int) 0x80) 
			{
				printf("Received incorrect function code: 0x%x \n", tty_data);
				tty_state_machine = STATE_TTY_HEADER_AA;
			}
			else
			{
				//int_function_code = tty_data;
				pUBP_DATA->str_tty_response[pntr_buffer_tty] = tty_data;// store byte 
				pntr_buffer_tty++;
				printf("Received function code: 0x%x \n", tty_data);
				tty_state_machine = STATE_TTY_LENGTH;
			}
			break;

		case STATE_TTY_LENGTH: 											// continue reading 

			// Check validity of length field (16 char payload expected)
			if(tty_data != (int) 0x10)
			{
				printf("Received incorrect length: 0x%x \n", tty_data);
				tty_state_machine = STATE_TTY_HEADER_AA;
			}
			else
			{
				int_length = tty_data;									// copy length for next state
				pUBP_DATA->str_tty_response[pntr_buffer_tty] = tty_data;// store byte 
				pntr_buffer_tty++;
				printf("Received length field: 0x%x \n", int_length);
				tty_state_machine = STATE_TTY_PAYLOAD;
			}
			break;

		case STATE_TTY_PAYLOAD: 										// Reading multiple payload data bytes 

				pUBP_DATA->str_tty_response[pntr_buffer_tty] = tty_data;// store data bytes according to length
				pntr_buffer_tty++;
				printf("Received payload data byte: 0x%x \n", tty_data);
				int_length--;
				if(int_length == 0)
				{
					tty_state_machine = STATE_TTY_CS_MSB;
				}

				
			break;

		case STATE_TTY_CS_MSB: 											// continue reading 

			//int_checksum_msb = tty_data;								// store byte locally
			pUBP_DATA->str_tty_response[pntr_buffer_tty] = tty_data;	// store byte in struct
			pntr_buffer_tty++;
			printf("Received checksum (msb): 0x%x \n", tty_data);
			tty_state_machine = STATE_TTY_CS_LSB;
			break;

		case STATE_TTY_CS_LSB: 											// continue reading 

			//int_checksum_lsb = tty_data;								// store byte locally
			pUBP_DATA->str_tty_response[pntr_buffer_tty] = tty_data;	// store byte 
			pntr_buffer_tty++;
			printf("Received checksum (lsb): 0x%x \n", tty_data);
			if(validate_checksum(pUBP_DATA->str_tty_response) == 0)
			{
				printf("Checksum = Ok.\n");
				tty_state_machine = STATE_TTY_HEADER_AA;				// Done and go wait for new response			
				retval = 0;												// Indicate that a complete data packet is received
			}
			else
			{
				printf("Checksum incorrect.\n");
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






/*
 * rx_tty_confirm_address:
 * Read all the datafields from the INV .
 *********************************************************************************
 */
							
int	rx_tty_confirm_address(t_UBP_DATA *pUBP_DATA, int tty_data)
{
	int				retval = -1;
	static int		tty_state_machine = STATE_TTY_HEADER_AA;
	static int		pntr_buffer_tty = 0;

	//static int		int_control_code = -1;
	//static int		int_function_code = -1;
	static int		int_length = -1;
	//static int		int_checksum_msb = -1;
	//static int		int_checksum_lsb = -1;	
	
	// 	AP 	-->	INV 	off_line_query 		FC=0x00, AP=0x88, INV=0x7F
	//  header(2),  orig, dest, CC,   FC,   LEN,  checksum(2) 
	//	0xAA, 0x55, 0x88, 0x7F, 0x00, 0x00, 0x00, ...... , 0xAB, 0xCD
	
	switch (tty_state_machine)
	{
		case STATE_TTY_HEADER_AA:										// check for Goodwe header 1

			if (tty_data == (int) 0xAA)									// GoodWe header1 0xAA
			{
				pntr_buffer_tty = 0;
				bzero(pUBP_DATA->str_tty_response, TTY_BUFF_SIZE);
				pUBP_DATA->str_tty_response[pntr_buffer_tty] = tty_data;
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
				pUBP_DATA->str_tty_response[pntr_buffer_tty] = tty_data;
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

			pUBP_DATA->str_tty_response[pntr_buffer_tty] = tty_data;	// store byte
			pntr_buffer_tty++;
			printf("Received originator address: 0x%x \n", tty_data);
			tty_state_machine = STATE_TTY_DEST_ADDR;
			break;

		case STATE_TTY_DEST_ADDR: 												// continue reading 

			// Correct receipient address (addressed to me?)
			if(tty_data != MY_ADDRESS)
			{
				printf("Received incorrect destination address: 0x%x \n", tty_data);
				tty_state_machine = STATE_TTY_HEADER_AA;
				retval = -3;											// Incorrect data received
			}
			else
			{
				pUBP_DATA->str_tty_response[pntr_buffer_tty] = tty_data;// store byte 
				pntr_buffer_tty++;
				printf("Received destination address: 0x%x \n", tty_data);
				tty_state_machine = STATE_TTY_CONTROL_CODE;
			}
			break;

		case STATE_TTY_CONTROL_CODE: 									// continue reading 

			// Check control code == 0x00
			if(tty_data != (int) 0x00) 
			{
				printf("Received incorrect control code: 0x%x \n", tty_data);
				tty_state_machine = STATE_TTY_HEADER_AA;
			}
			else
			{
				//int_control_code = tty_data;
				pUBP_DATA->str_tty_response[pntr_buffer_tty] = tty_data;// store byte 
				pntr_buffer_tty++;
				printf("Received control code: 0x%x \n", tty_data);
				tty_state_machine = STATE_TTY_FUNCTION_CODE;
			}
			break;

		case STATE_TTY_FUNCTION_CODE: 									// continue reading 

			// Check function code = 0x81
			if(tty_data != (int) 0x81) 
			{
				printf("Received incorrect function code: 0x%x \n", tty_data);
				tty_state_machine = STATE_TTY_HEADER_AA;
			}
			else
			{
				//int_function_code = tty_data;
				pUBP_DATA->str_tty_response[pntr_buffer_tty] = tty_data;// store byte 
				pntr_buffer_tty++;
				printf("Received function code: 0x%x \n", tty_data);
				tty_state_machine = STATE_TTY_LENGTH;
			}
			break;

		case STATE_TTY_LENGTH: 											// continue reading 

			// Check validity of length field (0 char payload expected)
			if(tty_data != (int) 0x00)
			{
				printf("Received incorrect length: 0x%x \n", tty_data);
				tty_state_machine = STATE_TTY_HEADER_AA;
			}
			else
			{
				int_length = tty_data;									// copy length for next state
				pUBP_DATA->str_tty_response[pntr_buffer_tty] = tty_data;// store byte 
				pntr_buffer_tty++;
				printf("Received length field: 0x%x \n", int_length);
				tty_state_machine = STATE_TTY_CS_MSB;					// NO PAYLOAD in confirm address
			}
			break;

		case STATE_TTY_PAYLOAD: 										// Reading multiple payload data bytes 

			pUBP_DATA->str_tty_response[pntr_buffer_tty] = tty_data;	// store data bytes according to length
			pntr_buffer_tty++;
			printf("Received payload data byte: 0x%x \n", tty_data);
			int_length--;
			if(int_length == 0)
			{
				tty_state_machine = STATE_TTY_CS_MSB;
			}
			break;

		case STATE_TTY_CS_MSB: 											// continue reading 

			//int_checksum_msb = tty_data;								// store byte locally
			pUBP_DATA->str_tty_response[pntr_buffer_tty] = tty_data;	// store byte in struct
			pntr_buffer_tty++;
			printf("Received checksum (msb): 0x%x \n", tty_data);
			tty_state_machine = STATE_TTY_CS_LSB;
			break;

		case STATE_TTY_CS_LSB: 											// continue reading 

			//int_checksum_lsb = tty_data;								// store byte locally
			pUBP_DATA->str_tty_response[pntr_buffer_tty] = tty_data;	// store byte 
			pntr_buffer_tty++;
			printf("Received checksum (lsb): 0x%x \n", tty_data);
			if(validate_checksum(pUBP_DATA->str_tty_response) == 0)
			{
				printf("Checksum = Ok.\n");
				tty_state_machine = STATE_TTY_HEADER_AA;				// Done and go wait for new response			
				retval = 0;												// Indicate that a complete data packet is received
			}
			else
			{
				printf("Checksum incorrect.\n");
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



/*
 * rx_tty_response_info:
 * Read all the datafields from response info.
 *********************************************************************************
 */
							
int	rx_tty_response_info(t_UBP_DATA *pUBP_DATA, int tty_data)
{
	int				retval = -1;
	static int		tty_state_machine = STATE_TTY_HEADER_AA;
	static int		pntr_buffer_tty = 0;

	//static int		int_control_code = -1;
	//static int		int_function_code = -1;
	static int		int_length = -1;
	//static int		int_checksum_msb = -1;
	//static int		int_checksum_lsb = -1;	
	
	// 	AP 	-->	INV 	off_line_query 		FC=0x00, AP=0x88, INV=0x7F
	//  header(2),  orig, dest, CC,   FC,   LEN,  checksum(2) 
	//	0xAA, 0x55, 0x88, 0x7F, 0x00, 0x00, 0x00, ...... , 0xAB, 0xCD
	
	switch (tty_state_machine)
	{
		case STATE_TTY_HEADER_AA:										// check for Goodwe header 1

			if (tty_data == (int) 0xAA)									// GoodWe header1 0xAA
			{
				pntr_buffer_tty = 0;
				bzero(pUBP_DATA->str_tty_response, TTY_BUFF_SIZE);
				pUBP_DATA->str_tty_response[pntr_buffer_tty] = tty_data;
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
				pUBP_DATA->str_tty_response[pntr_buffer_tty] = tty_data;
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

			pUBP_DATA->str_tty_response[pntr_buffer_tty] = tty_data;	// store byte
			pntr_buffer_tty++;
			printf("Received originator address: 0x%x \n", tty_data);
			tty_state_machine = STATE_TTY_DEST_ADDR;
			break;

		case STATE_TTY_DEST_ADDR: 										// continue reading 

			// Correct receipient address (addressed to me?)
			if(tty_data != MY_ADDRESS)
			{
				printf("Received incorrect destination address: 0x%x \n", tty_data);
				tty_state_machine = STATE_TTY_HEADER_AA;
				retval = -3;											// Incorrect data received
			}
			else
			{
				pUBP_DATA->str_tty_response[pntr_buffer_tty] = tty_data;// store byte 
				pntr_buffer_tty++;
				printf("Received destination address: 0x%x \n", tty_data);
				tty_state_machine = STATE_TTY_CONTROL_CODE;
			}
			break;

		case STATE_TTY_CONTROL_CODE: 									// continue reading 

			// Check control code == 0x01
			if(tty_data != (int) 0x01) 
			{
				printf("Received incorrect control code: 0x%x \n", tty_data);
				tty_state_machine = STATE_TTY_HEADER_AA;
			}
			else
			{
				//int_control_code = tty_data;
				pUBP_DATA->str_tty_response[pntr_buffer_tty] = tty_data;// store byte 
				pntr_buffer_tty++;
				printf("Received control code: 0x%x \n", tty_data);
				tty_state_machine = STATE_TTY_FUNCTION_CODE;
			}
			break;

		case STATE_TTY_FUNCTION_CODE: 									// continue reading 

			// Check function code = 0x81
			if(tty_data != (int) 0x81) 
			{
				printf("Received incorrect function code: 0x%x \n", tty_data);
				tty_state_machine = STATE_TTY_HEADER_AA;
			}
			else
			{
				//int_function_code = tty_data;
				pUBP_DATA->str_tty_response[pntr_buffer_tty] = tty_data;// store byte 
				pntr_buffer_tty++;
				printf("Received function code: 0x%x \n", tty_data);
				tty_state_machine = STATE_TTY_LENGTH;
			}
			break;

		case STATE_TTY_LENGTH: 											// continue reading 

			// Check validity of length field (0x40 bytes payload expected)
			if(tty_data != (int) 0x40)
			{
				printf("Received incorrect length: 0x%x \n", tty_data);
				tty_state_machine = STATE_TTY_HEADER_AA;
			}
			else
			{
				int_length = tty_data;									// copy length for next state
				pUBP_DATA->str_tty_response[pntr_buffer_tty] = tty_data;// store byte 
				pntr_buffer_tty++;
				printf("Received length field: 0x%x \n", int_length);
				tty_state_machine = STATE_TTY_PAYLOAD;
			}
			break;

		case STATE_TTY_PAYLOAD: 										// Reading multiple payload data bytes 

			pUBP_DATA->str_tty_response[pntr_buffer_tty] = tty_data;	// store data bytes according to length
			pntr_buffer_tty++;
			printf("Received payload data byte: 0x%x \n", tty_data);
			int_length--;
			if(int_length == 0)
			{
				tty_state_machine = STATE_TTY_CS_MSB;
			}
			break;

		case STATE_TTY_CS_MSB: 											// continue reading 

			//int_checksum_msb = tty_data;								// store byte locally
			pUBP_DATA->str_tty_response[pntr_buffer_tty] = tty_data;	// store byte in struct
			pntr_buffer_tty++;
			printf("Received checksum (msb): 0x%x \n", tty_data);
			tty_state_machine = STATE_TTY_CS_LSB;
			break;

		case STATE_TTY_CS_LSB: 											// continue reading 

			//int_checksum_lsb = tty_data;								// store byte locally
			pUBP_DATA->str_tty_response[pntr_buffer_tty] = tty_data;	// store byte 
			pntr_buffer_tty++;
			printf("Received checksum (lsb): 0x%x \n", tty_data);
			if(validate_checksum(pUBP_DATA->str_tty_response) == 0)
			{
				printf("Checksum = Ok.\n");
				tty_state_machine = STATE_TTY_HEADER_AA;				// Done and go wait for new response			
				retval = 0;												// Indicate that a complete data packet is received
			}
			else
			{
				printf("Checksum incorrect.\n");
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





/***********************************************************************
 *  
 * JSON format for single Modbus inverter 
 * 
 ***********************************************************************
 
{
"boxUuId":"16606006",
	"sensors":
	[
		{"sensorUuId":"MBPT0001", "timestamp":"2016-08-09T12:39:01.000Z", "load": 0.000000, "total": 0}
	]
}

***********************************************************************/



/*
 * create_http_payload:
 * Create HTTP payload from GoodWe data fields 
 ***********************************************************************
 */

void create_http_payload(t_UBP_DATA *pUBP_DATA)
{
	time_t t = time(NULL);							// copy local time
	struct tm tm = *gmtime(&t);
	//struct tm tm = *localtime(&t);

	// Strings for creating modbus data packet
	char strTIME[32] = "2016-05-31T15:35:47.000Z";		// Sample timestamp 24 char long
	char str_SENSOR_JSON[1024] = "{\"sensorUuId\":\"MBPT0001\", \"timestamp\":\"2016-08-09T12:39:01.000Z\", \"load\": 0.000000, \"total\": 0}";

	// Clear buffers
	bzero(strTIME, 32);
	bzero(pUBP_DATA->str_http_payload, HTTP_PAYLOAD_SIZE -1);
	
	// Create TIME string	
	sprintf(strTIME, "%04d-%02d-%02dT%02d:%02d:%02d.000Z", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);

	// Create MODBUS JSON string
	sprintf(str_SENSOR_JSON, "{\"sensorUuId\":\"MBPT0001\", \"timestamp\":\"%s\", \"load\":%d, \"total\":%d}", strTIME, pUBP_DATA->sensor[0].average_power, pUBP_DATA->sensor[0].total_energy);
	
	// Create HTTP payload
	sprintf(pUBP_DATA->str_http_payload, "{\n\"boxUuId\":\"%s\",\n\t\"sensors\":\n\t[\n\t%s\n\t]\n}\n", pUBP_DATA->str_ini_upp_box_id, str_SENSOR_JSON);
	
	// printf("JSON : \n%s\n\n", pUBP_DATA->str_http_payload);
	
}
	

// ---------------------------------------------------------------------------- 
// ---------------------------------------------------------------------------- 

	
/*
 * Receive with time-out specified by time-out
 * 
 * Params:
 *    fd       	- (int) socket file descriptor
 *    buffer 	- (char*) buffer to hold data
 *    len     	- (int) maximum number of bytes to recv()
 *    flags   	- (int) flags (as the fourth param to recv() )
 *    to       	- (int) timeout in milliseconds
 * Results:
 *    int     	- The same as recv, but -2 == TIMEOUT
 * Notes:
 *    You can only use it on file descriptors that are sockets!
 *    'to' must be different to 0
 *    'buffer' must not be NULL and must point to enough memory to hold at least 'len' bytes
 *********************************************************************************
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

	
/*
 *  init_config:
 *	Read config.ini and meter.ini 
 *********************************************************************************
 */
	
void init_config(t_UBP_DATA *pUBP_DATA)
{
	// Read config.ini
    if(ini_parse("config.ini", inih_call_back_handler, pUBP_DATA) < 0) 
    {
        printf("Can't load 'config.ini'\n");
        return;
    }
	    
    // Print config parameters
    printf("\nBOX configuration parameters read from config.ini\n");
    printf("config.ini UppBoxID    : %s\n", pUBP_DATA->str_ini_upp_box_id);
	printf("config.ini SensorID    : %s\n", pUBP_DATA->str_ini_sensor_id);
	printf("config.ini Hostname    : %s\n", pUBP_DATA->str_ini_hostname);
	printf("config.ini Directory   : %s\n", pUBP_DATA->str_ini_directory);
	printf("config.ini Port number : %d\n", pUBP_DATA->int_ini_portno);
	
}

	
/*
 *  init_wiringPi:
 *	Initialise wiringPi
 *********************************************************************************
 */

void init_wiringPi(t_UBP_DATA *pUBP_DATA)
{
    /* Initialize wiringPi library / process */ 
	wiringPiSetup ();
	
	/* Set LED pins to output */ 
	pinMode (LED_RED, OUTPUT);
	pinMode (LED_YELLOW, OUTPUT);
	pinMode (LED_GREEN, OUTPUT);
	
	/* Pull Up pin SWITCH */
	pullUpDnControl(GPIO_PIN_RESET, PUD_UP);
				
	/* Set the call back routine in wiringPi library */ 
	wiringPiISR(GPIO_PIN_RESET, INT_EDGE_FALLING, &call_back_wiringPi_GPIO_PIN_RESET);
	
}



/***********************************************************************
 * ttyOpen:
 *	Open and initialise the serial port, setting all the right
 *	port parameters - or as many as are required - hopefully!
 **********************************************************************/

int tty_open_old(const char *device)
{
	struct termios 	options;
	int     		tty_fd;

	memset(&options, 0, sizeof(options));
	options.c_iflag = 0;
	options.c_oflag = 0;
	options.c_cflag = CS7 | CREAD | CLOCAL | PARENB;  	// 7n1, see termios.h for more information
	options.c_lflag = 0;
	
	// Warning: The VMIN and VTIME flags are ignored because the O_NONBLOCK flag is set.
	options.c_cc[VMIN] = 1;
	options.c_cc[VTIME] = 5;

	printf("\n");
	printf("termios.c_iflag    : %x\n", options.c_iflag);
	printf("termios.c_oflag    : %x\n", options.c_oflag);
	printf("termios.c_cflag    : %x\n", options.c_cflag);
	printf("termios.c_lflag    : %x\n\n", options.c_lflag);

	tty_fd = open(device, O_RDWR | O_NONBLOCK);      
	
	cfsetospeed(&options, B9600);            			// 9600 baud
	cfsetispeed(&options, B9600);            			// 9600 baud

	tcsetattr(tty_fd, TCSANOW, &options);

	usleep(10000);						// 10mS

	return tty_fd;
}


/*
 * ttyOpen:
 * Open and initialise the serial port, setting all the right
 * port parameters - or as many as are required - hopefully!
 *********************************************************************************
 */

int ttyOpen(const char *device, const int baud)
{
  struct termios options;
  speed_t myBaud;
  int     status, fd;

  switch (baud)
  {
	  
    case     50:	myBaud =     B50 ; break ;
    case     75:	myBaud =     B75 ; break ;
    case    110:	myBaud =    B110 ; break ;
    case    134:	myBaud =    B134 ; break ;
    case    150:	myBaud =    B150 ; break ;
    case    200:	myBaud =    B200 ; break ;
    case    300:	myBaud =    B300 ; break ;
    case    600:	myBaud =    B600 ; break ;
    case   1200:	myBaud =   B1200 ; break ;
    case   1800:	myBaud =   B1800 ; break ;
    case   2400:	myBaud =   B2400 ; break ;
    case   4800:	myBaud =   B4800 ; break ;
    case   9600:	myBaud =   B9600 ; break ;
    case  19200:	myBaud =  B19200 ; break ;
    case  38400:	myBaud =  B38400 ; break ;
    case  57600:	myBaud =  B57600 ; break ;
    case 115200:	myBaud = B115200 ; break ;
    case 230400:	myBaud = B230400 ; break ;

    default:
      return -2;
  }

  fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);
  if (fd == -1)
  {
    return -1;
  }

  fcntl(fd, F_SETFL, O_RDWR);

  // Get and modify current options:

  tcgetattr (fd, &options);

    cfmakeraw   (&options);
    cfsetispeed (&options, myBaud);
    cfsetospeed (&options, myBaud);

    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;

    options.c_cc [VMIN]  =   0;
    options.c_cc [VTIME] =  30;				// Ten seconds (100 deciseconds)

  tcsetattr (fd, TCSANOW | TCSAFLUSH, &options);

  ioctl (fd, TIOCMGET, &status);

  status |= TIOCM_DTR;
  status |= TIOCM_RTS;

  ioctl (fd, TIOCMSET, &status);

  usleep (10000);	// 10mS

  return fd;
}



/*
 * ttyFlush:
 *	Flush the serial buffers (both tx & rx)
 *********************************************************************************
 */

void ttyFlush (const int fd)
{
  tcflush(fd, TCIOFLUSH) ;
}


/*
 * ttyClose:
 *	Release the serial port
 *********************************************************************************
 */

void ttyClose (const int fd)
{
  close(fd) ;
}


/*
 * ttyPutchar:
 *	Send a single character to the serial port
 *********************************************************************************
 */

void ttyPutchar (const int fd, const unsigned char c)
{
  write(fd, &c, 1);
}


/*
 * ttyPuts:
 *	Send a string to the serial port
 *********************************************************************************
 */

void ttyPuts(const int fd, const char *s)
{
  write(fd, s, strlen(s));
}


/*
 * ttyDataAvail:
 *	Return the number of bytes of data avalable to be read in the serial port
 *********************************************************************************
 */

int ttyDataAvail(const int fd)
{
  int result;

  if(ioctl (fd, FIONREAD, &result) == -1)
  {
    return -1;
  }
  return result;
}


/*
 * ttyGetchar:
 *	Get a single character from the serial device.
 *	Note: Zero is a valid character and this function will time-out 
 *  after 10 seconds.
 *********************************************************************************
 */

int ttyGetchar(const int fd)
{
  uint8_t x ;

  if (read(fd, &x, 1) != 1)
  {
    return -1 ;
  }
  
  return ((int)x) & 0xFF ;
}


/*
 * hexdump:
 * Hexdump("Description", *address, length)
 *********************************************************************************
 */


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



/*
 * bincpyx:
 * Copy binary data with length x
 *********************************************************************************
 */

void bincpyx(char *dest, char *src, unsigned int x)
{
	unsigned int i;
	
	// Make a binary copy of the string / array without stopping at a null value 
	for(i=0; i<x; i++)
	{
		dest[i] = src[i];
	}
}



// ---------------------------------------------------------------------
// ---------------------------------------------------------------------

void flash_LEDs()
{
	digitalWrite(LED_GREEN, HIGH) ;		// LED ON
	delay(LED_FLASH);
	digitalWrite(LED_GREEN, LOW) ;		// LED OFF
	delay(LED_FLASH);
	digitalWrite(LED_YELLOW, HIGH) ;	// LED ON
	delay(LED_FLASH);
	digitalWrite(LED_YELLOW, LOW) ;		// LED OFF
	delay(LED_FLASH);
	digitalWrite(LED_RED, HIGH) ;		// LED ON
	delay(LED_FLASH);
	digitalWrite(LED_RED, LOW) ;		// LED OFF
	delay(LED_FLASH);
}

// ---------------------------------------------------------------------
// --- END -------------------------------------------------------------
// ---------------------------------------------------------------------



/***********************************************************************
 * ttyOpen:
 *	Open and initialise the serial port, setting all the right
 *	port parameters - or as many as are required - hopefully!
 **********************************************************************

int ttyOpen(const char *device, const int baud)
{
	struct termios 	options;
	speed_t 		myBaud;
	int     		status, fd;

	switch (baud)
	{
		case     50:	myBaud =     B50 ; break;
		case     75:	myBaud =     B75 ; break;
		case    110:	myBaud =    B110 ; break;
		case    134:	myBaud =    B134 ; break;
		case    150:	myBaud =    B150 ; break;
		case    200:	myBaud =    B200 ; break;
		case    300:	myBaud =    B300 ; break;
		case    600:	myBaud =    B600 ; break;
		case   1200:	myBaud =   B1200 ; break;
		case   1800:	myBaud =   B1800 ; break;
		case   2400:	myBaud =   B2400 ; break;
		case   4800:	myBaud =   B4800 ; break;
		case   9600:	myBaud =   B9600 ; break;
		case  19200:	myBaud =  B19200 ; break;
		case  38400:	myBaud =  B38400 ; break;
		case  57600:	myBaud =  B57600 ; break;
		case 115200:	myBaud = B115200 ; break;
		case 230400:	myBaud = B230400 ; break;
		
		default: return -2 ;
	}

	// Open non-blocking file descriptor: O_RDONLY or O_WRONLY or O_RDWR
	if((fd = open(device, O_RDONLY | O_NOCTTY | O_NDELAY | O_NONBLOCK)) == -1)
	{
		return -1;
	}
	
	// Another way to set the parameters of the file descriptor
	// On Linux this command can only change the O_APPEND, O_ASYNC, O_DIRECT, O_NOATIME, and O_NONBLOCK flags
	//fcntl(fd, F_SETFL, O_RDWR);

	// Get and modify current options
	tcgetattr(fd, &options) ;			// get termio configuration

	printf("termios.c_iflag : %x\n", options.c_iflag);
	printf("termios.c_oflag : %x\n", options.c_oflag);
	printf("termios.c_cflag : %x\n", options.c_cflag);
	printf("termios.c_lflag : %x\n", options.c_lflag);
	
	cfmakeraw(&options);				// set serial to raw mode (char by char)
	cfsetispeed(&options, myBaud);		// set input speed
	cfsetospeed(&options, myBaud);		// set output speed

	// Input flags - Turn off input processing
	// convert break to null byte, no CR to NL translation,
	// no NL to CR translation, don't mark parity errors or breaks
	// no input parity check, don't strip high bit off,
	// no XON/XOFF software flow control
	options.c_iflag = 0;
	//options.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);

    // Output flags - Turn off output processing
	// no CR to NL translation, no NL to CR-NL translation,
	// no NL to CR translation, no column 0 CR suppression,
	// no Ctrl-D suppression, no fill characters, no case mapping,
	// no local output processing
	options.c_oflag = 0;
	//options.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OLCUC | OPOST);
	
	// Set local flags
	options.c_lflag = 0;
	//options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG) ;
	
	// Set control flags
	options.c_cflag = 0;				// reset all control flags		
	//options.c_cflag |= CREAD;			// lower modem control lines after hang-up
	//options.c_cflag |= PARENB;		// enabled parity checking (even is default)
	//options.c_cflag &= ~PARODD;		// dissable odd parity --> even partity
	//options.c_cflag &= ~CSIZE;		// clear the current CSIZE mask (already done by 0)
	//options.c_cflag |= CS7;			// set char size mask to 7 bits (CS5, CS6, CS7, CS8)
	//options.c_cflag |= CLOCAL; 		// ignore modem control lines
	//options.c_cflag |= HUPCL;			// lower modem control lines after hang-up
	//options.c_cflag |= PARODD;		// enable odd parity 
	options.c_cflag = CS7 | CREAD | CLOCAL | PARENB;

	printf("\n");
	printf("termios.c_iflag    : %x\n", options.c_iflag);
	printf("termios.c_oflag    : %x\n", options.c_oflag);
	printf("termios.c_cflag    : %x\n", options.c_cflag);
	printf("termios.c_lflag    : %x\n\n", options.c_lflag);
	
	printf("termio flag CREAD  : %x\n", CREAD);
	printf("termio flag PARENB : %x\n", PARENB);
	printf("termio flag PARODD : %x\n", PARODD);
	printf("termio flag CS5    : %x\n", CS5);
	printf("termio flag CS6    : %x\n", CS6);
	printf("termio flag CS7    : %x\n", CS7);
	printf("termio flag CS8    : %x\n", CS8);
	printf("termio flag CSIZE  : %x\n", CSIZE);
	printf("termio flag CLOCAL : %x\n", CLOCAL);
	printf("termio flag HUPCL  : %x\n", HUPCL);

	// CLOCAL Ignore modem control lines.
	// CSIZE  Character size mask.  Values are CS5, CS6, CS7, or CS8.
	// CSTOPB Set two stop bits, rather than one.
	// CREAD  Enable receiver.
	// PARENB Enable parity generation on output and parity checking for input.
	// PARODD If set, then parity for input and output is odd; otherwise even parity is used.

	// The various termio flags / structs
	// tcflag_t  c_iflag     input modes
	// tcflag_t  c_oflag     output modes
	// tcflag_t  c_cflag     control modes
	// tcflag_t  c_lflag     local modes
	// cc_t      c_cc[NCCS]  control chars

	// wiringPi original settings (I think)
	//options.c_cflag |= (CLOCAL | CREAD) ;
	//options.c_cflag &= ~PARENB ;
	//options.c_cflag &= ~CSTOPB ;
	//options.c_cflag &= ~CSIZE ;
	//options.c_cflag |= CS7 ;
	//options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG) ;
	//options.c_oflag &= ~OPOST ;

	options.c_cc[VMIN]  =   0;
	options.c_cc[VTIME] = 250;			// 25 seconds (250 deciseconds)

	tcsetattr(fd, TCSANOW | TCSAFLUSH, &options);

	// Initialize DTR and RTS lines
	ioctl(fd, TIOCMGET, &status);
	status |= TIOCM_DTR;
	status |= TIOCM_RTS;
	ioctl(fd, TIOCMSET, &status);

	usleep(10000);						// 10mS

	return fd;
}
 
***********************************************************************/
