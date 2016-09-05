/********************************************************/
/* MODBUS SERVER PoC for Serial Connections				*/
/* Function Code 03 - Read Holding Registers			*/
/* Function Code 16 - Write Multiple Registers			*/
/********************************************************/
/* Author: Erwin Hendriks								*/
/* Start Date 15 JUNE 2015 - Imtech						*/
/* Update 10 AUG 2016 - AgileXS				`			*/
/********************************************************/
 
#include<stdio.h>
#include<string.h>    				// strlen
#include<sys/socket.h>
#include<arpa/inet.h> 				// inet_addr
#include<unistd.h>    				// write
 
 
// Modbus TCP header (MBAP) fields 
// Modbus (MBAP) fields are the same for REQuest and RESponse
#define		MBAP_TRANSACTION_ID_MSB			(0)
#define		MBAP_TRANSACTION_ID_LSB			(1)
#define		MBAP_PROTOCOL_ID_MSB			(2)
#define		MBAP_PROTOCOL_ID_LSB			(3)
#define		MBAP_LENGTH_IN_BYTES_MSB		(4)
#define		MBAP_LENGTH_IN_BYTES_LSB		(5)
#define		MBAP_UNIT_ID					(6)

// Modbus REQuest Packet Data Unit (PDU) fields 
#define		PDU_REQ_FUNCTION_CODE			 (7)	// Modbus PDU REQuest: Function Code is a single byte
#define		PDU_REQ_START_ADDRESS_MSB		 (8)	// Modbus PDU REQuest: Start address of first register MSB:LBS in data bank
#define		PDU_REQ_START_ADDRESS_LSB		 (9)	// Modbus PDU REQuest: Start address of first register MSB:LBS in data bank
#define		PDU_REQ_NUMB_OF_REG_MSB			(10)	// Modbus PDU REQuest: Number of registers MSB:LBS
#define		PDU_REQ_NUMB_OF_REG_LSB			(11)	// Modbus PDU REQuest: Number of registers MSB:LBS
#define		PDU_REQ_BYTE_COUNT				(12)	// Modbus PDU REQuest: Byte Count
#define		PDU_REQ_START_PAYLOAD			(13)	// Modbus PDU REQuest: Start address of payload

// Modbus RESponse Packet Data Unit (PDU) fields 
#define		PDU_RES_FUNCTION_CODE			 (7)	// Modbus PDU RESponse Function Code 
#define		PDU_RES_BYTE_COUNT				 (8)	// Modbus PDU RESponse for FC 03: Number of bytes and NOT registers
#define		PDU_RES_START_OF_DATA			 (9)	// Modbus PDU RESponse for FC 03: Start of data section 
#define		PDU_RES_START_ADDR_MSB			 (8)	// Modbus PDU RESponse for FC 16: Start Address MSB
#define		PDU_RES_START_ADDR_LSB			 (9)	// Modbus PDU RESponse for FC 16: Start Address LSB
#define		PDU_RES_NUMB_OF_REG_MSB			(10)	// Modbus PDU RESponse for FC 16: Number of bytes MSB
#define		PDU_RES_NUMB_OF_REG_LSB			(11)	// Modbus PDU RESponse for FC 16: Number of bytes LSB


 
// Function declarations 
int iModbusRequest(int, int);						// Handle Modbus request
int iFunctionCode01(int, int);						// Handle Function Code 01
int iFunctionCode02(int, int);						// Handle Function Code 02
int iFunctionCode03(int, int);						// Handle Function Code 03
int iFunctionCode04(int, int);						// Handle Function Code 04
int iFunctionCode16(int, int);						// Handle Function Code 16
int iFunctionCode23(int, int);						// Handle Function Code 23
int iGenericExceptionCode(int, int);				// Create exception code for none supported functions

void vLogMBAP(void);								// Analyse and log MBAP (Modbus TCP) header
void vInitDataBank(void);
 
// Global buffers 
char client_request[2000];			// Modbus specifies that the max size of MBAP + PDU = 260 bytes
char server_response[2000];			// Modbus specifies that the max size of MBAP + PDU = 260 bytes
char modbus_memory_map[20000];		// 20k Bytes --> Modbus memory map is 10k Words 

// For TrafficBox an address range of 625 x 16 bits data words = 10000 bits (= 1250 bytes) should be enough
 
 
int main(int argc, char *argv[])
{
    int socket_desc, client_sock, c, read_size; 
    struct sockaddr_in server, client;
	
	// Initialize Memory Map (625 words of 16 bits = 10000 bits) with arbitrary data
	vInitDataBank();
         
    // Create socket
    socket_desc = socket(AF_INET , SOCK_STREAM , 0);
    if (socket_desc == -1) 
    {
        printf("Could not create socket");
    }
    printf("Socket created: %d \n", socket_desc);
	 
    //Prepare the sockaddr_in structure
    server.sin_family = AF_INET;
    server.sin_addr.s_addr = INADDR_ANY;
    server.sin_port = htons( 502 );
     
    //Bind
    if( bind(socket_desc,(struct sockaddr *) &server, sizeof(server)) < 0) 
    {
        //print the error message
        perror("Bind failed. Error");
        return 1;
    }
    puts("Bind done ...");
     
    //Listen
    listen(socket_desc , 3);
     
    //Accept and incoming connection
    puts("Waiting for incoming connections...");
    c = sizeof(struct sockaddr_in);
     
    //accept connection from an incoming client
    client_sock = accept(socket_desc, (struct sockaddr *)&client, (socklen_t*) &c);
    if (client_sock < 0) 
    {
        perror("Accept failed ...");
        return 1;
    }
    printf("Connection accepted on: %d \n", client_sock);
		 
    // Receive a TCP message from client
    while((read_size = recv(client_sock, client_request, 2000, 0)) > 0 ) 
    {
	 
		if(read_size == 0) 
		{					// disconnect
			puts("Client disconnected ...");
			fflush(stdout);
		}
		else if(read_size == -1) 
		{
			perror("Receive failed ...");
		}

		// Handle Modbus request
		if(iModbusRequest(client_sock, read_size) != 0)	
		{
			puts("Incorrect Modbus request received...");
		}
    }
	return 0;
}


int iModbusRequest(int client_sock, int read_size)  	// New MODBUS request received in client_request
{
	int 	iFunctionCode; 			// Temp. stores the Function Code for switch
	int 	iReturnValue;			// Used to store the Response length 
	
	// Log Modbus MBAB (TCP) header from client_request 
	vLogMBAP();

	// ToDo: Check MBAP header for consistency ...
	
	// Function Code is first byte in PDU behind MBAP header 	
	iFunctionCode = client_request[PDU_REQ_FUNCTION_CODE];			// Always @ offset 7
	
	switch(iFunctionCode)
	{
		case 1 :		// Function Code 01 (0x01)
		{
			iReturnValue = iFunctionCode01(client_sock, read_size);
			break;
		}
		case 2 :		// Function Code 02 (0x02)
		{
			iReturnValue = iFunctionCode02(client_sock, read_size);
			break;
		}
		case 3 :		// Function Code 03 (0x03)
		{
			iReturnValue = iFunctionCode03(client_sock, read_size); 
			break;
		}
		case 4 :		// Function Code 04 (0x04)
		{
			iReturnValue = iFunctionCode04(client_sock, read_size);
			break;
		}
		case 16 :		// Function Code 16 (0x10)
		{
			iReturnValue = iFunctionCode16(client_sock, read_size);
			break;
		}
		case 23 :		// Function Code 23 (0x17)
		{
			iReturnValue = iFunctionCode23(client_sock, read_size);
			break;
		}
		default :		// Unsupported Function Code --> Raise exception
		{
			iReturnValue = iGenericExceptionCode(client_sock, read_size);
			break;
		}
	}
	return iReturnValue;
}



/************************************************************************/
/* MODBUS function code 01                                              */
/* Read data from memory											 	*/
/************************************************************************/

int iFunctionCode01(int client_sock, int read_size)
{
	printf("Function Code 01 (0x01) not supported \n\n");
	
	return 0;
}

/************************************************************************/
/* MODBUS function code 02                                              */
/* Read data from memory												*/
/************************************************************************/

int iFunctionCode02(int client_sock, int read_size)
{
	printf("Function Code 02 (0x02) not supported \n\n");
	return 0;
}

/************************************************************************/
/* Handle MODBUS Function Code (03) Read Holding registers 				*/
/************************************************************************/

int iFunctionCode03(int client_sock, int read_size)
{
	int					iLoop;							// Used to copy the registers in Loop
	//int	unsigned long 	iNumber = 129LU;			// For debug purposes
	
	//int	unsigned long	iStartingAddress;				// Used to calculate the starting address in request	
	unsigned int		iStartingAddress;				// Used to calculate the starting address in request	
	unsigned int		iNumberOfRegisters ;			// Used to calculate the requested number of registers
	unsigned int		iNumberOfBytesInResponse;		// Used to calculate the number of bytes in the server response
	
	// Calculate the number of bytes in the request
	iStartingAddress = ((256 * client_request[PDU_REQ_START_ADDRESS_MSB]) + client_request[PDU_REQ_START_ADDRESS_LSB]);	// Calculate starting 
	
	// Calculate the number of requested registers (max 125)
	iNumberOfRegisters = ((256 * client_request[PDU_REQ_NUMB_OF_REG_MSB]) + (client_request[PDU_REQ_NUMB_OF_REG_LSB]));

	// Check if requested number of bytes is not too long
	if (iNumberOfRegisters > 125) 
	{
		printf("ERR: Requested # of Registers > 125 ");
		return 1;
	}
	
 	// Log client_request PDU package
	printf("MODBUS - Function Code (03) - Read Multiple Registers \n");
	printf(" ------------------------------------------ \n");
	printf("PDU(07) - Function code (1 byte)    : %d \n", client_request[PDU_REQ_FUNCTION_CODE]);
	printf("PDU(08) - Start address MSB         : %d \n", client_request[PDU_REQ_START_ADDRESS_MSB]);
	printf("PDU(09) - Start address LSB  %%d     : %d \n", client_request[PDU_REQ_START_ADDRESS_LSB]);
	printf("PDU - iStartingAddress =     %%lu    : %d \n", iStartingAddress + 1);			// Plus 1 because the Modbus memory bank starts at 1 and not 0
	//printf("PDU - iStartingAddress =     %%u     : %u \n", (unsigned int) iStartingAddress);
	//printf("PDU - iStartingAddress =     %%x     : %x \n", iStartingAddress);
	//printf("Unsigned Long INT iNumber =  %%lu   : %lu \n", iNumber);
	//if (iStartingAddress = 128) printf("PDU - Starting address = 129 !!!! \n");
	printf("PDU(10) - Requested registers MSB   : %d \n", client_request[PDU_REQ_NUMB_OF_REG_MSB]);
	printf("PDU(11) - Requested registers LSB   : %d \n", client_request[PDU_REQ_NUMB_OF_REG_LSB]);  // MAX == 125 !! so why MSB field ?
	printf("PDU - Requested number of registers : %d \n", iNumberOfRegisters);
	printf("\n");
	
	// Calculate the requested number of bytes in the server response (max 125 x 2)
	iNumberOfBytesInResponse = (2 * iNumberOfRegisters);	
	
	// Construct PDU header - Function Code + Byte Count
	server_response[PDU_RES_FUNCTION_CODE] = client_request[PDU_REQ_FUNCTION_CODE];		 		// Copy Function code (single byte)
	server_response[PDU_RES_BYTE_COUNT] = (iNumberOfBytesInResponse);							// Copy Byte count + 2 (single byte)

	// Copy memory map
	for(iLoop = 0; iLoop < iNumberOfBytesInResponse; iLoop++) 
	{
		server_response[PDU_RES_START_OF_DATA + iLoop] = modbus_memory_map[iLoop + (iStartingAddress * 2)];				// Copy memory_map[x] to server_response[x]
	}
	
	// INCREMENT VALUE in REGISTER 4 (that is MEMORY LOCATION 6 & 7) !!!!!!!!!!!!!!!!
	modbus_memory_map[7]++;
	
	// Construct MBAP (TCP) header for response 
	
	// Construct MBAP header - Copy Transaction ID
	server_response[MBAP_TRANSACTION_ID_MSB] = client_request[MBAP_TRANSACTION_ID_MSB]; 		// Copy Transaction ID MSB
	server_response[MBAP_TRANSACTION_ID_LSB] = client_request[MBAP_TRANSACTION_ID_LSB];			// Copy Transaction ID LSB
	// Construct MBAP header - Copy Protocol ID 
	server_response[MBAP_PROTOCOL_ID_MSB] = client_request[MBAP_PROTOCOL_ID_MSB]; 	 			// Copy Protocol ID MSB
	server_response[MBAP_PROTOCOL_ID_LSB] = client_request[MBAP_PROTOCOL_ID_LSB];		 		// Copy Protocol ID LSB
	// Include the number of bytes in the server response TODO: use 16bit word
	server_response[MBAP_LENGTH_IN_BYTES_MSB] = 0; 					 							// Number of following bytes MSB always 0
	server_response[MBAP_LENGTH_IN_BYTES_LSB] = (iNumberOfBytesInResponse + 2); 				// Number of following bytes (max 250 + (FC + ByteCount)) 
	// Construct MBAP header - Unit ID 
	server_response[MBAP_UNIT_ID] = client_request[MBAP_UNIT_ID];		 						// Copy Unit ID (single byte)	

	
	// Log MODBUS server_response
	printf("MODBUS - Server_response \n");
	printf(" ------------------------------------------ \n");
	printf("MBAB (00) --> Transaction ID  MSB    : %d \n", server_response[0]);
	printf("MBAB (01) --> Transaction ID  LSB    : %d \n", server_response[1]);
	printf("MBAB (02) --> Protocol ID     MSB    : %d \n", server_response[2]);
	printf("MBAB (03) --> Protocol ID     LSB    : %d \n", server_response[3]);
	printf("MBAB (04) --> Number of bytes MSB    : %d \n", server_response[4]);
	printf("MBAB (05) --> Number of bytes LSB    : %d \n", server_response[5]);
	printf("MBAB (06) --> Unit ID (1 byte)       : %d \n", server_response[6]);
	
	// Construct PDU package
	printf(" ------------------------------------------ \n");
	printf("PDU  (07) --> Function code (1 byte) : %d \n", server_response[7]);
	printf("PDU  (08) --> Byte count (1 byte)    : %d \n", server_response[8]);
	
	// Log PAYLOAD data - REQuested registers
	printf(" ------------------------------------------ \n");
	
	
	// Log requested modbus registers to stdout
	for(iLoop = 0; iLoop < iNumberOfRegisters; iLoop++) 
	{
		printf("MODBUS REGISTER (%d) -->       : %d %d \n", (iStartingAddress + iLoop + 1), (unsigned int) server_response[9 + iLoop * 2], (unsigned int) server_response[10 + iLoop * 2]);
	}
	
	// Write response Modbus response to client TCP socket
	// TCP number of bytes =  iNumberOfBytesInResponse + PDU Function Code + PDU Byte Count + MBAP(always 7 byte)
	write(client_sock, server_response, iNumberOfBytesInResponse + 2 + 7);	// Send response to client
	
	// return ok... 
	return 0;  
}

/************************************************************************/
/* MODBUS function code 04                                              */
/* Read data to memory													*/
/************************************************************************/

int iFunctionCode04(int client_sock, int read_size)
{
	printf("Function Code 04 (0x04) not supported \n\n");
	return 0;
}

/************************************************************************/
/* MODBUS function code 16 (0x10) - Write Multiple Registers            */
/* Write data to memory													*/
/************************************************************************/

int iFunctionCode16(int client_sock, int read_size)
{
	int					iLoop;							// Used to copy the registers in Loop
	//int	unsigned long 	iNumber = 129LU;			// For debug purposes
	
	int	unsigned long	iStartingAddress;				// Used to calculate the starting address in request	
	unsigned int		iNumberOfRegisters ;			// Used to calculate the requested number of registers
	//unsigned int		iNumberOfBytesInResponse;		// Used to calculate the number of bytes in the server response
	
	// Calculate the number of bytes in the request
	iStartingAddress = ((256 * client_request[PDU_REQ_START_ADDRESS_MSB]) + client_request[PDU_REQ_START_ADDRESS_LSB]);	// Calculate starting 
	
	// Calculate the number of registers (MAX NUMBER OF REGISTERS IS 125)
	iNumberOfRegisters = ((256 * client_request[PDU_REQ_NUMB_OF_REG_MSB]) + (client_request[PDU_REQ_NUMB_OF_REG_LSB]));

	// Check if requested number of bytes is not too long
	if (iNumberOfRegisters > 125) 
	{
		printf("ERR: Requested # of Registers > 125 ");
		return 1;
	}
	
 	// Log client_request PDU package
	printf("MODBUS - Function Code (16) - Write Multiple Registers \n");
	printf(" ------------------------------------------ \n");
	printf("PDU(07) - Function code             : %d \n", client_request[PDU_REQ_FUNCTION_CODE]);
	printf("PDU(08) - Start address MSB         : %d \n", client_request[PDU_REQ_START_ADDRESS_MSB]);
	printf("PDU(09) - Start address LSB         : %d \n", client_request[PDU_REQ_START_ADDRESS_LSB]);
	printf("PDU(10) - Number of registers MSB   : %d \n", client_request[PDU_REQ_NUMB_OF_REG_MSB]);
	printf("PDU(11) - Number of registers LSB   : %d \n", client_request[PDU_REQ_NUMB_OF_REG_LSB]);  		// 1 - 123 --> max 0x007B 
	printf("PDU(12) - Byte Count                : %d \n", client_request[PDU_REQ_BYTE_COUNT]);  			// MAX == 250 ??? 
	printf("PDU(13) - Register 01               : %d \n", client_request[13]);  // PAYLOAD
	printf("PDU(14) - Register 01               : %d \n", client_request[14]);  // PAYLOAD
	printf("PDU(15) - Register 02               : %d \n", client_request[15]);  // PAYLOAD
	printf("PDU(16) - Register 02               : %d \n", client_request[16]);  // PAYLOAD
	printf("PDU(17) - Register 03               : %d \n", client_request[17]);  // PAYLOAD
	printf("PDU(18) - Register 03               : %d \n", client_request[18]);  // PAYLOAD
	printf(" \n etc ... \n\n ");

	// Copy client request data fields --> modbus memory map
	for(iLoop = 0; iLoop < 2 * iNumberOfRegisters; iLoop++) 
	{
		modbus_memory_map[iStartingAddress * 2 + iLoop] = client_request[PDU_REQ_START_PAYLOAD + iLoop];		// Copy memory_map[x] to server_response[x]
	}

	// Log updated memory map to stdout
	printf(" ------------------------------------------ \n");
	printf("          MEMORY BANK UPDATED               \n");
	printf(" ------------------------------------------ \n");
	for(iLoop = 0; iLoop < 50; iLoop++) 
	{
		printf("Register location: %d  --> contains data word: %d%d \n", iLoop + 1, modbus_memory_map[iLoop * 2], modbus_memory_map[iLoop * 2 + 1]);
	}
	printf("\n etc ... \n\n");
	printf(" ------------------------------------------ \n\n");
	
	
	// Construct MBAP (TCP) header for response 
	
	// Construct MBAP header - Copy Transaction ID
	server_response[MBAP_TRANSACTION_ID_MSB] = client_request[MBAP_TRANSACTION_ID_MSB]; 		// Copy Transaction ID MSB
	server_response[MBAP_TRANSACTION_ID_LSB] = client_request[MBAP_TRANSACTION_ID_LSB];			// Copy Transaction ID LSB
	// Construct MBAP header - Copy Protocol ID 
	server_response[MBAP_PROTOCOL_ID_MSB] = client_request[MBAP_PROTOCOL_ID_MSB]; 	 			// Copy Protocol ID MSB
	server_response[MBAP_PROTOCOL_ID_LSB] = client_request[MBAP_PROTOCOL_ID_LSB];		 		// Copy Protocol ID LSB
	// Include the number of bytes in the server response TODO: use 16bit word
	server_response[MBAP_LENGTH_IN_BYTES_MSB] = 0; 					 							// Number of following bytes MSB always 0
	server_response[MBAP_LENGTH_IN_BYTES_LSB] = (6); 											// Number of following bytes (always 6 = UNIT ID (1 Byte) + PDU RESponse (5 Byte)) 
	// Construct MBAP header - Unit ID 
	server_response[MBAP_UNIT_ID] = client_request[MBAP_UNIT_ID];		 						// Copy Unit ID (single byte)	

	// Construct PDU response - Function Code (1 Byte) + Start Address (2 Byte) + Number of Registers (2 Byte) 
	server_response[PDU_RES_FUNCTION_CODE] = client_request[PDU_REQ_FUNCTION_CODE];		 		// Copy Function code (single byte)
	server_response[PDU_RES_START_ADDR_MSB] = client_request[PDU_REQ_START_ADDRESS_MSB];		// Copy START ADDRESS MSB
	server_response[PDU_RES_START_ADDR_LSB] = client_request[PDU_REQ_START_ADDRESS_LSB];		// Copy START ADDRESS LSB
	server_response[PDU_RES_NUMB_OF_REG_MSB] = client_request[PDU_REQ_NUMB_OF_REG_MSB];			// Copy START ADDRESS MSB
	server_response[PDU_RES_NUMB_OF_REG_LSB] = client_request[PDU_REQ_NUMB_OF_REG_LSB];			// Copy START ADDRESS LSB

	
	// Log MODBUS server_response
	printf("MODBUS - Server_response \n");
	printf(" ------------------------------------------ \n");
	printf("MBAB (00) --> Transaction ID  MSB    : %d \n", server_response[0]);
	printf("MBAB (01) --> Transaction ID  LSB    : %d \n", server_response[1]);
	printf("MBAB (02) --> Protocol ID     MSB    : %d \n", server_response[2]);
	printf("MBAB (03) --> Protocol ID     LSB    : %d \n", server_response[3]);
	printf("MBAB (04) --> Number of bytes MSB    : %d \n", server_response[4]);
	printf("MBAB (05) --> Number of bytes LSB    : %d \n", server_response[5]);
	printf("MBAB (06) --> Unit ID (1 byte)       : %d \n", server_response[6]);
	
	// Construct PDU package
	printf(" ------------------------------------------ \n");
	printf("PDU  (07) --> Function code          : %d \n", server_response[7]);
	printf("PDU  (08) --> Starting address MSB   : %d \n", server_response[8]);
	printf("PDU  (09) --> Starting address LSB   : %d \n", server_response[9]);
	printf("PDU  (10) --> Quantity of reg. MSB   : %d \n", server_response[10]);
	printf("PDU  (11) --> Quantity of reg. LBS   : %d \n", server_response[11]);
	
	// Log PAYLOAD data - REQuested registers
	printf(" ------------------------------------------ \n");
	
	
	// Write response Modbus response to client TCP socket
	write(client_sock, server_response, 12);	// Send response to client (Function Code 16 response is always fixed length = 12 bytes
	
	return 0;	// Ok
}


/************************************************************************/
/* MODBUS function code 23 (0x17)                                       */
/* Write data to memory													*/
/************************************************************************/

int iFunctionCode23(int client_sock, int read_size)
{
	printf("Function Code 23 (0x17) not yet supported \n\n");
	return 0;
}

/************************************************************************/
/* Create Exception Code for unimplemented Function Codes				*/
/************************************************************************/

int iGenericExceptionCode(int client_sock, int read_size)
{
	printf("Raise Generic Exception Code \n\n");
	return 0;
}


/************************************************************************/
/* Log MBAP (TCP) header to stdout                                      */
/************************************************************************/

void vLogMBAP(void)
{
	// Log client_request MBAB header to stdout
	printf("\n\n\n\n");
	printf("NEW MODBUS - Client Request Received \n");
	printf(" ------------------------------------------ \n");
	printf("MBAB(00) - Transaction ID  MSB      : %d \n", client_request[MBAP_TRANSACTION_ID_MSB]);
	printf("MBAB(01) - Transaction ID  LSB      : %d \n", client_request[MBAP_TRANSACTION_ID_LSB]);
	printf("MBAB(02) - Protocol ID     MSB      : %d \n", client_request[MBAP_PROTOCOL_ID_MSB]);
	printf("MBAB(03) - Protocol ID     LSB      : %d \n", client_request[MBAP_PROTOCOL_ID_LSB]);
	printf("MBAB(04) - Length in bytes MSB      : %d \n", client_request[MBAP_LENGTH_IN_BYTES_MSB]);
	printf("MBAB(05) - Length in bytes LSB      : %d \n", client_request[MBAP_LENGTH_IN_BYTES_LSB]);
	printf("MBAB(06) - Unit ID (1 byte)         : %d \n", client_request[MBAP_UNIT_ID]);
	printf("\n");
}


/************************************************************************/
/* Initialize Memory Map with arbitrary debug data  1,2,3,...           */
/************************************************************************/

void vInitDataBank(void)
{
	int		iLoop;			// Loop counter
	
	// Initialize memory map with debug data
	// Register 400001 == 0x0001
	// Register 400002 == 0x0002
	// Register 400003 == 0x0003
	// Register 400004 == 0x0004  etc...
	
	for(iLoop = 0; iLoop < 500; iLoop++) 
	{
		modbus_memory_map[iLoop*2] = 0;					// Initialize memory bank 
		modbus_memory_map[iLoop*2+1] = iLoop + 1;		// Initialize memory bank per word (16 bits wide)
	}

	printf(" ------------------------------------------ \n");
	printf("          MEMORY BANK INITIALIZED           \n");
	printf(" ------------------------------------------ \n");
	
	// Log memory map to stdout
	for(iLoop = 0; iLoop < 50; iLoop++) 
	{
		printf("Register location: %d  --> contains data word: %d%d \n", iLoop+1, modbus_memory_map[iLoop*2], modbus_memory_map[iLoop*2+1]);
	}
	
	printf("\n etc ... \n\n");
	printf(" ------------------------------------------ \n\n");
}
	
