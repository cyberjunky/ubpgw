/***************************************************************************
 * 
 *  Project: UppS0  
 *  
 *  header file that defines the stract that holds all the data 
 *
 ***************************************************************************/

/***************************************************************************
 * 
 *  Geany compile and build directives  
 *  
 *  Compile: 	gcc -Wall -c "%f"
 *  Build: 		gcc -Wall -o "%e" "%f" "ini.c" "-lcurl" "-lwiringPi"
 *
 ***************************************************************************/


// ---------------------------------------------------------------------
// Configuration parameters in config.ini 
// ---------------------------------------------------------------------
// [config]               			; S0 sensor configuration
// UppBoxID 	= 16607001       	; UppBox ID YYMDDnnn where M is Month in Hex
// UppS0ID1 	= 16607020       	; UppSensor ID YYMDDnnn where M is Month in Hex
// ImpskWh1 	= 1000				; Number of impluses per kWh
// UppS0ID2 	= 16607021       	; UppSensor ID YYMDDnnn where M is Month in Hex
// ImpskWh2 	= 1000				; Number of impluses per kWh
// UppS0ID3 	= 16607022       	; UppSensor ID YYMDDnnn where M is Month in Hex
// ImpskWh3 	= 1000				; Number of impluses per kWh
// UppS0ID4 	= 16607023       	; UppSensor ID YYMDDnnn where M is Month in Hex
// ImpskWh4 	= 1000				; Number of impluses per kWh
// UppS0ID5 	= 16607024       	; UppSensor ID YYMDDnnn where M is Month in Hex
// ImpskWh5 	= 1000				; Number of impluses per kWh (typically 500 or 1000)
// Interval 	= 10				; Interval between S0 telegrams in seconds (10 sec)
// Dongle		= /dev/ttyACM0		; S0 tty input port				
// URL			= http://uppenergy.com/public/api/store/measurement/	; appended with the UppBoxID
// ---------------------------------------------------------------------

// ---------------------------------------------------------------------
// Actuals in meter.ini 
// ---------------------------------------------------------------------
// [actual]							; Actual meter value
// ActualM1 		= 1000       	; Actual kWh value M1
// ActualM1 		= 2000			; Actual kWh value M2
// ActualM1 		= 3000          ; Actual kWh value M3
// ActualM1	 		= 4000          ; Actual kWh value M4
// ActualM1 		= 5000			; Actual kWh value M5
// --------------------------------------------------------------------
// ---------------------------------------------------------------------


// ---------------------------------------------------------------------
// --- declare constants
// ---------------------------------------------------------------------

#define 	BUF_SIZE	 6			// Buffer size for moving average

// ---------------------------------------------------------------------
// --- typedev struct that holds all the S0 data
// ---------------------------------------------------------------------


typedef struct
{
	// parameters from config.ini
    char* str_config_ini_upp_box_ID;
    char* str_config_ini_uppsensor_ID_M1;
    char* str_config_ini_puls_per_kWh_M1;
    char* str_config_ini_uppsensor_ID_M2;
    char* str_config_ini_puls_per_kWh_M2;
    char* str_config_ini_uppsensor_ID_M3;
    char* str_config_ini_puls_per_kWh_M3;
    char* str_config_ini_uppsensor_ID_M4;
    char* str_config_ini_puls_per_kWh_M4;
    char* str_config_ini_uppsensor_ID_M5;
    char* str_config_ini_puls_per_kWh_M5;
    char* str_config_ini_TTY_DEVICE;	 		// --- S0 tty input device name e.g. /dev/ttyACM0
    char* str_config_ini_TTY_BAUD;	 			// --- Baudrate of the tty input port	
    char* str_config_ini_INTERVAL;				// --- Interval between S0 telegrams
	char* str_config_ini_URL;					// --- http://uppenergy.com/public/api/store/measurement/<UppBoxID>


	// Actual starting values meter.ini
    char* str_meter_ini_M1;
    char* str_meter_ini_M2;
    char* str_meter_ini_M3;
    char* str_meter_ini_M4;
    char* str_meter_ini_M5;


	// S0 RAW data string from dongle
	char* str_tty_raw_data;				// RAW string from ttyACM0
	
	// S0 interpretted data from dongle
	unsigned int uint_tty_id;			// ID- typically something like this "S0M10001"
	unsigned int uint_tty_interval;		// Interval in seconds - typically 10 sec

	unsigned int uint_tty_delta_M1;		// delta pulses during the last 10 seconds
	unsigned int uint_tty_delta_M2;		// delta pulses during the last 10 seconds
	unsigned int uint_tty_delta_M3;		// delta pulses during the last 10 seconds
	unsigned int uint_tty_delta_M4;		// delta pulses during the last 10 seconds
	unsigned int uint_tty_delta_M5;		// delta pulses during the last 10 seconds

	unsigned int uint_tty_total_M1;		// total number of pulses sinds last restart
	unsigned int uint_tty_total_M2;		// total number of pulses sinds last restart
	unsigned int uint_tty_total_M3;		// total number of pulses sinds last restart
	unsigned int uint_tty_total_M4;		// total number of pulses sinds last restart
	unsigned int uint_tty_total_M5;		// total number of pulses sinds last restart


	/* Fifo buffers for average */
	unsigned int uint_load_buffer_M1[BUF_SIZE]; 	// circular FiFo buffer to calculate the moving average of load
	unsigned int uint_load_buffer_M2[BUF_SIZE]; 	// circular FiFo buffer to calculate the moving average of load
	unsigned int uint_load_buffer_M3[BUF_SIZE]; 	// circular FiFo buffer to calculate the moving average of load
	unsigned int uint_load_buffer_M4[BUF_SIZE]; 	// circular FiFo buffer to calculate the moving average of load
	unsigned int uint_load_buffer_M5[BUF_SIZE]; 	// circular FiFo buffer to calculate the moving average of load

	/* Internal calculation */
	unsigned int uint_calc_load_M1;			// actual load in Watt
	unsigned int uint_calc_total_M1;		// total enegry in kWh 
	unsigned int uint_calc_load_M2;			// actual load in Watt
	unsigned int uint_calc_total_M2;		// total enegry in kWh 
	unsigned int uint_calc_load_M3;			// actual load in Watt
	unsigned int uint_calc_total_M3;		// total enegry in kWh 
	unsigned int uint_calc_load_M4;			// actual load in Watt
	unsigned int uint_calc_total_M4;		// total enegry in kWh 
	unsigned int uint_calc_load_M5;			// actual load in Watt
	unsigned int uint_calc_total_M5;		// total enegry in kWh 


	/* Payload data for HTTP POST request */
	char* str_http_post_payload;
	
} struct_SO_DATA;


