/*
Ed Castillo
ugh
*/
#include "menu.h"

#include <stdio.h>
#include <inttypes.h>
#include <stdbool.h>

// my GLOBALS
// PRINT/DEBUG FLAGS
extern bool logger;		// print values for graphs
extern bool prntKs;		// print Kp, Ki, Kd
extern bool toggleLCD;
extern bool clearLCD;

// PID Globals
extern float Pr;		// reference position / reference speed(?)
extern float Vr;
extern float Pm;
extern float Kp;
extern float Ki;
extern float Kd;
extern bool execTragectory;
extern bool velocity;

// MOTOR Control Vars
extern int	motor1_speed;

// local "global" data structures
char receive_buffer[48];
unsigned char receive_buffer_position;
char send_buffer[48];
// A generic function for whenever you want to print to your serial comm window.
// Provide a string and the length of that string. My serial comm likes "\r\n" at
// the end of each string (be sure to include in length) for proper linefeed.
void print_usb( char *buffer, int n ){
	serial_send( USB_COMM, buffer, n );
	wait_for_sending_to_finish();
}
void print_menu(){
	pre_print_usb("\r\nMenu: <char> <float>\r\n");
	pre_print_usb("\rTYPE {Hh?} for help: ");
}
void pre_print_usb( char *buffer){
	int length;
	char tempBuffer[48];

	length = sprintf(tempBuffer, buffer);
	print_usb(tempBuffer, length);
}

//------------------------------------------------------------------------------------------
// Initialize serial communication through USB and print menu options
// This immediately readies the board for serial comm
void init_menu() {
	
	char printBuffer[48];
	
	// Set the baud rate to 9600 bits per second.  Each byte takes ten bit
	// times, so you can get at most 960 bytes per second at this speed.
	serial_set_baud_rate(USB_COMM, 9600);

	// Start receiving bytes in the ring buffer.
	serial_receive_ring(USB_COMM, receive_buffer, sizeof(receive_buffer));

	//memcpy_P( send_buffer, PSTR("USB Serial Initialized\r\n"), 24 );
	//snprintf( printBuffer, 24, "USB Serial Initialized\r\n");
	//print_usb( printBuffer, 24 );
	pre_print_usb( "\r\n\r\nUSB Serial Initialized\r\n");

	//memcpy_P( send_buffer, MENU, MENU_LENGTH );
	//print_usb( MENU, MENU_LENGTH );
	print_menu();
}

//------------------------------------------------------------------------------------------
// process_received_byte: Parses a menu command (series of keystrokes) that
// has been received on USB_COMM and processes it accordingly.
// The menu command is buffered in check_for_new_bytes_received (which calls this function).
void process_received_string(const char* buffer)
{
	// Used to pass to USB_COMM for serial communication
	int length;
	char tempBuffer[48];
	
	// parse and echo back to serial comm window (and optionally the LCD)
	char op_cmd;
	float value;
	int parsed;
	
	parsed = sscanf(buffer, "%c %f", &op_cmd, &value);
	
	length = sprintf(tempBuffer, "Op:%c V:%d\r\n", op_cmd, value);
	print_usb( tempBuffer, length);
	
	switch (op_cmd)
	{
		case 'l':
		case 'L':	//L/l: Start/Stop Logging (print) the values of Pr, Pm, and T
		{
			logger = !logger;
			break;
		}
		case 'v':
		case 'V':	//V/v: View the current values Kd, Kp, Vm, Pr, Pm, and T
		{
			prntKs = true;
			break;
		}
		case 'r':
		case 'R':	//R/r : Set the reference position (use unit "counts")
		{
			setPBasePIDs();
			velocity = false;
			Pr = value;
			break;
		}
		case 's':
		case 'S':	//S/s : Set the reference speed (use unit "counts"/sec)
		{
			setVBasePIDs();
			velocity = true;
			Vr = value;
			break;
		}
		case 'P':
		case 'p':	//p: Decrease Kp by an amount of your choice
		{
			Kp = value;
			break;
		}
		case 'i':
		case 'I':	//P: Increase Kp by an amount of your choice*
		{
			Ki = value;
			break;
		}
		case 'd':	//D: Increase Kd by an amount of your choice
		case 'D':	//d: Decrease Kd by an amount of your choice
		{
			Kd = value;
			break;
		}
		case 'b':
		case 'B':
		{
			clearLCD = true;
			toggleLCD = !toggleLCD;
			break;
		}
		case 'c':
		case 'C':
		{
			test_engine();
			break;
		}
		case 't':
		case 'T':	//t: Execute trajectory
		{
			setTBasePIDs();
			execTragectory = true;
			velocity = false;
			break;
		}
		case 'k':
		case 'K':
		{
			set_motor_speed();
			resetPIDs();
			break;
		}
		case 'h':
		case 'H':
		case '?':
		{
			pre_print_usb("L/l:Toggle logging Pr,Pm,T\r\n");
			pre_print_usb("V/v:View Kd,Kp,Vm,Pr,Pm,T\r\n");
			pre_print_usb("R/r:Set ref pos (use unit count)\r\n");
			pre_print_usb("S/s:Set ref speed (counts/sec)\r\n");
			pre_print_usb("P/p:Set Kp - 10 -> .1\r\n");
			pre_print_usb("D/d:Set Kd - 10 -> .1\r\n");
			pre_print_usb("I/i:Set Ki - 10 -> .1\r\n");
			pre_print_usb("B/b:Toggle LCD\r\n");
			pre_print_usb("C/c:Run test func. For dbg\r\n");
			pre_print_usb("t:Execute trajectory\r\n");
			break;
		}
		default:
		{
			pre_print_usb( "Cmd unkn. Try {?}\r\n" );
			return;
		}
	}
	if(!prntKs)
	{
		print_menu();
	}

} //end menu()

//---------------------------------------------------------------------------------------
// If there are received bytes to process, this function loops through the receive_buffer
// accumulating new bytes (keystrokes) in another buffer for processing.
void check_for_new_bytes_received()
{
	/*
	The receive_buffer is a ring buffer. The call to serial_check() (you should call prior to this function) fills the buffer.
	serial_get_received_bytes is an array index that marks where in the buffer the most current received character resides.
	receive_buffer_position is an array index that marks where in the buffer the most current PROCESSED character resides.
	Both of these are incremented % (size-of-buffer) to move through the buffer, and once the end is reached, to start back at the beginning.
	This process and data structures are from the Pololu library. See examples/serial2/test.c and src/OrangutanSerial/ *.*
	
	//A carriage return from your comm window initiates the transfer of your keystrokes.
	All key strokes prior to the carriage return will be processed with a single call to this function (with multiple passes through this loop).
	On the next function call, the carriage return is processes with a single pass through the loop.
	The menuBuffer is used to hold all keystrokes prior to the carriage return. The "received" variable, which indexes menuBuffer, is reset to 0
	after each carriage return.
	*/
	char menuBuffer[32];
	static int received = 0;
	
	// while there are unprocessed keystrokes in the receive_buffer, grab them and buffer
	// them into the menuBuffer
	while(serial_get_received_bytes(USB_COMM) != receive_buffer_position)
	{
		// place in a buffer for processing
		menuBuffer[received] = receive_buffer[receive_buffer_position];
		++received;
		
		// Increment receive_buffer_position, but wrap around when it gets to
		// the end of the buffer.
		if ( receive_buffer_position == sizeof(receive_buffer) - 1 )
		{
			receive_buffer_position = 0;
		}
		else
		{
			receive_buffer_position++;
		}
	}
	//if there were keystrokes processed, check if a menu command
	if (received)
	{
		if (1 == received && (menuBuffer[0] == '\n' || menuBuffer[0] == '\r'))
		{
			received = 0;
			return;
		}
		// Process buffer: terminate string, process, rest index to beginning of array to receive another command
		menuBuffer[received] = '\0';
		process_received_string(menuBuffer);
		received = 0;
	}
}

//-------------------------------------------------------------------------------------------
// wait_for_sending_to_finish:  Waits for the bytes in the send buffer to
// finish transmitting on USB_COMM.  We must call this before modifying
// send_buffer or trying to send more bytes, because otherwise we could
// corrupt an existing transmission.
void wait_for_sending_to_finish()
{
	while(!serial_send_buffer_empty(USB_COMM))
	serial_check();		// USB_COMM port is always in SERIAL_CHECK mode
}

void setVBasePIDs(){
	Kp = .8;
	Ki = .8;
	Kd = .02;
}

void setPBasePIDs(){
	Kp = .05;
	Ki = .25;
	Kd = .1;
}

void setTBasePIDs(){
	Kp = .1;
	Ki = .25;
	Kd = .1;
}

void resetPIDs(){
	Kp = 0.0;
	Ki = 0.0;
	Kd = 0.0;
}