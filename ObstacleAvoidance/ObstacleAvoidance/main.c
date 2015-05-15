/* ObstacleAvoidance - an application for the Pololu Orangutan SVP
*
* This application uses the Pololu AVR C/C++ Library.  For help, see:
* -User's guide: http://www.pololu.com/docs/0J20
* -Command reference: http://www.pololu.com/docs/0J18
*
* Created: 5/8/2015 9:50:17 AM
*  Author: US296865
*/

#include <pololu/orangutan.h>
#include "menu.h"

#define L_SNSR 0
#define R_SNSR 1

////GLOBALS
static long global_counts_m1;
static long last_global_counts_m1;
static char global_error_m1;
static char global_last_m1a_val;
static char global_last_m1b_val;
// ----- 2
static long global_counts_m2;
static long last_global_counts_m2;
static char global_error_m2;
static char global_last_m2a_val;
static char global_last_m2b_val;

////Menu Globals
volatile float Ts;		// target speed
volatile float Mxr;		// max range
volatile float Mnr;		// min range
volatile float H;		// Hysteresis
volatile float Kp;		// P Gain
volatile float Ki;		// I Gain
volatile float Kd;		// D Gain
volatile bool Et;		// execute Trajectory


void init_my_timers() {
	//////////////////////////////////////////////////////////////////////////
	//	PID Controller  - TIMER 1
	//////////////////////////////////////////////////////////////////////////
	// Compare Output Mode, COM11:0 = 11
	// WGM mode 10 PWM, Phase Correct, WGM13:2:1:0 = 1010
	// Clock speed (Prescalar 1024) CS12:1:0 = 101
	TCCR1A = 0x80;//(1<<COM1A1) | (0<<COM1A0) | (0<<COM1B1) | (0<<COM1B0) | (0x0<<3) | (0x0<<2) | (0<<WGM11) | (0<<WGM10);
	TCCR1B = 0x0A;//(0<<ICNC1) | (0<<ICES1) | (0x0<<5) | (0<<WGM13) | (1<<WGM12) | (0<<CS12) | (1<<CS11) | (0<<CS10);
	OCR1A = 25000;
	TIMSK1 = 0x02;// (0x0<<7) | (0x0<<6) | (0<<ICIE1) | (0x0<<4) | (0x0<<3) | (0<<OCIE1B) | (1<<OCIE1A) | (0<<TOIE1);

	//////////////////////////////////////////////////////////////////////////
	//	ENCODER  - PCINT3
	//////////////////////////////////////////////////////////////////////////
	// SET PIN CHANGE INTERRUPT CONFIGURATION
	// PCIE3 for PD2 and PD3 encoder B and A positions on board for motor 1
	// PCIE3 for PD0 and PD1 encoder B and A positions on board for motor 2
	PCMSK3	= (0<<PCINT31) | (0<<PCINT30) | (0<<PCINT29) | (0<<PCINT28) | (1<<PCINT27) | (1<<PCINT26) | (1<<PCINT25) | (1<<PCINT24);;
	PCICR	= 0xFF;
	PCIFR   = 0xFF;
}

// Read Encoder Values
ISR(PCINT3_vect)
{
	// Is Encoder HIGH
	unsigned char m1a_val = ((PIND & (1<<PIND3)) > 0);
	unsigned char m1b_val = ((PIND & (1<<PIND2)) > 0);
	unsigned char m2a_val = ((PIND & (1<<PIND1)) > 0);
	unsigned char m2b_val = ((PIND & (1<<PIND0)) > 0);
	
	// Truth Table for Direction
	char plus_m1 = m1a_val ^ global_last_m1b_val;
	char minus_m1 = m1b_val ^ global_last_m1a_val;
	// ---- 2
	char plus_m2 = m2a_val ^ global_last_m2b_val;
	char minus_m2 = m2b_val ^ global_last_m2a_val;

	// Increment appropriate counts
	if(plus_m1)
	{
		global_counts_m1 += 1;
	}
	if(minus_m1)
	{
		global_counts_m1 -= 1;
	}
	// ---- 2
	if(plus_m2)
	{
		global_counts_m2 += 1;
	}
	if(minus_m2)
	{
		global_counts_m2 -= 1;
	}

	// Set error to 1, if both m1 values do not equal the their last respective value
	if(m1a_val != global_last_m1a_val && m1b_val != global_last_m1b_val)
	{
		global_error_m2 = 1;
	}
	
	// Set error to 1, if both m2 values do not equal the their last respective value
	if(m1a_val != global_last_m1a_val && m1b_val != global_last_m1b_val)
	{
		global_error_m2 = 1;
	}
	
	// Set Last variable for next iteration
	global_last_m1a_val = m1a_val;
	global_last_m1b_val = m1b_val;
	// ----- 2
	global_last_m2a_val = m2a_val;
	global_last_m2b_val = m2b_val;
}

/***********************************************************
PID CONTROLLER
************************************************************/
ISR(TIMER1_COMPA_vect) {

	static float integral = 0.0;
	float derivative = 0.0;
	float torq = 0.0;
	float dt = 0.0;
	long trgt = 500;
	
	dt = (double)(TIMER1_PERIOD/1000.0);
	
	M1t = (global_counts_m1 - last_global_counts_m1);
	last_global_counts_m1 = global_counts_m1;
	error = Vr - Vm;
	
	integral += (double)(error * dt);
	
	if (integral > 1000)
	{
		integral = 1000;
	}
	else if (integral < -1000)
	{
		integral = -1000;
	}
	
	derivative = (double)(error - lasterror)/dt; // changed signs from +error-lasterror - due to not working out well.
	lasterror = error;

	torq = (Kp*(Vr - Vm)) + (Ki * error) + (Kd * derivative);
	
	newMotorSpeed += torq;
	set_motor_speed(newMotorSpeed);
}

int main()
{
	play_from_program_space(PSTR(">g32>>c32"));  // Play welcoming notes.
	
	int throttlePrint = 0;
	int lastReading = 4316;
	int currentReading = 0;
	//init_menu();
	
	set_m1_speed(70);
	delay_ms(2000);
	set_m1_speed(0);
	delay_ms(100);
	set_m1_speed(-70);
	delay_ms(2000);
	set_m1_speed(0);
	
	while(1)
	{
		if (throttlePrint % 5000 == 0)
		{
			clear();
			lcd_goto_xy(0,0);
			//print_unsigned_long(analog_read_millivolts(0));
			currentReading = analog_read_millivolts(0);
			if ((lastReading >= currentReading + 5)
			|| (lastReading <= currentReading - 5))
			{
				lastReading = currentReading;
			}
			print_unsigned_long(lastReading);

			throttlePrint = 0;
		}
		throttlePrint++;
	}
}
