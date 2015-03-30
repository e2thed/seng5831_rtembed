/* Seng5831_Assignments - an application for the Pololu Orangutan SVP
*
* This application uses the Pololu AVR C/C++ Library.  For help, see:
* -User's guide: http://www.pololu.com/docs/0J20
* -Command reference: http://www.pololu.com/docs/0J18
*
* Created: 1/31/2015 2:39:21 PM
*  Author: Edward Castillo
*
* This is the lab in progress
*/

#include <pololu/orangutan.h>
#include "menu.h"
//Gives us uintX_t (e.g. uint32_t - unsigned 32 bit int)
//On the ATMega128 int is actually 16 bits, so it is better to use
//  the int32_t or int16_t so you know exactly what is going on
#include <inttypes.h> //gives us uintX_t

// useful stuff from libc
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#define targetRPM 125
enum DIR {Forward='F', Reverse='R', Stop='S'};

// Local
volatile float new_motor_speed = 0.0;
volatile char motor1_dir = Stop;
volatile int16_t motor1_speed = 0;
volatile throttle_mode = 1;
volatile uint32_t G_timer_period = 10;
volatile bool print_to_lcd = false;

// My Locals for the lab assignment 2
volatile float T =	0.0;	//Output motor signal (torque)
volatile float Pr =	0.0;	//Desired value (motor or speed for us)
volatile float Pm =	0.0;	//Measured value
volatile float Kp =	0.0;	//Proportional gain
volatile float Ki =	0.0;	//Integral gain Kd = Derivative gain

// MOTOR Globals
static char global_m1a;
static char global_m1b;
static int global_counts_m1;
static int last_global_counts_m1;
static char global_error_m1;
static char global_last_m1a_val;
static char global_last_m1b_val;

// Calculation variables
float RPM = 0.0;
int32_t rotations = 0;
#define gear_ratio 2256


void init_motor()
{
	DDRD    = 0;
	PIND	= 0;
	DDRD	|= 0x12;
	PIND	|= 0x12;
	PCICR	= (1<<PCIE3) | (0<<PCIE2 ) | (0<<PCIE1 ) | (0<<PCIE0 );
	PCMSK3	= (0<<PCINT31) | (0<<PCINT30) | (0<<PCINT29) | (0<<PCINT28) | (1<<PCINT27) | (1<<PCINT26) | (0<<PCINT25) | (0<<PCINT24);
}

void init_timer2()
{
	//<--      MOTOR PWM Controller          -->
	// Compare Output Mode, fast PWM mode = COM2A1:0 = 10 for Clear on OC2A Compare Match
	// WGM mode 3 Fast PWM. WGM2:1:0 = 011
	// Clock speed (Prescaler) will be lowest since it does note matter at this point. 8 010
	TCCR2A = (1<<COM2A1) | (0<<COM2A0) | (0<<COM2B1) | (0<<COM2B0) | (0x0<<3) | (0x0<<2) | (1<<WGM21) | (1<<WGM20);
	TCCR2B = (0<<FOC2A) | (0<<FOC2B) | (0x0<<5) | (0x0<<4) | (0<<WGM22) | (0<<CS22) | (1<<CS21) | (0<<CS21);
	OCR2A = 0;
	
	// SET PORT Low
	PORTD &= ~(1<<PIND7);
	PORTC |= (1<<PINC7);	// Try high for experiment; nice it has flipped directions.
	
	DDRD |= (1<<PIND7);
	DDRC |= (1<<PINC7);
}

void init_timer1()
{
	//<--	PID Controller   ->
	// Compare Output Mode, COM11:0 = 11
	// WGM mode 10 PWM, Phase Correct, WGM13:2:1:0 = 1010
	// Clock speed (Prescalar 1024) CS12:1:0 = 101
	TCCR1A = (1<<COM1A1) | (1<<COM1A0) | (0<<COM1B1) | (0<<COM1B0) | (0x0<<3) | (0x0<<2) | (1<<WGM11) | (0<<WGM10);
	TCCR1B = (0<<ICNC1) | (0<<ICES1) | (0x0<<5) | (1<<WGM13) | (0<<WGM12) | (1<<CS12) | (0<<CS11) | (1<<CS10);
	OCR1A = 19531*((float)G_timer_period/2000);
	ICR1  = 39062*((float)G_timer_period/2000);
	TIMSK1 = (0x0<<7) | (0x0<<6) | (0<<ICIE1) | (0x0<<4) | (0x0<<3) | (0<<OCIE1B) | (1<<OCIE1A) | (0<<TOIE1);
}

ISR(PCINT3_vect)
{
	unsigned char m1a_val = ((PIND & (1<<PIND3)) > 0);
	unsigned char m1b_val = ((PIND & (1<<PIND2)) > 0);
	
	// Define and calculate High and Low Signal
	char plus_m1 = m1a_val ^ global_last_m1b_val;
	char minus_m1 = m1b_val ^ global_last_m1a_val;

	// Increment counts
	if(plus_m1)
	{
		global_counts_m1 += 1;
	}
	if(minus_m1)
	{
		global_counts_m1 -= 1;
	}

	// Set error to 1, if both m1 values do not equal the their last respective value
	if(m1a_val != global_last_m1a_val && m1b_val != global_last_m1b_val)
	{
		global_error_m1 = 1;
	}

	global_last_m1a_val = m1a_val;
	global_last_m1b_val = m1b_val;
	print_to_lcd = true;
}

int main()
{
	play_from_program_space(PSTR(">g32>>c32"));  // Play welcoming notes.

	//init_led();
	//init_menu();
	//init_motor();
	//init_timer1();
	init_timer2();
	OCR2A = 125;
	delay_ms(2000);
	OCR2A = 0;
	sei();
	clear();

	while (1)
	{
		serial_check();
		check_for_new_bytes_received();
		
		if (print_to_lcd)
		{
			lcd_goto_xy(0,0);
			print_long(rotations);
			lcd_goto_xy(4,0);
			print_long(RPM);
			
			lcd_goto_xy(0,1);
			print_long((int16_t)new_motor_speed);
			//printf("%f",new_motor_speed);
			print_to_lcd = false;
		}
		
	}
}

// INTERRUPT HANDLER
ISR(TIMER1_COMPA_vect) {
	print_to_lcd=true;
	float kp = 1.1;
	float ki = 10;
	float kd = 0;
	static float integral = 0.0;
	float derivative = 0.0;
	
	rotations = (global_counts_m1 / gear_ratio);
	
	RPM = (((global_counts_m1 - last_global_counts_m1) / (G_timer_period / 1000.0)) * 60.0) / gear_ratio;
	last_global_counts_m1 = global_counts_m1;
	float error = targetRPM - RPM;
	
	if (abs(error) > 0.1)
	{
		integral += error * (G_timer_period / 1000.0);
	}
	
	new_motor_speed = kp * error + ki * integral + kd * derivative;
	
	if (new_motor_speed > 255)
	{
		new_motor_speed = 255;
	}
	if (new_motor_speed < -255)
	{
		new_motor_speed = -255;
	}
	
	//OCR2A = new_motor_speed;
	//set_motors(new_motor_speed,0);
	
}




//// for the light for testing purposes
//#define DD_REG_GREEN  DDRD
//#define PORT_GREEN  PORTD
//#define BIT_GREEN    ( 1 << 5 )
//#define LED_ON(x)     (PORT_##x |= BIT_##x)
//#define LED_OFF(x)    (PORT_##x &= ~(BIT_##x))
//#define LED_TOGGLE(x) (PORT_##x ^= BIT_##x)
//#define DD_REG_OFF(x) (DD_REG_##x &= ~(BIT_##x))
//#define DD_REG_ON(x) (DD_REG_##x |= (BIT_##x))


