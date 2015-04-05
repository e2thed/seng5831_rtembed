/* Lab_Assignment_2 - an application for the Pololu Orangutan SVP
*
* This application uses the Pololu AVR C/C++ Library.  For help, see:
* -User's guide: http://www.pololu.com/docs/0J20
* -Command reference: http://www.pololu.com/docs/0J18
*
* Created: 3/29/2015 2:31:22 PM
*  Author: Edward Castillo
*/

#include <pololu/orangutan.h>
//Gives us uintX_t (e.g. uint32_t - unsigned 32 bit int)
//On the ATMega128 int is actually 16 bits, so it is better to use
//  the int32_t or int16_t so you know exactly what is going on
#include <inttypes.h> //gives us uintX_t

// useful stuff from libc
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

//my includes
#include "menu.h"

// GLOBALS
static int global_counts_m1;
static int last_global_counts_m1;
static char global_error_m1;
static char global_last_m1a_val;
static char global_last_m1b_val;

// my GLOBALS
volatile bool logger = false;
volatile bool printValues = false;
volatile bool test_interrupt = true;
volatile bool execTragectory = false;
volatile bool print2lcd = false;
volatile bool printLCD = false;
volatile bool toggleLCD = false;
volatile double Pr = 0.0;
volatile double Pm = 0.0;
volatile double Vr = 0.0;
volatile double Vm = 0.0;

// PID Globals
volatile double error = 0.0;
volatile double integral = 0.0;
volatile double derivative = 0.0;
volatile double Kp = 0.0;
volatile double Ki = 0.0;
volatile double Kd = 0.0;
volatile double newMotorSpeed = 0.0;
volatile double rotations = 0.0;
volatile double RPM = 0.0;
volatile int motor1_speed = 0;
volatile G_timer_period = 10;

// MOTOR STUFFS
#define GEAR_RATIO 2249
#define TIMER1_PERIOD 10
#define MOTOR_FWD (PORTC |= (1<<PORTC7))
#define MOTOR_REV (PORTC &= ~(1<<PORTC7))
#define STOP_MOTOR OCR2A = 0
#define MOTOR_MIN 8
#define MOTOR_MAX 255
#define COUNT_PER_DEGREES (GEAR_RATIO/360)


void init_timers() {
	//////////////////////////////////////////////////////////////////////////
	//	MOTOR
	//////////////////////////////////////////////////////////////////////////
	DDRD |= 0x80;
	DDRC |= 0x80;
	TCCR2A = 0x83;
	TCCR2B = 0x02;
	OCR2A = 0;

	//////////////////////////////////////////////////////////////////////////
	//	PID Controller
	//////////////////////////////////////////////////////////////////////////
	// Compare Output Mode, COM11:0 = 11
	// WGM mode 10 PWM, Phase Correct, WGM13:2:1:0 = 1010
	// Clock speed (Prescalar 1024) CS12:1:0 = 101
	TCCR1A = 0x80;//(1<<COM1A1) | (0<<COM1A0) | (0<<COM1B1) | (0<<COM1B0) | (0x0<<3) | (0x0<<2) | (0<<WGM11) | (0<<WGM10);
	TCCR1B = 0x0A;//(0<<ICNC1) | (0<<ICES1) | (0x0<<5) | (0<<WGM13) | (1<<WGM12) | (0<<CS12) | (1<<CS11) | (0<<CS10);
	OCR1A = 25000;
	TIMSK1 = 0x02;// (0x0<<7) | (0x0<<6) | (0<<ICIE1) | (0x0<<4) | (0x0<<3) | (0<<OCIE1B) | (1<<OCIE1A) | (0<<TOIE1);

	//////////////////////////////////////////////////////////////////////////
	//	ENCODER
	//////////////////////////////////////////////////////////////////////////
	// SET PIN CHANGE INTERRUPT CONFIGURATION
	// PCIE3 for PD2 and PD3 encoder B and A positions on board
	DDRC |= 0x12;
	PIND |= 0x12;
	PCMSK3	= 0x0C; //(0<<PCINT31) | (0<<PCINT30) | (0<<PCINT29) | (0<<PCINT28) | (1<<PCINT27) | (1<<PCINT26) | (0<<PCINT25) | (0<<PCINT24);//0x0C;
	PCICR	= 0xff;
	PCIFR   = 0xff;
}

void set_motor_speed(int speed){
	if (speed < 0) { MOTOR_REV;	}
	else { MOTOR_FWD; }
	
	motor1_speed = abs(speed);

	if (motor1_speed >= 255)    { OCR2A = MOTOR_MAX; }
	else                        { OCR2A = motor1_speed; }
}

ISR(PCINT3_vect)
{
	printLCD = true;
	// Is Encoder HIGH
	unsigned char m1a_val = ((PIND & (1<<PIND3)) > 0);
	unsigned char m1b_val = ((PIND & (1<<PIND2)) > 0);
	
	// Truth Table for Direction
	char plus_m1 = m1a_val ^ global_last_m1b_val;
	char minus_m1 = m1b_val ^ global_last_m1a_val;

	// Increment appropriate counts
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
	
	// Set Last variable for next iteration
	global_last_m1a_val = m1a_val;
	global_last_m1b_val = m1b_val;
}

ISR(TIMER1_COMPA_vect){
	rotations = (global_counts_m1 / GEAR_RATIO);

	if (execTragectory)
	{
		RPM = (((global_counts_m1 - last_global_counts_m1) / (TIMER1_PERIOD / 1000.0)) * 60.0) / GEAR_RATIO;
		last_global_counts_m1 = global_counts_m1;
		error = Vr - RPM;

		if ((Ki > 0) && (Vr > 0))
		{
			if (abs(error) > 0.1)
			{
				integral += error * (TIMER1_PERIOD / 1000.0);
			}
		}

		newMotorSpeed = Kp * error + Ki * integral + Kd * derivative;

		set_motor_speed((int)newMotorSpeed);
	}
}


int main()
{
	lcd_init_printf();
	init_menu();
	init_timers();
	sei();
	
	play_from_program_space(PSTR(">g32>>c32"));  // Play welcoming notes.
	clear();
	
	prnt_enc_vals();
	
	int length;
	char tempBuffer[48];
	int print_throttle = 0;

	while(1)
	{
		print_throttle++;
		if (printLCD = false && print_throttle % 500 == 0)
		{
			lcd_goto_xy(0,0);
			printf("E:% .2f", error);
			/* RPM and rotations*/
			lcd_goto_xy(8,1);
			print("C:");
			print_long(global_counts_m1);
			print_throttle = 0;
		}
		if (logger)
		{
			//do stuffs
		}
		if (printValues)
		{
			length = sprintf(tempBuffer, "\r\nKp:% .2f Ki:% .2f Kd:% .2f\r\n", Kp, Ki, Kd);
			print_usb(tempBuffer, length);
			printValues = !printValues;
		}
		if (printLCD)
		{
			prnt_enc_vals();
			printLCD = false;
		}
		serial_check();
		check_for_new_bytes_received();
	}
}

void prnt_enc_vals(){
	lcd_goto_xy(0,0);
	printf("c:%d", global_counts_m1);
	lcd_goto_xy(0,1);
	printf("l:%d", last_global_counts_m1);
}

void prnt_mtr_vals(){
	lcd_goto_xy(8,0);
	printf("M:% .2f", RPM);
	lcd_goto_xy(0,1);
	printf("T:% .2f", rotations);
}

void test_engine(){
	set_motor_speed(25);
	delay_ms(1000);
	set_motor_speed(-25);
	delay_ms(1000);
	STOP_MOTOR;
}

float degrees_to_count(int degrees){
	return degrees * COUNT_PER_DEGREES;
}

int counts_to_degrees(float count){
	return round(count / COUNT_PER_DEGREES);
}

