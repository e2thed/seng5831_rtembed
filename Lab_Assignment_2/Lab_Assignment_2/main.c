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

// My header files
//#include "Motor.h"

// My DEFINES
// define the data direction registers
#define DD_REG_SPEED     DDRD
#define DD_REG_DIRECTION DDRC
#define PORT_SPEED     PORTD
#define PORT_DIRECTION PORTC
#define PIN_SPEED      PIND
#define PIN_DIRECTION  PINC
#define ENCODER_A  (1<<3)
#define ENCODER_B  (1<<2)
#define BIT_MOTOR1 (1<<7)
#define BIT_MOTOR2 (1<<6)
#define GEAR_RATIO 2249
#define TIMER1_PERIOD 10


// GlOBALS
//static char global_m1a;
//static char global_m1b;
static int global_counts_m1;
static int last_global_counts_m1;
static char global_error_m1;
static char global_last_m1a_val;
static char global_last_m1b_val;


// LOCALS
volatile bool print_to_lcd = false;
volatile float rotations = 0.0;
volatile float RPM = 0.0;
volatile float new_motor_speed = 0.0;
volatile float targetRPM = 70;


//ISR HANDLERS
ISR(TIMER1_COMPA_vect) {
	
	//print_to_lcd=true;
	
	float kp = 1.1;
	float ki = 10;
	float kd = 0;
	static float integral = 0.0;
	float derivative = 0.0;
	
	rotations = (global_counts_m1 / GEAR_RATIO);
	
	RPM = (((global_counts_m1 - last_global_counts_m1) / (TIMER1_PERIOD / 1000.0)) * 60.0) / GEAR_RATIO;
	last_global_counts_m1 = global_counts_m1;
	float error = targetRPM - RPM;
	
	if (abs(error) > 0.1)
	{
		integral += error * (TIMER1_PERIOD / 1000.0);
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
}


ISR(PCINT3_vect)
{
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


// INITILIZATIONS
void init_motor1() {
	//////////////////////////////////////////////////////////////////////////
	//	MOTOR PWM Controller             
	//////////////////////////////////////////////////////////////////////////
	// Compare Output Mode, fast PWM mode = COM2A1:0 = 10 for Clear on OC2A Compare Match
	// WGM mode 3 Fast PWM. WGM2:1:0 = 011
	// Clock speed (Prescaler) will be lowest since it does note matter at this point. 8 010
	TCCR2A = (1<<COM2A1) | (0<<COM2A0) | (0<<COM2B1) | (0<<COM2B0) | (0x0<<3) | (0x0<<2) | (1<<WGM21) | (1<<WGM20);
	TCCR2B = (0<<FOC2A) | (0<<FOC2B) | (0x0<<5) | (0x0<<4) | (0<<WGM22) | (0<<CS22) | (1<<CS21) | (0<<CS21);

	// SET PORTD Low
	DD_REG_SPEED &= ~BIT_MOTOR1;
	// SET PORTC HIGH
	DD_REG_DIRECTION |= BIT_MOTOR1;
	
	DD_REG_SPEED     |= BIT_MOTOR1;
	DD_REG_DIRECTION |= BIT_MOTOR1;
	
	//////////////////////////////////////////////////////////////////////////
	//	SETTING PIN WORKS ONLY HERE 
	//	AND STOPS WHEN OUT OF SCOPE!!!
	//////////////////////////////////////////////////////////////////////////
	OCR2A = 70;
	delay_ms(2000);
	//OCR2A = 10;
}

void init_encoder_reading(){
	//////////////////////////////////////////////////////////////////////////
	//	ENCODER  
	//////////////////////////////////////////////////////////////////////////
	// Clear all data direction ports
	DD_REG_SPEED     = 0;
	DD_REG_DIRECTION = 0;
	
	// Configure data direction as input
	DD_REG_SPEED |= ENCODER_A | ENCODER_B;
	PIN_SPEED    |= ENCODER_A | ENCODER_B;
	
	// SET PIN CHANGE INTERRUPT CONFIGURATION
	// PCIE3 for PD2 and PD3 encoder B and A positions on board
	// Enable the encoder pins in the PCMSK
	PCICR	= (1<<PCIE3)   | (0<<PCIE2)   | (0<<PCIE1 )  | (0<<PCIE0 );
	PCMSK3	= (0<<PCINT31) | (0<<PCINT30) | (0<<PCINT29) | (0<<PCINT28) | (1<<PCINT27) | (1<<PCINT26) | (0<<PCINT25) | (0<<PCINT24);
}

void init_PID_control(){
	//////////////////////////////////////////////////////////////////////////
	//	PID Control
	//////////////////////////////////////////////////////////////////////////
	// Compare Output Mode, COM11:0 = 11
	// WGM mode 10 PWM, Phase Correct, WGM13:2:1:0 = 1010
	// Clock speed (Prescalar 1024) CS12:1:0 = 101
	TCCR1A = (1<<COM1A1) | (1<<COM1A0) | (0<<COM1B1) | (0<<COM1B0) | (0x0<<3) | (0x0<<2) | (1<<WGM11) | (0<<WGM10);
	TCCR1B = (0<<ICNC1) | (0<<ICES1) | (0x0<<5) | (1<<WGM13) | (0<<WGM12) | (1<<CS12) | (0<<CS11) | (1<<CS10);
	OCR1A = 19531*((float)TIMER1_PERIOD/2000);
	ICR1  = 39062*((float)TIMER1_PERIOD/2000);
	TIMSK1 = (0x0<<7) | (0x0<<6) | (0<<ICIE1) | (0x0<<4) | (0x0<<3) | (0<<OCIE1B) | (1<<OCIE1A) | (0<<TOIE1);
}

void test_motor1(){
	//////////////////////////////////////////////////////////////////////////
	//     SETTING PIN DOES NOT WORK HERE !!!!!
	//////////////////////////////////////////////////////////////////////////
	print("test motor 1");
	// SPIN MOTOR FOR 2 SECONDS TO MAKE SURE IT WORKS
	OCR2A = 120;
	delay_ms(2000);
	OCR2A = 70;
}

void init_motor(){
	init_motor1();
	init_encoder_reading();
	init_PID_control();
}

int main()
{
	//////////////////////////////////////////////////////////////////////////
	//	TRIED USING THE HEADER FILES TO SEPARATE CODE
	//	BACKED OUT THINKING THIS WAS THE ISSUE
	//	NOW KNOW THAT IT WAS IRRELEVANT AT THIS POINT
	//	CANNOT SET OCR2A AND GET MOTOR WORKING OUT SIDE OF INITIALIZER
	//////////////////////////////////////////////////////////////////////////
	play_from_program_space(PSTR(">g32>>c32"));  // Play welcoming notes.
	
	init_motor();
	test_motor1();
	
	int32_t print_throttle = 0;
	
	sei();
	
	OCR2A = 70;
	delay_ms(2000);
	OCR2A = 70;
	
	while(1)
	{
		print_throttle++;
		if (print_throttle % 500000)
		{
			lcd_goto_xy(0,0);
			print("rpm");
			lcd_goto_xy(8,4);
			print_long(RPM);
			lcd_goto_xy(0,1);
			print("tar");
			lcd_goto_xy(4,1);
			print_long(targetRPM);
			
			print_throttle = 0;
		}
	}
}
