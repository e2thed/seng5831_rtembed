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
//the int32_t or int16_t so you know exactly what is going on
#include <inttypes.h> //gives us uintX_t

//useful stuff from libc
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

//my includes
#include "menu.h"

////GLOBALS
static long global_counts_m1;
static long last_global_counts_m1;
static char global_error_m1;
static char global_last_m1a_val;
static char global_last_m1b_val;

//PRINT/DEBUG FLAGS
volatile bool clearLCD = false;
volatile bool logger = false; // print values for graphs
volatile bool loggerLCD = false;
volatile int logger_cnt = 0;
volatile bool toggleLCD = false;
volatile bool prntKs = false; // print Kp, Ki, Kd
volatile bool dbgLCD = false; // print for debugging, context is what I need to debug
volatile bool test_interrupt = true;
volatile int timer1_cnt = 0;
volatile int step = 1;
volatile int lastStep = 0;

//my GLOBALS
volatile int G_timer_period = 10;

//PID Globals
volatile float Pr = 0.0;
volatile float Pm = 0.0;
volatile float Vr = 0.0;
volatile float Vm = 0.0;
volatile float Kp = 0.0;
volatile float Ki = 0.0;
volatile float Kd = 0.0;
volatile bool execTragectory = false;
volatile bool velocity = false;

//MOTOR Control Vars
volatile int motor1_speed = 0;
volatile int newMotorSpeed = 0;
volatile double error = 0.0;
volatile double integral = 0.0;
volatile double derivative = 0.0;
volatile double rotations = 0.0;
volatile double RPM = 0.0;
volatile double targetRPM = 0.0;
volatile double lasterror = 0.0;

//defines
#define GEAR_RATIO 2249
#define TIMER1_PERIOD 10
#define MOTOR_FWD (PORTC |= (1<<PORTC7))
#define MOTOR_REV (PORTC &= ~(1<<PORTC7))
#define STOP_MOTOR OCR2A = 0
#define MOTOR_MIN 8
#define MOTOR_MAX 255
#define CNT_PER_DEG (GEAR_RATIO/360)
#define cnt_to_deg(cnt) (cnt / CNT_PER_DEG)
#define deg_to_cnt(deg) (deg * 6.2472)

void init_my_timers() {
	//////////////////////////////////////////////////////////////////////////
	//	PWM  - MOTOR1   - TIMER 2
	//////////////////////////////////////////////////////////////////////////
	DDRD |= (1 << PORTD7);
	DDRC |= (1 << PORTC7);
	TCCR2A = (1 << COM2A1) | (0 << COM2A0) | (0 << COM2B1) | (0 << COM2B0) | (0 << 3) | (0 << 2) | (1 << WGM21) | (1 << WGM20); //0x83;
	TCCR2B = (0 << FOC2A) | (0 << FOC2B) | (0x0 << 5) | (0x0 << 4) | (0 << WGM22) |  (0 << CS22) | (1 << CS21) | (0 << CS20); //0x02;
	OCR2A = 0;

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
	// PCIE3 for PD2 and PD3 encoder B and A positions on board
	//DDRC |= 0x12;
	//PIND |= 0x12;
	PCMSK3	= 0x0F; //(0<<PCINT31) | (0<<PCINT30) | (0<<PCINT29) | (0<<PCINT28) | (1<<PCINT27) | (1<<PCINT26) | (0<<PCINT25) | (0<<PCINT24);//0x0C;
	PCICR	= 0xFF;
	PCIFR   = 0xFF;
}

void set_motor_speed(int speed){
	if (speed < 0) { MOTOR_REV;	}
	else { MOTOR_FWD; }
	
	motor1_speed = abs(speed);

	if (motor1_speed >= 255)    { OCR2A = MOTOR_MAX; }
	else						{ OCR2A = motor1_speed; }
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

/***********************************************************
PID CONTROLLER
************************************************************/
ISR(TIMER1_COMPA_vect) {

	static float integral = 0.0;
	float derivative = 0.0;
	float torq = 0.0;
	
	if (velocity)
	{
		dbgLCD = true;
		loggerLCD = true;
		rotations = (global_counts_m1 / GEAR_RATIO);
		RPM = (((global_counts_m1 - last_global_counts_m1) / (TIMER1_PERIOD / 1000.0)) * 60.0) / GEAR_RATIO;
		
		last_global_counts_m1 = global_counts_m1;
		error = Vr - RPM;
		
		integral += (double)(error * (TIMER1_PERIOD / 1000.0));
		
		if (integral > 1000)
		{
			integral = 1000;
		}
		else if (integral < -1000)
		{
			integral = -1000;
		}
		
		derivative = (double)(-error + lasterror)/(TIMER1_PERIOD / 1000.0);
		lasterror = error;
		
		newMotorSpeed = (int)(Kp * error + Ki * integral + Kd * derivative);
		
		set_motor_speed((int)newMotorSpeed);
		
		if ((targetRPM >= RPM - 10)
		&& (targetRPM <= RPM + 10))
		{
			timer1_cnt++;
			if (timer1_cnt % 50 == 0)
			{
				loggerLCD = false;
				set_motor_speed(0);
				Vr = 0;
			}
		}
	}
	else //positional
	{
		loggerLCD = true;
		Pm = global_counts_m1;
		error = Pr - global_counts_m1;
		integral += error * (TIMER1_PERIOD / 1000.0);
		derivative = error - lasterror;
		torq = (Kp*error) + (Ki*integral) + (Kd*derivative);
		newMotorSpeed = torq;
		set_motor_speed(newMotorSpeed);
		lasterror = error;
		
		if (execTragectory)
		{
			if ((global_counts_m1 <= Pr+10)
			&& (global_counts_m1 >= Pr-10))
			{
				timer1_cnt++;
				if(timer1_cnt % 50 == 0)
				{
					step++;
					timer1_cnt = 0;
				}
			}
			else
			{
				timer1_cnt = 0;
			}
		}
		else
		{
			if ((global_counts_m1 <= Pr+10)
			&& (global_counts_m1 >= Pr-10))
			{
				timer1_cnt++;
				if(timer1_cnt % 50 == 0)
				{
					loggerLCD = false;
					set_motor_speed(0);
					Pr = 0;
				}
			}
		}
	}
}

void prnt_intrpt_vals(){
	lcd_goto_xy(0,0);
	printf("tcnt:% d", timer1_cnt);
	lcd_goto_xy(0,1);
	printf("ecnt:% d", global_counts_m1);
}

void prnt_mtr_vals(){
	if(velocity)
	{
		lcd_goto_xy(0,0);
		printf("M:% .2f", RPM);
		lcd_goto_xy(10,0);
		printf("t:%.0f", Vr);
		lcd_goto_xy(0,1);
		printf("T:% .2f", rotations);
	}
	else
	{
		lcd_goto_xy(0,0);
		printf("M:%ld", global_counts_m1);
		lcd_goto_xy(0,1);
		printf("t:%.0f", Pr);
	}
}

void prnt_PID()
{
	lcd_goto_xy(0,0);
	printf("I:% .2f",integral);
	lcd_goto_xy(8,0);
	printf("P:% .2f", error);
	lcd_goto_xy(0,1);
	printf("D:% .2f", derivative);
}

void test_engine(){
	set_motor_speed(25);
	delay_ms(1000);
	set_motor_speed(-25);
	delay_ms(1000);
	STOP_MOTOR;
}

void killMotor()
{
	Kp = 0;
	Ki = 0;
	Kd = 0;
	set_motor_speed(0);
	execTragectory = false;
}

int main()
{
	lcd_init_printf();
	init_menu();
	init_my_timers();
	sei();
	
	int length;
	char tempBuffer[48];
	int print_throttle = 0;
	
	while(1)
	{
		if (clearLCD)
		{
			clear();
		}
		if (execTragectory)
		{
			if (step != lastStep)
			{
				switch (step)
				{
					case 1:
					{
						Pr = Pr + deg_to_cnt(90);
						length = sprintf(tempBuffer, "\r\ns1 Pr:% .3f", Pr);
						print_usb(tempBuffer, length);
						lastStep = 1;
						break;
					}
					case 2:
					{
						Pr = Pr -(deg_to_cnt(360));
						length = sprintf(tempBuffer, "\r\ns2 Pr:% .3f", Pr);
						print_usb(tempBuffer, length);
						lastStep = 2;
						break;
					}
					case 3:
					{
						Pr = Pr + deg_to_cnt(5);
						length = sprintf(tempBuffer, "\r\nS3 Pr:% .3f", Pr);
						print_usb(tempBuffer, length);
						lastStep = 3;
						break;
					}
					default:
					{
						length = sprintf(tempBuffer, "\r\nS Done\r\n");
						print_usb(tempBuffer, length);
						execTragectory = false;
						step = 1;
						killMotor();
						break;
					}
				}
			}
		}
		if (logger)
		{
			if(loggerLCD && (timer1_cnt % 500 == 0))
			{
				logger_cnt++;
				if (velocity)
				{
					length = sprintf(tempBuffer, "\r\n% .2f,% .2f,%d", RPM, Vr, newMotorSpeed);
					print_usb(tempBuffer, length);
					loggerLCD = false;
					timer1_cnt = 0;
				}
				else
				{
					length = sprintf(tempBuffer, "\r\n% .2f,% .2f,%d", Pm, Pr, newMotorSpeed);
					print_usb(tempBuffer, length);
					loggerLCD = false;
					timer1_cnt = 0;
				}
			}
		}
		if (!logger && (logger_cnt > 0))
		{
			print_menu();
			logger_cnt = 0;
		}
		if (prntKs)
		{
			length = sprintf(tempBuffer, "\r\nKp:% .2f Ki:% .2f Kd:% .2f\r\n", Kp, Ki, Kd);
			print_usb(tempBuffer, length);
			length = sprintf(tempBuffer, "Step:%d LastStep:%d\r\n", step, lastStep);
			print_usb(tempBuffer, length);
			prntKs = !prntKs;
			print_menu();
		}
		if (dbgLCD)
		{
			if(execTragectory)
			{
				length = sprintf(tempBuffer, "\r\n% .2f,% .2f,%d", Pm, Pr, newMotorSpeed);
				print_usb(tempBuffer, length);
			}
			else
			{
				if(toggleLCD)
				{
					prnt_PID();
				}
				else
				{
					prnt_mtr_vals();
				}
			}
			dbgLCD = false;
		}
		serial_check();
		check_for_new_bytes_received();
	}
}
