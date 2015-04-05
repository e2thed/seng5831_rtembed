////#include "timer.h"
//#include "motor.h"
//
//#include <avr/interrupt.h>
//#include <stdbool.h>
//
//// GLOBALS
//static int global_counts_m1;
//static int last_global_counts_m1;
//static char global_error_m1;
//static char global_last_m1a_val;
//static char global_last_m1b_val;
//
//extern bool printLCD;
//extern float newMotorSpeed;
//extern float rotations;
//extern float RPM;
//extern float targetRPM;
//
//// PID Globals
//extern float error;
//extern float integral;
//extern float derivative;
//extern float Kp;
//extern float Ki;
//extern float Kd;
//
//volatile G_timer_period = 10;
//
//void init_timers() {
	////<--	PID Controller   ->
	//// Compare Output Mode, COM11:0 = 11
	//// WGM mode 10 PWM, Phase Correct, WGM13:2:1:0 = 1010
	//// Clock speed (Prescalar 1024) CS12:1:0 = 101
	//TCCR1A = (1<<COM1A1) | (1<<COM1A0) | (0<<COM1B1) | (0<<COM1B0) | (0x0<<3) | (0x0<<2) | (1<<WGM11) | (0<<WGM10);
	//TCCR1B = (0<<ICNC1) | (0<<ICES1) | (0x0<<5) | (1<<WGM13) | (0<<WGM12) | (1<<CS12) | (0<<CS11) | (1<<CS10);
	//OCR1A = 19531*((float)G_timer_period/2000);
	//ICR1  = 39062*((float)G_timer_period/2000);
	//TIMSK1 = (0x0<<7) | (0x0<<6) | (0<<ICIE1) | (0x0<<4) | (0x0<<3) | (0<<OCIE1B) | (1<<OCIE1A) | (0<<TOIE1);
//}
//
//ISR(TIMER1_COMPA_vect){
	//printLCD = true;
	//
	//rotations = (global_counts_m1 / GEAR_RATIO);
//
	//RPM = (((global_counts_m1 - last_global_counts_m1) / (TIMER1_PERIOD / 1000.0)) * 60.0) / GEAR_RATIO;
	//last_global_counts_m1 = global_counts_m1;
	//error = targetRPM - RPM;
//
	//if (abs(error) > 0.1)
	//{
		//integral += error * (TIMER1_PERIOD / 1000.0);
	//}
//
	//newMotorSpeed = Kp * error + Ki * integral + Kd * derivative;
//
	//set_motor_speed(newMotorSpeed);
//}