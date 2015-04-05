//#include "motor.h"
//#include "menu.h"
//
//#include <pololu/orangutan.h>
//#include <avr/interrupt.h>
//#include <inttypes.h> //gives us uintX_t
//#include <stdbool.h>
//
//extern int motor1_speed;
//
////void start_motor() {
	////DDRD |= 0x80;
	////TCCR2A = 0x83;
	////TCCR2B = 0x02;
	////OCR2A = 0;
	////
	//////test_engine();
////}
//
//void set_motor_speed(int speed){
	//if (speed < 0) { MOTOR_REV;	}
	//else { MOTOR_FWD; }
	//
	//motor1_speed = abs(speed);
//
	//if (motor1_speed >= 255)    { OCR2A = MOTOR_MAX; }
	//else if (motor1_speed <= 6) { OCR2A = MOTOR_MIN; }
	//else                        { OCR2A = motor1_speed; }
//}
//
//float degrees_to_count(int degrees){
	//return degrees * COUNT_PER_DEGREES;
//}
//
//int counts_to_degrees(float count){
	//return round(count / COUNT_PER_DEGREES);
//}
//
//void test_engine(){
	//set_motor_speed(120);
	//delay_ms(2000);
	//set_motor_speed(25);
	//delay_ms(2000);
	//set_motor_speed(-25);
	//delay_ms(2000);
	//STOP_MOTOR;
//}
