//#ifndef __Motors_H
//#define __Motors_H
//
//#include <avr/io.h>         //gives us names for registers
//#include <avr/interrupt.h>
//
//#include <inttypes.h> //gives us uintX_t
//
//#include <string.h>
//
//#define GEAR_RATIO 2249
//#define TIMER1_PERIOD 10
////#define REVERSE_MOTOR (PORTC ^= (1<<7))
//#define MOTOR_FWD (PORTC |= (1<<7))
//#define MOTOR_REV (PORTC &= ~(1<<7))
//#define STOP_MOTOR OCR2A = 0
//#define MOTOR_MIN 8
//#define MOTOR_MAX 255
//#define COUNT_PER_DEGREES (GEAR_RATIO/360)
//
//// function call prototypes
//void start_motor();
//void set_motor_speed(int speed);
//void test_engine();
//#endif