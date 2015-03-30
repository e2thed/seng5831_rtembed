////#ifndef __Motors_H
////#define __Motors_H
////
////#include <avr/io.h>         //gives us names for registers
////#include <avr/interrupt.h>
////
////#include <inttypes.h> //gives us uintX_t
////
////#include <string.h>
////
/////*
////DDxn : data direction
////PORTxn : drive pin hi or lo (when configured as output)
////PINxn : receive data here (when configured as input)
////*/
////
////// define the data direction registers
////#define DD_REG_SPEED     DDRD
////#define DD_REG_DIRECTION DDRC
////
////// define the ports by which you send signals for MOTOR speed and direction
////#define PORT_SPEED     PORTD
////#define PORT_DIRECTION PORTC
////#define PIN_SPEED      PIND
////#define PIN_DIRECTION  PINC
////#define ENCODER_A  (1<<3)
////#define ENCODER_B  (1<<2)
////
////// define the bit-masks for each port that MOTOR 1 are attached to
////#define BIT_MOTOR1 (1<<7)
////#define BIT_MOTOR2 (1<<6)
////
////#define GEAR_RATIO 2249
////#define TIMER1_PERIOD 10
////
////// define "function" calls for motor
/////*
////MOTOR SET SPEED OCR2A between lowest and highest ( 5? and 250? )
////MOTOR REVERSE TOGGLE PINCx
////MOTOR TURN OFF SPEED = 0
////*/
////#define MOTOR_REVERSE(x) (PORT_DIRECTION ^= BIT_##x)
////
////// function call prototypes
////void init_motor();
////void test_motor1();
////
////#endif