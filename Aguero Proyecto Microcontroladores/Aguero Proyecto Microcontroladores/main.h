#ifndef MAIN_H_
#define MAIN_H_

#define F_CPU 16000000
#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h> 
#include <avr/interrupt.h>
#include <stdint.h>
#include <inttypes.h>

#define TRUE  1
#define FALSE 0

#define BIT(x) (1<<x)
#define SETBITS(x,y) (x |= y)
#define CLEARBITS(x,y) (x &=~ y)
#define SETBIT(x,y) SETBITS(x, BIT(y))
#define CLEARBIT(x,y) CLEARBITS(x, BIT(y))
#define TESTBIT(x,y)  (x & BIT(y))

#define BUFFER_SIZE 12  //6 [digitales] + 6[analogicos]
#define PUD_OFF  (CLEARBIT(MCUCR, PUD))

#ifdef ARDUINO
	#if ARDUINO == 1
		#define MOTOR1 1
		#define MOTOR2 2
		#define DELTA_STEPS 50
		//Fines de carrera
		#define switchA_INPUT (CLEARBIT(DDRD, DDD2)) //pin2-INT0
		#define switchA_TEST  (TESTBIT(PIND, PIND2))
		#define switchB_INPUT (CLEARBIT(DDRD, DDD3)) //pin3-INT1
		#define switchB_TEST  (TESTBIT(PIND, PIND3))
		#define opto_isolator_INPUT (CLEARBIT(DDRC, DDC3))
		#define opto_isolator_PULL_UP (SETBIT(PORTC, PORTC3))
		#define opto_isolator_TEST (TESTBIT(PINC, PINC3))
		//Configuracion interrupciones switches
		#define INT0_ON           (SETBIT(EIMSK, INT0))
		#define INT0_OFF          (CLEARBIT(EIMSK, INT0))
		#define INT0_FlagOFF      (SETBIT(EIFR, INTF0))
		#define INT0_FlancoSubida (EICRA |= BIT(ISC01)|BIT(ISC00)) 
		#define INT1_ON           (SETBIT(EIMSK, INT1))
		#define INT1_OFF          (CLEARBIT(EIMSK, INT1))
		#define INT1_FlagOFF      (SETBIT(EIFR, INTF1))
		#define INT1_FlancoSubida (EICRA |= BIT(ISC11)|BIT(ISC10))
		//Configuracion pin8, como interrupcion IRQ
		#define PCINT0_INPUT  (CLEARBIT(DDRB, DDB0)) //pin 8
		#define PCINT0_TEST   (TESTBIT(PINB, PINB0))			
		//Configuracion motores. Motor 1          
		#define motor1_DIR         (PORTD4) //pin4
		#define motor1_STEP        (PORTD5) //pin5
		#define motor1_DIR_OUTPUT  (SETBIT(DDRD, motor1_DIR)) 
		#define motor1_DIR_CW      (SETBIT(PORTD, motor1_DIR))   //1
		#define motor1_DIR_CCW     (CLEARBIT(PORTD, motor1_DIR)) //0
		#define motor1_DIR_TEST    (TESTBIT(PORTD, motor1_DIR)) //verifica direccion
		#define motor1_STEP_OUTPUT (SETBIT(DDRD, motor1_STEP)) 
		#define motor1_STEP_HIGH   (SETBIT(PORTD, motor1_STEP))
		#define motor1_STEP_LOW    (CLEARBIT(PORTD, motor1_STEP))
		//Motor 2
		#define motor2_DIR         (PORTD6) //pin6
		#define motor2_STEP        (PORTD7) //pin7
		#define motor2_DIR_OUTPUT  (SETBIT(DDRD, motor2_DIR))
		#define motor2_DIR_CW      (SETBIT(PORTD, motor2_DIR))   //1
		#define motor2_DIR_CCW     (CLEARBIT(PORTD, motor2_DIR)) //0
		#define motor2_DIR_TEST    (TESTBIT(PORTD, motor2_DIR)) //verifica direccion
		#define motor2_STEP_OUTPUT (SETBIT(DDRD, motor2_STEP))
		#define motor2_STEP_HIGH   (SETBIT(PORTD, motor2_STEP))
		#define motor2_STEP_LOW    (CLEARBIT(PORTD, motor2_STEP))
		//LEDs
		#define GREEN_LED_OUTPUT (SETBIT(DDRC, DDC5))
		#define GREEN_LED_ON     (SETBIT(PORTC, PORTC5))
		#define GREEN_LED_OFF     (CLEARBIT(PORTC, PORTC5))
		#define RED_LED_OUTPUT (SETBIT(DDRC, DDC4))
		#define RED_LED_ON     (SETBIT(PORTC, PORTC4))
		#define RED_LED_OFF     (CLEARBIT(PORTC, PORTC4))
	#endif
	
	#if ARDUINO == 2
		#define SS_OUTPUT (SETBIT(DDRB, DDB2)) //pin10 - SS
			
		//Botones
		#define B1_INPUT (CLEARBIT(DDRB, DDB0))
		#define B1_PULL_UP (SETBIT(PORTB, PORTB0))
		#define B1_TEST  (TESTBIT(PINB, PINB0))
	
		#define B2_INPUT (CLEARBIT(DDRB, DDB1))
		#define B2_PULL_UP (SETBIT(PORTB, PORTB1)) 
		#define B2_TEST  (TESTBIT(PINB, PINB1))
		
		#define B3_INPUT (CLEARBIT(DDRD, DDD3))
		#define B3_PULL_UP (SETBIT(PORTD, PORTD3))
		#define B3_TEST  (TESTBIT(PIND, PIND3))
		
		#define B4_INPUT (CLEARBIT(DDRD, DDD2))
		#define B4_PULL_UP (SETBIT(PORTD, PORTD2))
		#define B4_TEST  (TESTBIT(PIND, PIND2))
		
		//Switches
		#define S1_INPUT (CLEARBIT(DDRD, DDD7))
		#define S1_PULL_UP (SETBIT(PORTD, PORTD7))
		#define S1_TEST  (TESTBIT(PIND, PIND7))
		
		#define S2_INPUT (CLEARBIT(DDRD, DDD4))
		#define S2_PULL_UP (SETBIT(PORTD, PORTD4))
		#define S2_TEST  (TESTBIT(PIND, PIND4))
	#endif
#endif

#endif 