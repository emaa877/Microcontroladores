
#define ARDUINO 1 //1: Arduino UNO, 2: ARDUINO PRO-MINI

#include "main.h"
#include "timer0.h"
#include "timer1.h"
#include "nrf24l01.h"

volatile float T=0.8; //periodo del pulso TC1
volatile float T_ins=0.4;
volatile uint8_t motor1_moving = FALSE; //FALSE parado, TRUE andando
volatile uint8_t motor2_moving = FALSE; //FALSE parado, TRUE andando
volatile uint8_t limit_switch_m1 = 0;   //0 ninguno, 1 izq, 2 der. Permite tener un conocimiento de que fin de carrera se alcanzo
volatile uint8_t limit_switch_m2 = 0;   //0 ninguno, 1 llego al optoacoplador
volatile uint8_t timer_connection = 0;
volatile uint8_t motor1_reach_limit_switch = 0;
enum State{init, initial_homing, homing_M1, homing_M2, receive_p, manual_mode, automatic_mode};
volatile uint8_t last_state = 0;
volatile enum State state;
volatile uint16_t steps = 0;
volatile uint8_t ramp_motor1 = 0; //4 estados: 0: vel cero, 1: rampa acel, 2: vel max, 3: rampa desacel
volatile uint16_t steps_between=0;
volatile uint8_t want_to_countM1=0;
volatile uint8_t want_to_countM2=0;

void configure_INT0();
void configure_INT1();
void configure_PCINT0();
void configure_PCINT11();
uint8_t reach_limit_switch_M1(uint8_t);
void homing(uint8_t motor, uint8_t l_switch);
void master_config();

struct NRF24{
	uint8_t PRX_PTX;
	uint8_t addr;
	uint8_t channel;
	uint8_t state;
	uint8_t payload[BUFFER_SIZE];
};
struct NRF24 nrf24;
//printf("Register CONFIG: %d\n", Get_Reg(CONFIG)); //Vale 31 para PRX, 30 para PTX para esta configuracion

//#################UART#########################UART########
#include <stdlib.h> //libreria para atoi
#include <stdbool.h>
int mi_putc(char c,FILE *stream){
	while(!(UCSR0A&(1<<UDRE0))); //transmisión ocupada
	UDR0=c; //Cuando se desocupa, UDR0 puede recibir el nuevo dato c a trasmitir
	return 0;
}
int mi_getc(FILE *stream){
	while(!(UCSR0A&(1<<RXC0)));//recepción incompleta
	return UDR0;//Cuando se completa, se lee UDR0
}
#define fgetc()  mi_getc(&uart_io)
#define fputc(x) mi_putc(x,&uart_io)
FILE uart_io=FDEV_SETUP_STREAM(mi_putc,mi_getc,_FDEV_SETUP_RW);
void configuraUART(int baud_rate,bool Rx,bool Tx){
	UBRR0 =F_CPU/16/baud_rate-1;    //Configura baudrate
	UCSR0A &= ~(1<<U2X0);		    //Velocidad simple
	UCSR0B |=(1<<RXEN0)|(1<<TXEN0); //Habilita recepcion y transmision
	UCSR0C |=(1<<USBS0)|(3<<UCSZ00);//2 bits stop, 8 bits de dato
	stdout = stdin = &uart_io;
	if(Rx){
		UCSR0A &= ~(1<<RXC0); //apaga flag RX completa
		UCSR0B |= (1<<RXCIE0);
	}
	if(Tx){
		UCSR0A &= ~(1<<TXC0); //apaga flag TX completa
		UCSR0B |= (1<<TXCIE0);
	}
}
//##########################################################

int main(void){
	master_config();
	reset_IRQ();
	
	configuraUART(9600, 1, 0);
	printf("Inicializando...\n");
	_delay_ms(500);
	
	for(uint8_t i=0; i<10; i++){
		steps_between=0;
		want_to_countM1 = 0;
		//Homing
		while(limit_switch_m1 != 1){
			motor1_moving = TRUE;
			motor1_DIR_CW;
		}
		motor1_moving = FALSE;
		limit_switch_m1 = 0;
		_delay_ms(600);
		
		//Empezar a contar
		want_to_countM1 = 1;
		while(limit_switch_m1 != 2){
			motor1_moving = TRUE;
			motor1_DIR_CCW;
		}
		motor1_moving = FALSE;
		limit_switch_m1 = 0;
		printf("steps motor1: %"PRIu16"\n", steps_between);
		_delay_ms(500);
	}
	
	//Homing
	while(limit_switch_m2 != 1){
		motor2_moving = TRUE;
		motor2_DIR_CW;
	}
	motor2_moving = FALSE;
	limit_switch_m2 = 0;
	_delay_ms(600);
	
	//Empezar a contar
	want_to_countM2 = 1;
	while(limit_switch_m2 != 1){
		motor2_moving = TRUE;
		motor2_DIR_CW;
	}
	motor2_moving = FALSE;
	limit_switch_m2 = 0;
	printf("steps motor2: %"PRIu16"\n", steps_between);
	steps_between=0;
	_delay_ms(500);
	
	while (1){
		reach_limit_switch_M1(1);
		reach_limit_switch_M1(2);
	}
}

void master_config(){
	nrf24.PRX_PTX = PRX;  //PRX
	nrf24.addr = 0x01;    //address=5*addr (cada byte =)
	nrf24.channel = 0x6E; //110 -> 2510 MHz
	nrf24.state = DISCONNECTED;
	
	PUD_OFF;
	opto_isolator_INPUT; opto_isolator_PULL_UP;
	switchA_INPUT; switchB_INPUT; PCINT0_INPUT;
	motor1_DIR_OUTPUT; motor1_STEP_OUTPUT;
	motor2_DIR_OUTPUT; motor2_STEP_OUTPUT;
	GREEN_LED_OUTPUT; RED_LED_OUTPUT;
	SPI_pin_setup();
	config_SPI_Master(0, 16); //modo=0 y prescaler= 16
	RED_LED_ON;
	
	cli();
	//Timer 0: estado de la conexion
	configure_PWM_TC0(10); //T en ms
	
	//Timer 1: motores
	configure_PWM_TC1(T, 50, 50);   //T en ms, dutyA%, dutyB%
	configura_ModoSalidas_TC1(0,0); //modo desconectado para A y B
	interrupciones_TC1(0,1,1,1);    //compA pulsos motor1 y compB motor2
	
	//NRF24L01
	NRF24L01_config(nrf24.PRX_PTX, nrf24.addr, nrf24.channel);
	
	//Switches
	configure_INT0();
	configure_INT1();
	configure_PCINT0();
	configure_PCINT11();
	sei();
}

void configure_INT0(){ //fin de carrera 1
	INT0_FlancoSubida;
	INT0_FlagOFF;
	INT0_ON;
}

void configure_INT1(){ //fin de carrera 2
	INT1_FlancoSubida;
	INT1_FlagOFF;
	INT1_ON;
}

void configure_PCINT0(){ //pin IRQ del modulo, genera una interrupcion en caso de recepcion de datos
	SETBIT(PCICR, PCIE0);
	SETBIT(PCMSK0, PCINT0);
}

void configure_PCINT11(){ //optoacoplador
	SETBIT(PCICR, PCIE1);
	SETBIT(PCMSK1, PCINT11);
}

uint8_t reach_limit_switch_M1(uint8_t f){
	if(limit_switch_m1 == f){
		_delay_ms(500);                     //Para no cambiar instantaneamente de direccion
		if(motor1_DIR_TEST) motor1_DIR_CCW; //return de DIR_TEST: CW!=0  CCW=0
		else motor1_DIR_CW;
		limit_switch_m1 = 0;
		for(int i=0; i<100 ; i++){          //Corre un poco el carro en el sentido opuesto al que venia
			motor1_STEP_HIGH;               //genero los pulsos de forma manual
			_delay_us(500);
			motor1_STEP_LOW;
			_delay_us(500);
		}
		return TRUE;
	}
	else return FALSE;
}

ISR (PCINT1_vect){
	if(opto_isolator_TEST) limit_switch_m2 = 1;
	SETBIT(PCIFR, PCIF1);
}

//Interrupciones de switches
ISR (INT0_vect){
	_delay_ms(20);
	if(switchA_TEST){
		motor1_moving = FALSE;
		limit_switch_m1 = 1;
	};
	INT0_FlagOFF;
}

ISR (INT1_vect){
	_delay_ms(20);
	if(switchB_TEST){
		motor1_moving = FALSE;
		limit_switch_m1 = 2;
	};
	INT1_FlagOFF;
}

//Interrupcion IRQ de recepcion de datos
ISR (PCINT0_vect){
	cli();
	CE_OFF;                                                        //Dejar de escuchar
	if(! PCINT0_TEST){
		if(Get_Reg(STATUS) & BIT(RX_DR)){
			if(nrf24.state == DISCONNECTED){
				nrf24.state = CONNECTED;
				RED_LED_OFF;
				GREEN_LED_ON;
			}
			Read_NRF(R_RX_PAYLOAD, nrf24.payload, BUFFER_SIZE);
			reset_IRQ();
			timer_connection = 0;
		}
	}
	SETBIT(PCIFR, PCIF0);
	sei();
}

//Interrupcion timer 0
ISR (TIMER0_OVF_vect){
	timer_connection++;
	
	if(timer_connection == 100){           //t= 100*T_TC0
		timer_connection = 0;
		if(nrf24.state == CONNECTED){
			nrf24.state = DISCONNECTED;
			GREEN_LED_OFF;
			RED_LED_ON;
		}
	}
	SETBIT(TIFR0, TOV0);                   //apaga flag
}

//Interrupciones de timer1
ISR (TIMER1_COMPA_vect){
	if(motor1_moving) motor1_STEP_LOW;
	SETBIT(TIFR1, OCF1A); //apaga flag
}

ISR (TIMER1_COMPB_vect){
	if(motor2_moving) motor2_STEP_LOW;
	SETBIT(TIFR1, OCF1B); //apaga flag
}

ISR (TIMER1_OVF_vect){
	if(motor1_moving){ 
		if(want_to_countM1){ steps_between++;}
		motor1_STEP_HIGH;
		if((ramp_motor1==1 || ramp_motor1==3) || (ramp_motor1==2 && state==automatic_mode)) steps++;
		if(steps % DELTA_STEPS == 0 && ramp_motor1==1) T_ins = 1.0 + (T-1.0)*steps/500.0;
		if(steps % DELTA_STEPS == 0 && ramp_motor1==3) T_ins = T - (T-1.0)*steps/500.0;
		if(steps == 500 && ramp_motor1 == 1){ steps=0; ramp_motor1=2;}
		if(steps == 500 && ramp_motor1 == 3){ steps=0; ramp_motor1=0; motor1_moving = FALSE;}
	}
	
	if(motor2_moving){
		motor2_STEP_HIGH;
		if(want_to_countM2){ steps_between++;}
	}
	SETBIT(TIFR1, TOV1); //apaga flag
}