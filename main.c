#define ARDUINO 1 //1: Arduino UNO, 2: ARDUINO PRO-MINI

#include "main.h"
#include "timer0.h"
#include "timer1.h"
#include "nrf24l01.h"

volatile float T=0.8; 
volatile float T_ins=0.8;
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

int main(void){	
	state = init;	
	_delay_ms(500);	
	
	while (1){
		switch(state){
			case init:
				master_config();
				reset_IRQ();
				state = initial_homing;
			    last_state = init;
				break;
			
			case initial_homing:
				if(last_state == init || last_state == automatic_mode){ 
					state = homing_M1;
					motor1_reach_limit_switch = 2;
				}
				else if(last_state == homing_M1) state = homing_M2;
				else state = receive_p;
				
				if(last_state != automatic_mode) last_state = initial_homing; //lo hago de esta manera xq desde homing tengo que volver a initial homing y luego a automatic mode -> necesito saber el estado ante-anterior. Lo hago asi para no generar una nueva variable
				break;
				
			case homing_M1: 
				configure_PWM_TC1(0.8, 50, 50);
				if(motor1_reach_limit_switch == 1) motor1_DIR_CW;
				if(motor1_reach_limit_switch == 2) motor1_DIR_CCW;
				
				motor1_moving = TRUE;
				if(reach_limit_switch_M1(motor1_reach_limit_switch)){ //cambia de estado al llegar al fin de carrera establecido
					motor1_moving = FALSE;
					_delay_ms(300);
					if(last_state == initial_homing){ 
						state = initial_homing; 
						last_state = homing_M1;
					}
					if(last_state == automatic_mode) state = homing_M2;
				}
				break;
				
			case homing_M2: 
				configure_PWM_TC1(0.8, 50, 50);
				if(limit_switch_m2 == 0){
					motor2_DIR_CCW;
					motor2_moving = TRUE;
				}
				else{
					limit_switch_m2 = 0;
					motor2_moving = FALSE;
					_delay_ms(300);
					if(last_state == initial_homing) state = initial_homing;
					if(last_state == automatic_mode){ _delay_ms(500); state = automatic_mode;}
					last_state = homing_M2;
				}
				break;
			
			case receive_p: 
				receive_payload();
				if(nrf24.state == CONNECTED && nrf24.payload[9] == 1 && limit_switch_m1 == 0){
					state = manual_mode;
				}
				if(nrf24.state == CONNECTED && nrf24.payload[9] == 0){
					state = automatic_mode;
					limit_switch_m2 = 0;
				}
				last_state = receive_p;
				break;
			
			case manual_mode:
				//Motor1
				if(nrf24.payload[0]<=107){motor1_moving = TRUE; motor1_DIR_CW;}
				if(nrf24.payload[0]>=147){motor1_moving = TRUE; motor1_DIR_CCW;}
				//Motor2
				if(nrf24.payload[2]>=147){motor2_moving = TRUE; motor2_DIR_CCW;}
				if(nrf24.payload[2]<=107){motor2_moving = TRUE; motor2_DIR_CW;}
				if(nrf24.payload[2]>107 && nrf24.payload[2]<147) motor2_moving = FALSE;
				//Control PWM
				T = (float)(0.933333 - 0.002667*nrf24.payload[4]);
				if(T < 0.4) T = 0.4;
				if(T > 0.8) T = 0.8;			

				if(ramp_motor1==0 && (nrf24.payload[0]<=107 || nrf24.payload[0]>=147)){
					ramp_motor1 = 1; //rampa aceleracion
				} 
				if((ramp_motor1==1 || ramp_motor1==2) && nrf24.payload[0]>107 && nrf24.payload[0]<147){ 
					ramp_motor1 = 3; //rampa desaceleracion
				} 
				if(ramp_motor1==2) T_ins = T;
				configure_PWM_TC1(T_ins, 50, 50);
				state = receive_p;
				last_state = manual_mode;
				break;
			
			case automatic_mode:
				if(last_state == receive_p){ 
					state = initial_homing; 
					last_state = automatic_mode;
				} 
				if(last_state == homing_M2){
					motor1_moving = TRUE;
					motor1_DIR_CW;
					T=0.8;
					ramp_motor1 = 1;
					last_state = automatic_mode;
				}
				if(ramp_motor1 == 2){
					motor2_moving = TRUE;
					motor2_DIR_CCW;
				}
				configure_PWM_TC1(T_ins, 50, 50);	
				break;
		}
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
		if(motor1_DIR_TEST) motor1_DIR_CCW; //return de DIR_TEST: CW!=0  CCW=0
		else motor1_DIR_CW;
		limit_switch_m1 = 0;
		for(int i=0; i<80 ; i++){           //Corre un poco el carro en el sentido opuesto al que venia
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
	if(switchA_TEST){
		motor1_moving = FALSE;	
		limit_switch_m1 = 1;
	}
	INT0_FlagOFF;
}

ISR (INT1_vect){
	if(switchB_TEST){
		motor1_moving = FALSE;	
		limit_switch_m1 = 2;
	}
	INT1_FlagOFF;
}

//Interrupcion IRQ de recepcion de datos
ISR (PCINT0_vect){
	cli();
	CE_OFF; //Dejar de escuchar
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
	
	if(timer_connection == 100){ //t= 100*T_TC0
		timer_connection = 0;
		if(nrf24.state == CONNECTED){
			nrf24.state = DISCONNECTED;
			GREEN_LED_OFF;
			RED_LED_ON;
		}
	}
	SETBIT(TIFR0, TOV0); //apaga flag
}

//Interrupciones de timer1
ISR (TIMER1_COMPA_vect){ 
	if(motor1_moving && (nrf24.state == CONNECTED || state==homing_M1 || state==automatic_mode)){
		 motor1_STEP_LOW;
	}
	SETBIT(TIFR1, OCF1A); //apaga flag
}

ISR (TIMER1_COMPB_vect){
	if(motor2_moving && (nrf24.state == CONNECTED || state==homing_M2 || state==automatic_mode)){
		motor2_STEP_LOW;
    }
	SETBIT(TIFR1, OCF1B); //apaga flag
}

ISR (TIMER1_OVF_vect){
	if(motor1_moving && (nrf24.state == CONNECTED || state==homing_M1 || state==automatic_mode)){ 
		motor1_STEP_HIGH;
		if((ramp_motor1==1 || ramp_motor1==3) || (ramp_motor1==2 && state==automatic_mode)) steps++;
		if(steps % DELTA_STEPS == 0 && ramp_motor1==1) T_ins = 1.0 + (T-1.0)*steps/500.0;
		if(steps % DELTA_STEPS == 0 && ramp_motor1==3) T_ins = T - (T-1.0)*steps/500.0;
		if(steps == 500 && ramp_motor1 == 1){ steps=0; ramp_motor1=2;}
		if(steps == 500 && ramp_motor1 == 3){ steps=0; ramp_motor1=0; motor1_moving = FALSE;}
		if(state==automatic_mode && steps==2400){ steps=0; ramp_motor1=3; state=receive_p;}
	}
	
	if(motor2_moving && (nrf24.state==CONNECTED || state==homing_M2 || state==automatic_mode)){
		motor2_STEP_HIGH;
	}
	SETBIT(TIFR1, TOV1); //apaga flag
}