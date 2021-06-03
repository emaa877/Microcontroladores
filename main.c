#define ARDUINO 1 //1: Arduino UNO, 2: ARDUINO PRO-MINI

#include "main.h"
#include "timer0.h"
#include "timer1.h"
#include "nrf24l01.h"

volatile uint8_t motor1_moving = FALSE; //FALSE parado, TRUE andando
volatile uint8_t motor2_moving = FALSE; //FALSE parado, TRUE andando
volatile uint8_t limit_switch = 0;      //0 ninguno, 1 izq, 2 der. Permite tener un conocimiento de que fin de carrera se alcanzo
volatile uint8_t count_steps = FALSE;   //flag para contar pasos entre fines de carrera
volatile uint16_t steps = 0;            //nro de pasos entre fines de carrera
volatile uint8_t timer0_count = 0;

void configure_INT0();
void configure_INT1();
void configure_PCINT0();
uint8_t reach_limit_switch(uint8_t);
void homing(uint8_t);
void master_config();

struct NRF24{
	uint8_t PRX_PTX;
	uint8_t addr;
	uint8_t channel;
	uint8_t state;
};
struct NRF24 nrf24;

//Utilizo comunicacion UART para verificar los registros. No es parte del proyecto la comunicacion UART
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
	configuraUART(9600,1,0); //Baudrate, interr RX, interr TX
	printf("...Inicializando...\n");
	
	master_config();
	printf("Register CONFIG: %d\n", Get_Reg(CONFIG)); //Vale 31 para PRX, 30 para PTX para esta configuracion
	reset_IRQ();
	
	while (1){	 
		receive_payload();
		reach_limit_switch(1);
		reach_limit_switch(2);	
	}
}

void master_config(){
	nrf24.PRX_PTX = PRX;  //PRX!!
	nrf24.addr = 0x01; //recordar que es de 5bytes. Sera address x5, cada byte igual
	nrf24.channel = 0x6E; //110 -> 2510 MHz
	nrf24.state = DISCONNECTED;
	
	//Configuracion de los pines E/S
	switchA_INPUT; switchB_INPUT; PCINT0_INPUT;
	motor1_DIR_OUTPUT; motor1_STEP_OUTPUT;
	motor2_DIR_OUTPUT; motor2_STEP_OUTPUT;
	SPI_pin_setup();
	config_SPI_Master(0, 16); //modo=0 (polaridad 0, fase 0) y prescaler= 16
		
	cli();
	//Timer 0: para leer un nuevo dato del modulo cada intervalos de tiempo fijos
	/*configure_PWM_TC0(10); //en ms	
	configura_ModoSalidas_TC0(0,0);
	interrupciones_TC0(0,0,1);
	sei();*/
	//Timer 1: para motores
	configure_PWM_TC1(3.0, 14, 50.0, 50.0); //T, modo, dutyA%, dutyB% //1.4ms min - 5.0ms max
	
	//NRF24L01
	NRF24L01_config(nrf24.PRX_PTX, nrf24.addr, nrf24.channel); 
	
	//Switches
	configure_INT0();
	configure_INT1();
	configure_PCINT0();
	sei();         // Activación global de interrupciones
}

void configure_INT0(){
	INT0_FlancoSubida;
	INT0_FlagOFF;
	INT0_ON;
}

void configure_INT1(){
	INT1_FlancoSubida;
	INT1_FlagOFF;
	INT1_ON;
}

void configure_PCINT0(){ //pin IRQ del modulo, utilizado para generar una interrupcion en caso de recepcion de datos
	SETBIT(PCICR, PCIE0);
	SETBIT(PCMSK0, PCINT0);
}

uint8_t reach_limit_switch(uint8_t f){
	if(!motor1_moving && limit_switch == f){
		_delay_ms(400); //Para no cambiar instantaneamente de direccion
		if(motor1_DIR_TEST) motor1_DIR_CCW; //return de DIR_TEST: CW!=1  CCW=0 
		else motor1_DIR_CW;
		limit_switch = 0;
		for(int i=0; i<20; i++){ //Corre un poco el carro en el sentido opuesto al que venia
			motor1_STEP_HIGH;
			_delay_ms(2);       
			motor1_STEP_LOW;
			_delay_ms(2);
		}
	return TRUE;
	}
	else return FALSE;
}

void homing(uint8_t f){
	if(f==1){ motor1_DIR_CCW;}
	if(f==2){ motor1_DIR_CW;}
		
	motor1_moving = TRUE;
	while(!reach_limit_switch(f)){
		_delay_us(1);
	};
}

//Interrupcion timer 0
/*ISR (TIMER0_OVF_vect){
	timer0_count++;
	if(timer0_count == 1) CE_ON;
	if(timer0_count == 10){
		CE_OFF;
		timer0_count = 0;
	}
	SETBIT(TIFR0, TOV0); //apaga flag
}*/

//Interrupcion IRQ de recepcion de datos
ISR (PCINT0_vect){
	cli();
	CE_OFF; //Dejar de escuchar
	if(! PCINT0_TEST){                    //negado porque se activa por flanco de bajada
		if(Get_Reg(STATUS) & BIT(RX_DR)){ //pregunta si se recibieron datos
			nrf24.state = CONNECTED;
			uint8_t data[BUFFER_SIZE];
			Read_NRF(R_RX_PAYLOAD, data, BUFFER_SIZE); 
			reset_IRQ();
			for(int i=0; i<BUFFER_SIZE; i++){
				printf("%d ", data[i]);
			}
			printf("\n");
		}
	}
	SETBIT(PCIFR, PCIF0);
	sei();
}

//Interrupciones de switches
ISR (INT0_vect){
	_delay_ms(20); //Hay que verificar si el boton continua presionado luego de un intervalo de tiempo, al ser un pulsador mecanico tiene pequeños rebotes
	if(switchA_TEST){
		motor1_moving = FALSE;	//Detiene el motor en cada final de carrera
		limit_switch = 1;
		count_steps = 1;
	};
	INT0_FlagOFF;
}

ISR (INT1_vect){
	_delay_ms(20);
	if(switchB_TEST){
		motor1_moving = FALSE;	//Detiene el motor en cada final de carrera
		limit_switch = 2;
		count_steps = 0;
	};
	INT1_FlagOFF;
}

//Interrupciones de timer1
ISR (TIMER1_COMPA_vect){
	if(motor1_moving) motor1_STEP_LOW;
	if(count_steps) steps++;
	SETBIT(TIFR1, OCF1A); //apaga flag
}

ISR (TIMER1_COMPB_vect){
	if(motor2_moving) motor2_STEP_LOW;
	SETBIT(TIFR1, OCF1B); //apaga flag
}

ISR (TIMER1_OVF_vect){
	if(motor1_moving) motor1_STEP_HIGH;
	else motor1_STEP_LOW;
	
	if(motor2_moving) motor2_STEP_HIGH;
	else motor2_STEP_LOW;
	SETBIT(TIFR1, TOV1); //apaga flag
}