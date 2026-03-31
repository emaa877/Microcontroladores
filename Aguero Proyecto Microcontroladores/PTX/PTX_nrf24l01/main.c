#define ARDUINO 2 //1: Arduino UNO, 2: ARDUINO PRO-MINI

#include "main.h"
#include "nrf24l01.h"

uint8_t ADC_Read(uint8_t adc_to_read);

struct NRF24{
	uint8_t PRX_PTX;
	uint8_t addr;
	uint8_t channel;
	uint8_t state;
	uint8_t payload[BUFFER_SIZE];
};
struct NRF24 nrf24;

int main(void){
	nrf24.PRX_PTX = PTX;  //PTX
	nrf24.addr = 0x01;    //es de 5bytes. Sera address x5, cada byte igual
	nrf24.channel = 0x6E; //110 -> 2510 MHz
	
	PUD_OFF; //bit para activar las resistencias pull-up
	B1_INPUT; B2_INPUT; B3_INPUT; B4_INPUT; S1_INPUT; S1_INPUT;
	B1_PULL_UP; B2_PULL_UP; B3_PULL_UP; B4_PULL_UP; S1_PULL_UP; S2_PULL_UP;
			
	SS_OUTPUT;
	SPI_pin_setup();
	config_SPI_Master(0, 16); //modo=0 (polaridad 0, fase 0) y prescaler= 16 
	
	cli();
	NRF24L01_config(nrf24.PRX_PTX, nrf24.addr, nrf24.channel); 
	sei();
		
	while (1){
		nrf24.payload[0] = ADC_Read(1); //Se leen los 8 MSB
		nrf24.payload[1] = ADC_Read(0);
		nrf24.payload[2] = ADC_Read(2);
		nrf24.payload[3] = ADC_Read(3);
		nrf24.payload[4] = ADC_Read(7);
		nrf24.payload[5] = ADC_Read(6);
		
		nrf24.payload[6]  = B1_TEST ? 1 : 0; //entradas digitales con pull-up, 0: presionadas, 1:sin presionar
		nrf24.payload[7]  = B2_TEST ? 1 : 0;
		nrf24.payload[8]  = B3_TEST ? 1 : 0;
		nrf24.payload[9]  = B4_TEST ? 1 : 0;
		nrf24.payload[10] = S1_TEST ? 1 : 0;
		nrf24.payload[11] = S2_TEST ? 1 : 0;
			
		transmit_payload(nrf24.payload, BUFFER_SIZE);	
		reset_IRQ();
		_delay_ms(20); 
	}
}

uint8_t ADC_Read(uint8_t adc_to_read){
	uint8_t ADC_value;
	ADMUX = adc_to_read;                     //configuro el canal
	SETBIT(ADMUX, REFS0);                    //AREF
	SETBIT(ADMUX, ADLAR);                    //resolucion 10 bits alineado a izq
	
	SETBITS(ADCSRA, 3);                      //prescaler en 8
	SETBIT(ADCSRA, ADEN);                    //habilitamos el ADC
	SETBIT(ADCSRA, ADSC);                    //inicia la conversion

	while(ADCSRA & (1 << ADSC));             //espera a que finalice la conversion

	ADC_value = ADCH;                        //8 MSB
	return ADC_value;
}
