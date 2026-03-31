#include "nrf24l01_mnemonics.h"

void config_SPI_Master(uint8_t spi_mode, uint8_t prescaler){
	SPCR = 0;
	switch(spi_mode){ //Modo de funcionamiento: polaridad y fase de bits CPOL-CPHA
		case 0: CLEARBITS(SPCR, (BIT(CPHA)|BIT(CPOL))); break;
		case 1: SETBIT(SPCR, CPHA); break;
		case 2: SETBIT(SPCR, CPOL);	break;
		case 3: SETBITS(SPCR, (BIT(CPHA)|BIT(CPOL))); break;
		default: break;
	}
	
	CLEARBITS(SPCR, (BIT(SPR0)|BIT(SPR1))); //Apago el prescaler y luego enmascaro con OR
	switch(prescaler){
		case 4: CLEARBITS(SPCR, (BIT(SPR0)|BIT(SPR1))); break;
		case 16: SETBIT(SPCR, SPR0); break;
		case 64: SETBIT(SPCR, SPR1); break;
		case 128: SETBITS(SPCR, (BIT(SPR0)|BIT(SPR1))); break;
		default: break;
	}
	MASTER_MODE; SPI_MSB_FIRST; SPI_ON;
	CSN_ON; //En alto xq todavia no hay datos para enviar
	CE_OFF; //En bajo xq no hay nada para enviar/recibir
}

void SPI_pin_setup(){
	CE_OUT;
	CSN_OUT;
	SCK_OUT;
	MOSI_OUT;
	MISO_IN;
}

uint8_t SPI_transfer(uint8_t dato){
	SPDR = dato;      // Comienza la transmision
	while(!END_TRANSMISION){
		_delay_us(1);
	} //Espera que se complete la transmision
	return SPDR;
}

uint8_t Get_Reg(uint8_t reg){ //devuelve el valor del registro
	_delay_us(10);
	CSN_OFF;
	_delay_us(10);
	SPI_transfer(R_REGISTER | reg);
	_delay_us(10);
	reg = SPI_transfer(NOP);
	_delay_us(10);
	CSN_ON;
	return reg;
}
			
void Write_NRF(uint8_t reg, uint8_t *data, uint8_t size_package){ //tamaño del paquete en bytes
	_delay_us(10); //Para asegurarnos que la ultima transferencia fue hace + de 10us
	CSN_OFF;
	_delay_us(10);
	SPI_transfer(W_REGISTER | reg);
	_delay_us(10);

	for(int i=0; i<size_package; i++){
		SPI_transfer(data[i]);
		_delay_us(10);
	}
	CSN_ON;
}

void Read_NRF(uint8_t reg, uint8_t *data_in_reg, uint8_t size_package){
	_delay_us(10);
	CSN_OFF;
	_delay_us(10);
	SPI_transfer(R_REGISTER | reg);
	_delay_us(10);
	
	for(int i=0; i<size_package; i++){
		if(reg != W_TX_PAYLOAD){
			data_in_reg[i] = SPI_transfer(NOP);
			_delay_us(10);
		}
		else{
			SPI_transfer(data_in_reg[i]); //quitar el ELSE
			_delay_us(10);
		}
		
	}
	CSN_ON;
}

void NRF24L01_config(uint8_t PTX_PRX, uint8_t addr, uint8_t channel){
	_delay_ms(100);
	uint8_t val[5];
	
	val[0] = BIT(ENAA_P0);
	Write_NRF(EN_AA, val, 1); //habilita ACK pipe0
	
	val[0] = BIT(ERX_P0);
	Write_NRF(EN_RXADDR, val, 1); //habilita pipe0
	
	val[0] = 0x03;
	Write_NRF(SETUP_AW, val, 1); //5bytes de direccion
	
	val[0] = channel; //frecuencia= 2400+channel [MHz]. Mismo en TX y RX
	Write_NRF(RF_CH, val, 1);
	
	val[0] = (3<<RF_PWR) | BIT(LNA_HCURR);
	Write_NRF(RF_SETUP, val, 1);
	
	for(uint8_t i=0; i<5; i++){
		val[i] = addr; //direccion = addr*5
	}
	Write_NRF(RX_ADDR_P0, val, 5);
	Write_NRF(TX_ADDR, val, 5);
	
	val[0] = BUFFER_SIZE; //nro de bytes en la FIFO RX de pipe0
	Write_NRF(RX_PW_P0, val, 1);
	
	val[0] = (2<<ARD)|BIT(3<<ARC);
	Write_NRF(SETUP_RETR, val, 1);
	
	val[0] =  BIT(MASK_MAX_RT)|BIT(EN_CRC)|BIT(CRCO)|BIT(PWR_UP)|PTX_PRX;
	Write_NRF(CONFIG, val, 1);
	
	_delay_ms(5); //solo necesita 1.5ms para pasar a modo standby (CE=LOW)
}

void transmit_payload(uint8_t *data, uint8_t size_package){
	Write_NRF(FLUSH_TX, data, 0);
	Write_NRF(W_TX_PAYLOAD, data, size_package);
	
	_delay_ms(10);
	CE_ON;
	_delay_us(20); //al menos 10us encendido
	CE_OFF;
	_delay_ms(10);
}

void receive_payload(void){
	CE_ON;
	_delay_ms(20); //Escucha por 20ms
	CE_OFF;
}

void reset_IRQ(void){
	_delay_us(10);
	CSN_OFF;
	_delay_us(10);
	SPI_transfer(W_REGISTER | STATUS);
	_delay_us(10);
	SPI_transfer(0x70); //Resetea todas las IRQ en el registro STATUS = 0b01110000
	_delay_us(10);
	CSN_ON;
}