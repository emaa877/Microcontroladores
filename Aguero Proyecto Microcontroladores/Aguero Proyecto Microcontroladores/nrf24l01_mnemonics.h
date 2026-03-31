#ifndef _NRF24L01_MNEMONICS_H
#define _NRF24L01_MNEMONICS_H

#ifdef ARDUINO
	#if ARDUINO == 1
		#define CE_CSN_IO   DDRB
		#define CE_CSN_PORT PORTB
		#define CE       PORTB1 //pin 9
		#define CSN      PORTB2 //pin 10	
	#endif
	#if ARDUINO == 2
		#define CE_CSN_IO   DDRD
		#define CE_CSN_PORT PORTD
		#define CE       PORTD5 //pin 5
		#define CSN      PORTD6 //pin 6
	#endif
	#if (ARDUINO == 1 || ARDUINO == 2)
		#define SPI_IO      DDRB
		#define SCK      PORTB5 //pin 13
		#define MOSI     PORTB3 //pin 11
		#define MISO     PORTB4 //pin 12
		#define CE_OUT     (SETBIT(CE_CSN_IO, CE))
		#define CSN_OUT    (SETBIT(CE_CSN_IO, CSN))
		#define SCK_OUT    (SETBIT(SPI_IO, SCK))
		#define MOSI_OUT   (SETBIT(SPI_IO, MOSI))
		#define MISO_IN  (CLEARBIT(SPI_IO, MISO))	
		#define CE_ON      (SETBIT(CE_CSN_PORT, CE))
		#define CE_OFF   (CLEARBIT(CE_CSN_PORT, CE))
		#define CSN_ON     (SETBIT(CE_CSN_PORT, CSN))
		#define CSN_OFF  (CLEARBIT(CE_CSN_PORT, CSN))
	#endif
#endif

#define PRX 1
#define PTX 0
#define CONNECTED 1
#define DISCONNECTED 0

//Configuraciones SPI
#define SPI_ON            (SETBIT(SPCR, SPE))
#define SPI_OFF         (CLEARBIT(SPCR, SPE))
#define SPI_INTER_ON      (SETBIT(SPCR, SPIE))
#define SPI_INTER_OFF   (CLEARBIT(SPCR, SPIE))
#define MASTER_MODE       (SETBIT(SPCR, MSTR))
#define SLAVE_MODE      (CLEARBIT(SPCR, MSTR))
#define END_TRANSMISION  (TESTBIT(SPSR, SPIF))  //1 fin de la transmision
#define SPI_2X            (SETBIT(SPSR, SPI2X)) //se habilita con 1
#define SPI_MSB_FIRST   (CLEARBIT(SPCR, DORD))  //0 MSB primero, 1 LSB primero
#define SPI_LSB_FIRST     (SETBIT(SPCR, DORD))

//Comandos SPI 
#define R_REGISTER          0x00 // 000A AAAA //de los 5bits LSB, se escribe el registro AAAAA
#define W_REGISTER          0x20 // 001A AAAA //de los 5bits LSB, escribe el registro AAAAA
#define R_RX_PAYLOAD        0x61 // 0110 0001
#define W_TX_PAYLOAD        0xA0 // 1010 0000
#define FLUSH_TX            0xE1 // 1110 0001
#define FLUSH_RX            0xE2 // 1110 0010
#define REUSE_TX_PL         0xE3 // 1110 0011
#define R_RX_PL_WID         0x60 // 0110 0000
#define W_ACK_PAYLOAD       0xA8 // 1010 1PPP
#define W_TX_PAYLOAD_NOACK  0xB0 // 1011 0000
#define NOP                 0xFF // 1111 1111

//Mapa de registros
#define CONFIG      0x00
#define MASK_RX_DR  6
#define MASK_TX_DS  5
#define MASK_MAX_RT 4
#define EN_CRC      3
#define CRCO        2
#define PWR_UP      1
#define PRIM_RX     0

#define EN_AA       0x01
#define ENAA_P5     5
#define ENAA_P4     4
#define ENAA_P3     3
#define ENAA_P2     2
#define ENAA_P1     1
#define ENAA_P0     0

#define EN_RXADDR   0x02
#define ERX_P5      5
#define ERX_P4      4
#define ERX_P3      3
#define ERX_P2      2
#define ERX_P1      1
#define ERX_P0      0

#define SETUP_AW    0x03
#define AW          0

#define SETUP_RETR  0x04
#define ARD         4
#define ARC         0

#define RF_CH       0x05

#define RF_SETUP    0x06
#define PLL_LOCK    4 //se recomienda dejarlos en su estado por defecto
#define RF_DR       3
#define RF_PWR      1
#define LNA_HCURR   0 //se recomienda dejarlo en su estado por defecto

#define STATUS              0x07
#define RX_DR               6
#define TX_DS               5
#define MAX_RT              4
#define RX_P_NO_MASK        0x0E
#define STATUS_TX_FULL      0

#define OBSERVE_TX  0x08
#define PLOS_CNT    4
#define ARC_CNT     0

#define RPD         0x09

#define RX_ADDR_P0  0x0A
#define RX_ADDR_P1  0x0B
#define RX_ADDR_P2  0x0C
#define RX_ADDR_P3  0x0D
#define RX_ADDR_P4  0x0E
#define RX_ADDR_P5  0x0F

#define TX_ADDR     0x10

#define RX_PW_P0    0x11
#define RX_PW_P1    0x12
#define RX_PW_P2    0x13
#define RX_PW_P3    0x14
#define RX_PW_P4    0x15
#define RX_PW_P5    0x16

#define FIFO_STATUS         0x17
#define TX_REUSE            6
#define FIFO_TX_FULL        5
#define TX_EMPTY            4
#define RX_FULL             1
#define RX_EMPTY            0

#define DYNPD       0x1C
#define DPL_P5      5
#define DPL_P4      4
#define DPL_P3      3
#define DPL_P2      2
#define DPL_P1      1
#define DPL_P0      0

#define FEATURE     0x1D
#define EN_DPL      2
#define EN_ACK_PAY  1
#define EN_DYN_ACK  0

#endif