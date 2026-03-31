
uint16_t prescalerValue_TC0=0;

void configuraPrescaler_TC0(float T){ //T en ms
	T=(float)(T/1000.0);
	float aux = (float)((pow(2,8)-1)/F_CPU);
	if (T<=aux)			 prescalerValue_TC0=1;
	else if(T<=8*aux)    prescalerValue_TC0=8;
	else if(T<=64*aux)   prescalerValue_TC0=64;
	else if(T<=256*aux)  prescalerValue_TC0=256;
	else if(T<=1024*aux) prescalerValue_TC0=1024;
	TCCR0A &=~ (7<<CS00);
	switch(prescalerValue_TC0){
		case 0:    CLEARBITS(TCCR0B, 7); break;
		case 1:    SETBITS(TCCR0B, 1); break;
		case 8:    SETBITS(TCCR0B, 2); break;
		case 64:   SETBITS(TCCR0B, 3); break;
		case 256:  SETBITS(TCCR0B, 4); break;
		case 1024: SETBITS(TCCR0B, 5); break;
	}
}

void configura_Modo_TC0(uint8_t modo){
	CLEARBITS(TCCR0A, (BIT(WGM00)|BIT(WGM01)));
	CLEARBIT(TCCR0B, WGM02);
	switch(modo){
		case 0: CLEARBITS(TCCR0A, 3); CLEARBIT(TCCR0A, WGM02); break;
		case 1: SETBITS(TCCR0A, 1); break;
		case 2: SETBITS(TCCR0A, 2); break;
		case 3: SETBITS(TCCR0A, 3); break;
		case 4: SETBIT(TCCR0B, WGM02); break; 
		case 5: SETBIT(TCCR0B, WGM02); SETBITS(TCCR0A, 1); break;
		case 6: SETBIT(TCCR0B, WGM02); SETBITS(TCCR0A, 2); break;
		case 7: SETBIT(TCCR0B, WGM02); SETBITS(TCCR0A, 3); break;
		default: break;
	}
}

void configura_ModoSalidas_TC0(uint8_t outA,uint8_t outB){
	CLEARBITS(TCCR0A, (BIT(COM0A0)|BIT(COM0A1)));
	CLEARBITS(TCCR0A, (BIT(COM0B0)|BIT(COM0B1)));
	switch(outA){
		case 0: CLEARBITS(TCCR0A, (BIT(COM0A0)|BIT(COM0A1))); break; //desconectado
		case 1: SETBIT(TCCR0A, COM0A0); break;  //toggle
		case 2: SETBIT(TCCR0A, COM0A1); break;  //clear
		case 3: SETBITS(TCCR0A, (BIT(COM0A0)|BIT(COM0A1))); break;  //set
		default: break; 
	}
	switch(outB){
		case 0: CLEARBITS(TCCR0A, (BIT(COM0B0)|BIT(COM0B1))); break; //desconectado
		case 1: SETBIT(TCCR0A, COM0B0); break;  //toggle
		case 2: SETBIT(TCCR0A, COM0B1); break;  //clear
		case 3: SETBITS(TCCR0A, (BIT(COM0B0)|BIT(COM0B1))); break;  //set
		default: break; 
	}
}

void interrupciones_TC0(uint8_t OutputCaptA,uint8_t OutputCaptB,uint8_t Overflow){
	if(OutputCaptA){
		SETBIT(TIFR0, OCF0A);
		SETBIT(TIMSK0, OCIE0A);
	}
	if(OutputCaptB){
		SETBIT(TIFR0, OCF0B);
		SETBIT(TIMSK0, OCIE0B);
	}
	if(Overflow){
		SETBIT(TIFR0, TOV0);
		SETBIT(TIMSK0, TOIE0);
	}
}

void configure_PWM_TC0(float T){ 
	CLEARBIT(PRR, PRTIM0); //habilita el modulo del timer counter0
	configura_Modo_TC0(2); //CTC
	configuraPrescaler_TC0(T);
	
	T = (float)(T/1000.0);
	OCR0A = (uint8_t)(T * (F_CPU/(2*prescalerValue_TC0))-1);

	configura_ModoSalidas_TC0(0,0);
	interrupciones_TC0(0,0,1); 
}