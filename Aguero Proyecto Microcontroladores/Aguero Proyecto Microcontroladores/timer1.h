
uint16_t prescalerValue_TC1=0;

void configuraPrescaler_TC1(float T){ //T en ms
	T=(float)(T/1000.0);
	float aux = (float)((pow(2,16)-1)/F_CPU);
	if (T<=aux)			 prescalerValue_TC1=1;
	else if(T<=8*aux)    prescalerValue_TC1=8;
	else if(T<=64*aux)   prescalerValue_TC1=64;
	else if(T<=256*aux)  prescalerValue_TC1=256;
	else if(T<=1024*aux) prescalerValue_TC1=1024;
	CLEARBITS(TCCR1B, 7); //3 bits menos significativos: CS10-CS11-CS12 a 0
	switch(prescalerValue_TC1){
		case 0:  CLEARBITS(TCCR1B, 7); break;
		case 1:    SETBITS(TCCR1B, 1); break;
		case 8:    SETBITS(TCCR1B, 2); break;
		case 64:   SETBITS(TCCR1B, 3); break;
		case 256:  SETBITS(TCCR1B, 4); break;
		case 1024: SETBITS(TCCR1B, 5); break;
	}
}

void configura_Modo_TC1(uint8_t modo){
	CLEARBITS(TCCR1A, (BIT(WGM10)|BIT(WGM11)));
	CLEARBITS(TCCR1A, (BIT(WGM12)|BIT(WGM13)));
	switch(modo){
		case 0: CLEARBITS(TCCR1A, (BIT(WGM10)|BIT(WGM11))); CLEARBITS(TCCR1A, (BIT(WGM12)|BIT(WGM13))); break;
		case 1: SETBIT(TCCR1A, WGM10); break;
		case 2: SETBIT(TCCR1A, WGM11); break;
		case 3: SETBITS(TCCR1A, (BIT(WGM10)|BIT(WGM11))); break;
		case 4: SETBIT(TCCR1B, WGM12); break; //CTC
		case 5: SETBIT(TCCR1B, WGM12); SETBITS(TCCR1A, BIT(WGM10)); break;
		case 6: SETBIT(TCCR1B, WGM12); SETBIT(TCCR1A, WGM11); break;
		case 7: SETBIT(TCCR1B, WGM12); SETBITS(TCCR1A, (BIT(WGM10)|BIT(WGM11))); break;
		case 8: SETBIT(TCCR1B, WGM13); break;                        //fase y frecuencia correcta
		case 9: SETBIT(TCCR1B, WGM13); SETBIT(TCCR1A, WGM10); break;
		case 10: SETBIT(TCCR1B, WGM13); SETBIT(TCCR1A, WGM11); break;
		case 11: SETBIT(TCCR1B, WGM13); SETBITS(TCCR1A, (BIT(WGM10)|BIT(WGM11))); break;
		case 12: SETBITS(TCCR1B, (BIT(WGM12)|BIT(WGM13))); break;
		case 13: SETBITS(TCCR1B, (BIT(WGM12)|BIT(WGM13))); SETBIT(TCCR1A, WGM10); break;
		case 14: SETBITS(TCCR1B, (BIT(WGM12)|BIT(WGM13))); SETBIT(TCCR1A, WGM11); break; //fast PWM
		case 15: SETBITS(TCCR1B, (BIT(WGM12)|BIT(WGM13))); SETBITS(TCCR1A, (BIT(WGM10)|BIT(WGM11))); break;
		default: break;
	}
}

void configura_ModoSalidas_TC1(uint8_t outA,uint8_t outB){
	CLEARBITS(TCCR1A, (BIT(COM1A0)|BIT(COM1A1)));
	CLEARBITS(TCCR1A, (BIT(COM1B0)|BIT(COM1B1)));
	switch(outA){
		case 0: CLEARBITS(TCCR1A, (BIT(COM1A0)|BIT(COM1A1))); break; //desconectado
		case 1: SETBIT(TCCR1A, COM1A0); break; //toggle
		case 2: SETBIT(TCCR1A, COM1A1); break; //clear
		case 3: SETBITS(TCCR1A, (BIT(COM1A0)|BIT(COM1A1))); break; //set
		default: break;
	}
	switch(outB){
		case 0: CLEARBITS(TCCR1A, (BIT(COM1B0)|BIT(COM1B1))); break; 
		case 1: SETBIT(TCCR1A, COM1B0); break; 
		case 2: SETBIT(TCCR1A, COM1B1); break; 
		case 3: SETBITS(TCCR1A, (BIT(COM1B0)|BIT(COM1B1))); break;
		default: break;
	}
}

void interrupciones_TC1(uint8_t InputCapt,uint8_t OutputCaptA,uint8_t OutputCaptB,uint8_t Overflow){
	if(InputCapt){
		SETBIT(TIFR1, ICF1); //apaga flag
		SETBIT(TIMSK1, ICIE1); //habilita interrupcion
	}
	if(OutputCaptA){
		SETBIT(TIFR1, OCF1A); 
		SETBIT(TIMSK1, OCIE1A); 
	}
	if(OutputCaptB){
		SETBIT(TIFR1, OCF1B);
		SETBIT(TIMSK1, OCIE1B); 
	}
	if(Overflow){
		SETBIT(TIFR1, TOV1);
		SETBIT(TIMSK1, TOIE1);
	}
}

void configure_PWM_TC1(float T, float dutyA, float dutyB){ //T en ms, duty en % de T
	configura_Modo_TC1(14); //fast PWM
	configuraPrescaler_TC1(T);
	T = (float)(T/1000.0);
	
	ICR1  = (uint16_t)(T * (F_CPU/(prescalerValue_TC1))-1); 
	OCR1A = (uint16_t)(dutyA/100 * ICR1); 
	OCR1B = (uint16_t)(dutyB/100 * ICR1);
}
