
uint16_t prescalerValue_TC0=0;

void configuraPrescaler_TC0(float T){ //T en ms
	T=(float)(T/1000.0);
	float aux = 0.0000159375; //(float)((pow(2,8)-1)/F_CPU);
	if (T<=aux)			 prescalerValue_TC0=1;
	else if(T<=8*aux)    prescalerValue_TC0=8;
	else if(T<=64*aux)   prescalerValue_TC0=64;
	else if(T<=256*aux)  prescalerValue_TC0=256;
	else if(T<=1024*aux) prescalerValue_TC0=1024;
	TCCR0A &=~ (7<<CS00);
	switch(prescalerValue_TC0){
		case 0: TCCR0B &=~(7<<CS00); break;
		case 1: TCCR0B |=(1<<CS00); break;
		case 8: TCCR0B |=(2<<CS00); break;
		case 64: TCCR0B |=(3<<CS00); break;
		case 256: TCCR0B |=(4<<CS00); break;
		case 1024: TCCR0B |=(5<<CS00); break;
	}
}

void configura_Modo_TC0(uint8_t modo){
	TCCR0A &=~ (3<<WGM00);
	TCCR0B &=~ (3<<WGM02);
	switch(modo){
		case 0: TCCR0A &=~ (3<<WGM00); TCCR0B &=~ (1<<WGM02);break;
		case 1: TCCR0A |= (1<<WGM00); break;
		case 2: TCCR0A |= (1<<WGM01); break; //CTC
		case 3: TCCR0A |= (3<<WGM00); break;
		case 4: TCCR0B |= (1<<WGM02); break;                        
		case 5: TCCR0A |= (1<<WGM00); TCCR0B |= (1<<WGM02); break;
		case 6: TCCR0A |= (1<<WGM01); TCCR0B |= (1<<WGM02); break;
		case 7: TCCR0A |= (3<<WGM00); TCCR0B |= (1<<WGM02); break;
		default: break;
	}
}

void configura_ModoSalidas_TC0(uint8_t outA,uint8_t outB){
	TCCR0A &=~ (3<<COM0A0);
	TCCR0A &=~ (3<<COM0B0);
	switch(outA){
		case 0: TCCR0A &=~ (3<<COM0A0); break; //desconectado
		case 1: TCCR0A |= (1<<COM0A0); break;  //toggle
		case 2: TCCR0A |= (1<<COM0A1); break;  //clear
		case 3: TCCR0A |= (3<<COM0A0); break;  //set
		default: break; 
	}
	switch(outB){
		case 0: TCCR0A &=~ (3<<COM0B0); break;
		case 1: TCCR0A |= (1<<COM0B0); break;
		case 2: TCCR0A |= (1<<COM0B1); break;
		case 3: TCCR0A |= (3<<COM0B0); break;
		default: break; 
	}
}

void interrupciones_TC0(uint8_t OutputCaptA,uint8_t OutputCaptB,uint8_t Overflow){
	if(OutputCaptA){
		TIFR0  |= (1<<OCF0A);
		TIMSK0 |= (1<<OCIE0A);
	}
	if(OutputCaptB){
		TIFR0  |= (1<<OCF0B);
		TIMSK0 |= (1<<OCIE0B);
	}
	if(Overflow){
		TIFR0  |= (1<<TOV0);
		TIMSK0 |= (1<<TOIE0);
	}
}

void configure_PWM_TC0(float T){ //T en ms
	CLEARBIT(PRR, PRTIM0); //habilita el modulo del timer counter0
	configura_Modo_TC0(2);
	configuraPrescaler_TC0(T);
	
	T = (float)(T/1000.0);
	OCR0A = (uint8_t)(T * (F_CPU/(2*prescalerValue_TC0))-1);

	
}