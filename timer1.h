
uint16_t prescalerValue_TC1=0;

void configuraPrescaler_TC1(float T){ //T en ms
	T=(float)(T/1000.0);
	float aux=0.0040959375; //es igual a (float)((pow(2,16)-1)/F_CPU) pero ingreso el numero
	//en la variable porque usando esta ecuacion aux vale 0.000000 
	if (T<=aux)			prescalerValue_TC1=1;
	else if(T<=8*aux)   prescalerValue_TC1=8;
	else if(T<=64*aux)  prescalerValue_TC1=64;
	else if(T<=256*aux) prescalerValue_TC1=256;
	else if(T<=1024*aux)prescalerValue_TC1=1024;
	TCCR1B &=~ (7<<CS10);
	switch(prescalerValue_TC1){
		case 0: TCCR1B &=~(7<<CS10); break;
		case 1: TCCR1B |=(1<<CS10); break;
		case 8: TCCR1B |=(2<<CS10); break;
		case 64: TCCR1B |=(3<<CS10); break;
		case 256: TCCR1B |=(4<<CS10); break;
		case 1024: TCCR1B |=(5<<CS10); break;
	}
}

void configura_Modo_TC1(uint8_t modo){
	TCCR1A &=~ (3<<WGM10);
	TCCR1B &=~ (3<<WGM12);
	switch(modo){
		case 0: TCCR1A &=~ (3<<WGM10); TCCR1B &=~ (3<<WGM12);break;
		case 1: TCCR1A |= (1<<WGM10); break;
		case 2: TCCR1A |= (1<<WGM11); break;
		case 3: TCCR1A |= (3<<WGM10); break;
		case 4: TCCR1B |= (1<<WGM12); break;                        //CTC
		case 5: TCCR1A |= (1<<WGM10); TCCR1B |= (1<<WGM12); break;
		case 6: TCCR1A |= (1<<WGM11); TCCR1B |= (1<<WGM12); break;
		case 7: TCCR1A |= (3<<WGM10); TCCR1B |= (1<<WGM12); break;
		case 8: TCCR1B |= (1<<WGM13); break;                        //fase y frecuencia correcta
		case 9: TCCR1A |= (1<<WGM10); TCCR1B |= (1<<WGM13); break;
		case 10: TCCR1A |= (1<<WGM11); TCCR1B |= (1<<WGM13); break;
		case 11: TCCR1A |= (3<<WGM10); TCCR1B |= (1<<WGM13); break;
		case 12: TCCR1B |= (3<<WGM12); break;
		case 13: TCCR1A |= (1<<WGM10); TCCR1B |= (3<<WGM12); break;
		case 14: TCCR1A |= (1<<WGM11); TCCR1B |= (3<<WGM12); break; //fast PWM
		case 15: TCCR1A |= (3<<WGM10); TCCR1B |= (3<<WGM12); break;
		default: break;
	}
}

void configura_ModoSalidas_TC1(uint8_t outA,uint8_t outB){
	TCCR1A &=~ (3<<COM1A0);
	TCCR1A &=~ (3<<COM1B0);
	switch(outA){
		case 0: TCCR1A &=~ (3<<COM1A0); break; //desconectado
		case 1: TCCR1A |= (1<<COM1A0); break;  //toggle
		case 2: TCCR1A |= (1<<COM1A1); break;  //clear
		case 3: TCCR1A |= (3<<COM1A0); break;  //set
		default: break; //printf("Salida OC1A invalida\r\n");
	}
	switch(outB){
		case 0: TCCR1A &=~ (3<<COM1B0); break;
		case 1: TCCR1A |= (1<<COM1B0); break;
		case 2: TCCR1A |= (1<<COM1B1); break;
		case 3: TCCR1A |= (3<<COM1B0); break;
		default: break; //printf("Salida OC1B invalida\r\n");
	}
}

void interrupciones_TC1(uint8_t InputCapt,uint8_t OutputCaptA,uint8_t OutputCaptB,uint8_t Overflow){
	if(InputCapt){
		TIFR1  |= (1<<ICF1); //apaga flag
		TIMSK1 |= (1<<ICIE1); //habilita interrupcion
	}
	if(OutputCaptA){
		TIFR1  |= (1<<OCF1A);
		TIMSK1 |= (1<<OCIE1A);
	}
	if(OutputCaptB){
		TIFR1  |= (1<<OCF1B);
		TIMSK1 |= (1<<OCIE1B);
	}
	if(Overflow){
		TIFR1  |= (1<<TOV1);
		TIMSK1 |= (1<<TOIE1);
	}
}

void configure_PWM_TC1(float T, uint8_t modo, float dutyA, float dutyB){ //T en ms, duty en % de T
	configura_Modo_TC1(modo);
	configuraPrescaler_TC1(T);
	
	T = (float)(T/1000.0);
	float dutyA_percent = dutyA/100.0 * T;
	float dutyB_percent = dutyB/100.0 * T;
	uint8_t factor = 1; //factor que modifica el valor de OCnX segun el modo
	if((modo == 4) || (modo >= 8 && modo <= 11)) factor=2; //modo 8 a 11 son fase correcta-fase y frec correcta
	if((modo >= 5 && modo <= 7) || modo == 14 || modo == 15) factor=1; //5 a 7, 14 y 15 fast PWM
	
	ICR1  = (uint16_t)(T * (F_CPU/(factor*prescalerValue_TC1))-1); 
	OCR1A = (uint16_t)(dutyA_percent*(F_CPU/(factor*prescalerValue_TC1))-1); 
	OCR1B = (uint16_t)(dutyB_percent*(F_CPU/(factor*prescalerValue_TC1))-1);
	
	configura_ModoSalidas_TC1(0,0); //modo desconectado para A y B
	interrupciones_TC1(0,1,1,1);    //uso compare-A para pulsos de motor1 y compare-B para motor2
}
