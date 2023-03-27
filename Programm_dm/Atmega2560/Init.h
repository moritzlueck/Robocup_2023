#ifndef INIT_H_
#define INIT_H_

void init() {
	
	/*---------io configuration----------*/
	DDRA = 0b00000000;
	DDRB = 0b10000000;
	DDRC = 0b01111111;
	DDRD = 0b00000000;
	DDRE = 0b00001000;
	DDRF = 0b00000000;
	DDRG = 0b00100000;
	DDRH = 0b00111000;
	DDRL = 0b11111111;
	DDRK = 0b00000000;
	
	/*---------pullup configuration----------*/
	PORTA = 0b00111111;
	PORTC = 0b10000000;
	
	
	/*---------timer configuration----------*/
	TCCR0A = 0b00000000;
	TCCR0B = 0b00000110;
	TIMSK0 = 0b00000000;
	
	TCCR1A = 0b00000000;
	TCCR1B = 0b00000010;
	TIMSK1 = 0b00000000;
	
	TCCR2A = 0b00000000;
	TCCR2B = 0b00000010;
	TIMSK2 = 0b00000001;
	
	TCCR4A = 0b10100001;
	TCCR4B = 0b00000010;
	TIMSK4 = 0b00000000;
	
	/*---------externel interrupt configuration----------*/
	EICRA = 0b10100000;
	EICRB = 0b00001010;
	EIMSK = 0b00111100;

	/*---------adc configuration----------*/
	ADMUX  = 0b01100001;
	ADCSRA = 0b11001110;
	
	/*---------i2c configuration----------*/
	TWBR = 72;
	
	TWSR &= 0b11111100;
	TWCR = (1<<TWEN) | (1<<TWEA);
	
}



#endif /* INIT_H_ */