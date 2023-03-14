/*
 * Init.h
 *
 * Created: 2020
 */

#ifndef INIT_H_
#define INIT_H_

//Initialize the Microcontroller

void Init (void) {
	//Port settings
	//0 = Input, 1 = Output
	DDRA = 0b00000000; //0-7: LCD OUT
	DDRB = 0b10100011; //7: LED_RED;
	DDRC = 0b00000001; //00n.n.,1US-Trig,0Line-R,00Dist1&2,00Line-R&LA
	DDRD = 0b01100000; //011010,0SDA,0SCL
	DDRE = 0b00001110; //00n.n.,1US-Trig,0Line-R,00Dist1&2,00Line-R&LA
	DDRF = 0b00000000; // 0-7: Ir 0-7
	DDRG = 0b00100111; //5: Motor
	DDRH = 0b01111101; //
	DDRL = 0b11111111; // 3 = servo, 5 = button
	DDRK = 0b00000000; // 0-1: Ir 8-9	|	2-3: reflection sensor 0-1

	//enable Pull-up resistors
	PORTA = 0b00001111;
	////Interrupt settings
	////Internal
	////Timer 0 Frequency counter
	TCCR0A = 0b00000000; //0000Normal Port Operation
	TCCR0B = 0b00000110;//00doesn?t matter,00unimpl.,0PWM,110Counter on T0 Pin, falling edge
	TIMSK0 = 0b00000000; //Timer 0 no Interrupt
	////Timer 1 Ultrasonic
	TCCR1A = 0b00000000; //Normal-Mode
	TCCR1B = 0b00000010; //Timer 1 Prescaler 1:8, normal-Mode
	TIMSK1 = 0b00000001; //Interrupt at overflow, appr. 32ms000
	//Timer 2 Time divider
	TCCR2A = 0b00000000; //0000Normal port operation,00unimpl.,00WGM2[1:0]
	TCCR2B = 0b00000010; //0WGM22,010Prescaler 1:8
	TIMSK2 = 0b00000001; //Interrupt at TMR2-OVFL, 128us
	////Timer 4 PWM
	TCCR4A = 0b10100001; //Clear OC4A/OC4B on Compare Match when upcounting. Set OC4A/OC4B on Compare Match when downcounting
	TCCR4B = 0b00000010;//00doesn?t matter,00unimpl.,0PWM,010TMR4-Pres1:8
	TIMSK4 = 0b00000000; //Timer 4 no Interrupt

	//Timer/Interrupt 5
	EICRA = 0b10110000;
	EICRB = 0b00001010;
	EIMSK = 0b00111100; 
	
	

	////ADC settings
	ADMUX  = 0b01100001; //AVCC with external capacitor at AREF pin, left adjusted, A1-Channel
	ADCSRA = 0b11001110; //1enable ADC,1start conversion,1auto trigger,0int. flag,1int. enable,110prescaler 1:64
	//DIDR0 = 0b00001111; //
	//I2C settings
	TWBR = 72; //100.000Hz (100kHz), TWBR =(CPU-Frequ. / 2*Baudrate) - 8, Baudrate = 100kHz
	//TWCR = 0b11000100; //TWINT, clear; TWEA, enable acknowledge; TWSTA, not yet; TWSTO, not yet; TWWC, no collision; TWEN, enable TW-modul; ? TWIE, disable interrupt
	TWSR &= 0b11111100; //no change to the initial values: Prescaler 1:1
	TWCR = (1<<TWEN) | (1<<TWEA);
	//PRR0 = ~(1 << PRTWI); //The PRTWI bit (page 36) must be written to zero to enable the 2-wire serial interface
}

#endif /* INIT_H_ */


