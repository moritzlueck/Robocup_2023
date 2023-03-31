#ifndef INTERRUPT_SERVICE_ROUTINE_H_
#define INTERRUPT_SERVICE_ROUTINE_H_

#include "i2c.h"

#define ultrasonic_trigger_high PORTC |= 0b01110000
#define ultrasonic_trigger_low PORTC &= 0b10001111

#define lack_of_progress_switch !(PINC & (1<<PC7))

/*----------lack of progress handeling----------*/
unsigned char lack_of_progress_flag = 0;

/*----------Encoder----------*/
signed int encoder_counter;
unsigned char encoder_counter_ir_contact_right;
unsigned char encoder_counter_ir_contact_left;
unsigned char encoder_counter_ramp;
unsigned char encoder_counter_black_tile = 0;
unsigned char encoder_counter_wall_front = 0;
unsigned char encoder_counter_gap_side = 0;
unsigned char encoder_counter_victm = 0;
unsigned char driving;

/*----------color sensor----------*/
unsigned char color_clock;
unsigned char color;

/*----------ultrasonic sensor----------*/
unsigned char ultrasonic_clock;
unsigned char ultrasonic_front;
unsigned char ultrasonic_right;
unsigned char ultrasonic_left;

unsigned char junk;

/*----------ADC----------*/
unsigned char adc_clock;
unsigned char adc_channel;
unsigned char adc_channel_counter;
unsigned char analogue_value[10];

/*----------USART----------*/
unsigned char Counter_Transmission;
unsigned char Counter_Transmission_Data;

unsigned char usart_clock;
unsigned char counter_i2c_interrupts;
unsigned char t2ck_i2c;
unsigned char delay;

extern unsigned char data[10];

ISR(TIMER2_OVF_vect) { //every 128 us
	
	if (lack_of_progress_switch) lack_of_progress_flag = 1;
	
	adc_clock++;
	if (adc_clock == 16) {
		adc_clock = 0;
		ADMUX = 0b01100000 | adc_channel;
		ADCSRA |= (1<<ADSC);		
	}
	
	ultrasonic_clock++;
	if (!ultrasonic_clock) {
		ultrasonic_trigger_high;
		_delay_us(10);
		ultrasonic_trigger_low;
		TCNT1H = 0;
		TCNT1L = 0;
	}
	
	color_clock++;
	if (color_clock == 64) {
		color_clock = 0;
		color = TCNT0;
		TCNT0 = 0;
	}
	
	usart_clock++;
	if (usart_clock & 0b00000001) { //Every 256us,
		Counter_Transmission++; //Four digits will be transmitted
		if (Counter_Transmission == 4) { //
			Counter_Transmission = 0; //Modulo 4
			Counter_Transmission_Data++;  //
			if (Counter_Transmission_Data == 10) { //There will be transmitted ten literals
				Counter_Transmission_Data = 0; //Modulo 10
				USART_Transmit('\n'); //At last write a new line
			}
			Computing_Transmission_Values(data[Counter_Transmission_Data]); //Computing the next three digits
		}
		//Only initialize, there is no need waiting, while transmission is done
		if (Counter_Transmission == 3) USART_Transmit('\t'); //At last write a tabulator
		else USART_Transmit(Trans_data[Counter_Transmission]); //One digit after another
	}
	if (!usart_clock) {
		//PINB |= (1<<PINB7);
	}
	
}

ISR(INT2_vect) {
	junk = TCNT1L;
	ultrasonic_front = TCNT1H;
}

ISR(INT3_vect) {
	junk = TCNT1L;
	ultrasonic_right = TCNT1H;
}

ISR(INT4_vect) {
	junk = TCNT1L;
	ultrasonic_left = TCNT1H;
}

ISR(INT5_vect)  {
	encoder_counter++;
	if (driving) {
		encoder_counter_ir_contact_right++;
		encoder_counter_ir_contact_left++;
		encoder_counter_ramp++;
		encoder_counter_black_tile++;
		encoder_counter_wall_front++;
		encoder_counter_gap_side++;
		encoder_counter_victm++;
	}
}

ISR(ADC_vect) {
	analogue_value[adc_channel_counter] = ADCH;
	adc_channel_counter++;
	if (adc_channel_counter == 10) adc_channel_counter = 0;
	if (adc_channel_counter > 7) {
		adc_channel = adc_channel_counter - 8;
		ADCSRB |= (1<<MUX5);
	} else {
		adc_channel = adc_channel_counter;
		ADCSRB &= ~(1<<MUX5);
	}
}

#endif /* INTERRUPT_SERVICE_ROUTINE_H_ */