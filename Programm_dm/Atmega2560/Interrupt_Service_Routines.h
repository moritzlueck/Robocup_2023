#ifndef INTERRUPT_SERVICE_ROUTINES_H_
#define INTERRUPT_SERVICE_ROUTINES_H_

#include "I2C.h"

unsigned char blinken = 1;

unsigned char Counter_I2C_interrupts;	
unsigned int T2ck; //Counter TMR2 overflow
unsigned char T2ck_I2C;
unsigned char delay;

unsigned char lack_of_progress = 0;


/*----------Encoder----------*/
signed int encoder_counter;
unsigned char encoder_counter_ir_contact_right;
unsigned char encoder_counter_ir_contact_left;
unsigned char encoder_counter_ramp;
unsigned char encoder_counter_black_tile = 0;
unsigned char encoder_counter_wall_front = 0;
unsigned char encoder_counter_gap_side = 0;

unsigned char driving = 0;

/*----------Ultrasonic----------*/
unsigned char junk;
unsigned char US_Time; //Runtime of US signal, HI part
unsigned char ultrasonic_front;
unsigned char ultrasonic_right;
unsigned char ultrasonic_left;
unsigned char Counter_US; //Counter TMR2 overflow generating a US-trigger every 32ms
unsigned char ultrasonic_front_finished;
unsigned char ultrasonic_right_finished;
unsigned char ultrasonic_left_finished;

/*----------USART----------*/
unsigned char Counter_Transmission; //Counter of digits of a transmission of a literal
unsigned char Counter_Transmission_Data; //Counter of transmitted data

extern unsigned char Data[10]; //10 Spalten auf dem Data-Visualizer
unsigned char analogue_counter = 0;

unsigned char m, k = 0; //Counter for different clocks
unsigned char TWI_Counter; //Counter for TWI interrupts
unsigned char Register[8] = {0,  1,  2,  3, //First line
	16, 17, 18, 19};//Second line

/*----------ADC----------*/
unsigned char Channel; // analoger Kanal vom ADC
unsigned char Count_channel;
unsigned char Analogue_value[15]; // Nummer der analogen Inputs
unsigned char Clock_ADC; // ADC Timer


/*----------color sensor----------*/
unsigned char Clock_Frequency; //Counter Timer 2 Interrupts for getting the frequency of T0 pin (38, PD7) Color
unsigned char color_counter; // 
unsigned char color_bottom;

extern unsigned char PEC; //

/*** TMR2 overflow Interrupt ***/
ISR(TIMER2_OVF_vect) { //Prescaler TMR2 1:8 = Interrupt every 128 us

	/*----------LED blinken----------*/
	T2ck++; //Counter TMR2 OVF
	if(T2ck == 500) { //very 250ms
		T2ck = 0;
	}
	
	m++; //Counter of Timer2 interrupts
	if (m == 80) { //~10ms, 128us * 80 = 10240us
		//Data[7] = encoder_counter;
		m = 0; //Reset
		k++; //Counter of data to be transmit to LCD
		if (k == 8) k = 0; //8 datas
		TWI_Counter = 0; //Number of TWI interrupts
		i2c_Start(); //Send start condition to TWI bus
		TWDR = (0x20 << 1); //Address and write(0)
		TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE); //Initialize the transmission, enable TWI-Interrupt
	}

	/*----------ultrasonic sensor----------*/
	Counter_US++;
	if (!Counter_US) {
		PORTH |= 0b00000001;
		_delay_us(10);
		PORTH &= 0b11111110;
		TCNT1H = 0; // Timer 1 High Bit auf 0 setzen
		TCNT1L = 0; // Timer 1 Low Bit auf 0 setzen
	}
	
	/*----------Analogue ports----------*/
	Clock_ADC++; //Counter TMR2 OVF for initialising the ADC-Conversion
	if (Clock_ADC == 16) { //Every 2 milliseconds
		Clock_ADC = 0;
		ADMUX = 0b01100000 | Channel; //Bit 3:0 choose the channel
		ADCSRA |= (1 << ADSC); //Start conversion
	}


	/*----------color sensor----------*/
	Clock_Frequency++;
	if (Clock_Frequency == 8) { //1 millisecond
		Clock_Frequency = 0;
		//color_bottom = color_counter;
		//color_counter = 0;
		color_bottom = TCNT0;
		TCNT0 = 0;
	}

	/*----------USART transmition----------*/
	/*if (T2ck & 0b00000001) { //Every 256us,
		Counter_Transmission++; //Four digits will be transmitted
		if (Counter_Transmission == 4) { //
			Counter_Transmission = 0; //Modulo 4
			Counter_Transmission_Data++;  //
			if (Counter_Transmission_Data == 10) { //There will be transmitted ten literals
				Counter_Transmission_Data = 0; //Modulo 10
				USART_Transmit('\n'); //At last write a new line
			}
			Computing_Transmission_Values(Data[Counter_Transmission_Data]); //Computing the next three digits
		}
		//Only initialize, there is no need waiting, while transmission is done
		if (Counter_Transmission == 3) USART_Transmit('\t'); //At last write a tabulator
		else USART_Transmit(Trans_data[Counter_Transmission]); //One digit after another
	}*/
}

ISR(INT2_vect) {
	_delay_ms(30);
	Stop();
	lack_of_progress = 1;
}

/*----------Ultrasonic sensor interrupt----------*/
//ISR(INT2_vect) {
	//unsigned char junk;
	//junk = TCNT1L;
	//ultrasonic_right = TCNT1H;
	//TCNT1H = 0; //Reset TMR1
	//TCNT1L = 0; //Reset
//}

ISR(INT3_vect) {
	junk = TCNT1L;
	ultrasonic_front = TCNT1H;
	TCNT1H = 0; //Reset TMR1
	TCNT1L = 0; //Reset
}

//ISR(INT4_vect) {
	//unsigned char junk;
	//junk = TCNT1L;
	//ultrasonic_left = TCNT1H;
	//TCNT1H = 0; //Reset TMR1
	//TCNT1L = 0; //Reset
//}

/*----------color sensor interrupt----------*/
ISR(INT4_vect) {
	color_counter++;
}

/*----------encoder interrupt----------*/
ISR(INT5_vect)  {
	encoder_counter++;
	if (driving) {
		encoder_counter_ir_contact_right++;
		encoder_counter_ir_contact_left++;
		encoder_counter_ramp++;
		encoder_counter_black_tile++;
		encoder_counter_wall_front++;
		encoder_counter_gap_side++;
	}
}

//Timer 1 Overflow Interrupt
ISR(TIMER1_OVF_vect) { //Prescaler TMR1 1:8 = Interrupt every 32 ms
	ultrasonic_front = 255; //If there is no obstacle
}

/*----------Interrupt on completition of Analog_to_Digital_Conversion----------*/
ISR(ADC_vect) {
	Analogue_value[Count_channel] = ADCH; //HI byte of the resolution
	Count_channel++; //Channels of the ADC
	if (Count_channel == 10) {
		Count_channel = 0;
		//analogue_counter++;
	}
	if (Count_channel > 7) {
		Channel = Count_channel - 8;
		ADCSRB |= (1 << MUX5);
	}
	else {
		Channel = Count_channel;
		ADCSRB &= ~(1 << MUX5);
	}
}

ISR(TWI_vect) {
	switch (TWI_Counter) {
		case 0 : //TWI has been started in ISR Timer2 overflow
			TWDR = Register[k]; //Write the tabulator
			TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA) | (1<<TWIE); //Initialize the transmission, interrupt enable
			TWI_Counter++;
			break;
		case 1 : //TWI has been started in ISR Timer2 overflow
			TWDR = Data[k]; //Write the tabulator
			TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA) | (1<<TWIE); //Initialize the transmission, interrupt enable
			TWI_Counter++;
			break;
		case 2 : //Save PEC, but we don´t need it! And then stop the transmission
			TWCR= (1<<TWINT)|(1<<TWEN)|(1<<TWSTO); //Send stop condition, no more TWI interrupt
			break;
		default :
			TWCR = (1<<TWINT)|(1<<TWEN);
			break;
	}
}

#endif /* INTERRUPT_SERVICE_ROUTINES_H_ */
