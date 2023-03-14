/*
 * USART_Functions.h
 * Created: 2020
 */ 


#ifndef USART_FUNCTIONS_H_
#define USART_FUNCTIONS_H_

#define hundred				0 //First digit of a 3 digit literal
#define ten					1 //Second digit
#define one					2 //Third digit

unsigned char Trans_data[3]; //3 digits who will be transmitted

void Computing_Transmission_Values (unsigned char); //Compute the literals to be transmitted
void USART_Transmit(unsigned char); //Transmit ASCII to Data Visualizer
void USART_Init(unsigned int); //Initialize the USART-Module


//Transmit literal to Data Visualizer
//If we want to do it very, very fast, we have to divide the transmission into single digits.
//The transmission is not faster!!! But we dont wait while transmitting, we only initialize it!!!
//Initialisation every 256us.
void Computing_Transmission_Values (unsigned char transmitting_data) { //
	Trans_data[hundred] = transmitting_data / 100; //Its the same
	Trans_data[ten] = transmitting_data % 100 /10; //dito
	Trans_data[one] = (transmitting_data % 10) + 48; //A little bit different
	if (!Trans_data[hundred]) { //Now we have to save the results
		Trans_data[hundred] = 32; //blank
		if (!Trans_data[ten]) Trans_data[ten] = 32; //blank
		else Trans_data[ten] = Trans_data[ten] + 48; //ASCII for zero is 48
	}
	else {
		Trans_data[hundred] = Trans_data[hundred] + 48; //See above
		Trans_data[ten] = Trans_data[ten] + 48; //dito
	}
	//But there is no transmission!!!! See "ISR(TIMER2_OVF_vect)"!
}

//Transmission of data from ATMega2560 to computer
void USART_Transmit(unsigned char data) { //Transmission of ASCII data to monitor
	while (!(UCSR0A & (1<<UDRE0))); // Wait for empty transmit buffer, but we are shure, that we dont need to wait for it, because we time it by TMR2
	UDR0 = data; // Put data into buffer, sends the data automaticaly, by the way, we dont need the the MCU
}

//Initialize the transmission of data
void USART_Init(unsigned int ubrr) { //Initialize USART using baud rate
	UBRR0H = (unsigned char)(ubrr>>8); /*Set baud rate */
	UBRR0L = (unsigned char)ubrr; //See above
	UCSR0B = (1<<RXEN0)|(1<<TXEN0); /*Enable receiver and transmitter */
	UCSR0C = (3<<UCSZ00); /* Set frame format: 8 bit data, 1 stop bit */
}

#endif /* USART_FUNCTIONS_H_ */