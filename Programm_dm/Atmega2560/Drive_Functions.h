#ifndef DRIVE_FUNCTIONS_H_
#define DRIVE_FUNCTIONS_H_

#define PWMA			OCR4A
#define PWMB			OCR4B
#define DIRAPORT		PORTG
#define DIRBPORT		PORTE
#define DIRAPIN			PORTG5
#define DIRBPIN			PORTE3

void Forward (unsigned char, unsigned char); 
void Backward (unsigned char, unsigned char);
void Turn_right (unsigned char);
void Turn_left (unsigned char);
void Stop (void);


void Forward (unsigned char velocity_A, unsigned char velocity_B) {
	PWMA = 255 - velocity_A;
	PWMB = 255 - velocity_B;
	DIRAPORT |= (1 << DIRAPIN);
	DIRBPORT |= (1 << DIRBPIN);
}

void Backward (unsigned char velocity_A, unsigned char velocity_B) {
	PWMA =  velocity_A;
	PWMB =  velocity_B;
	DIRAPORT &= ~(1 << DIRBPIN);
	DIRBPORT &= ~(1 << DIRBPIN);
}

void Turn_right (unsigned char velocity) {
	PWMA = 255 - velocity;
	PWMB = velocity;
	DIRAPORT |=  (1 << DIRAPIN);
	DIRBPORT &= ~(1 << DIRBPIN);
}

void Turn_left (unsigned char velocity) {
	PWMA = velocity;
	PWMB = 255 - velocity;
	DIRAPORT &= ~(1 << DIRAPIN);
	DIRBPORT |=  (1 << DIRBPIN);
}

void Stop (void) {
	PWMA = 0;
	PWMB = 0;
	DIRAPORT &= ~(1 << DIRAPIN);
	DIRBPORT &= ~(1 << DIRBPIN);
}

#endif
