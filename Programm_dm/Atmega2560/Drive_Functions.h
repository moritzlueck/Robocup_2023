#ifndef DRIVE_FUNCTIONS_H_
#define DRIVE_FUNCTIONS_H_

#define pwm_right				OCR4A
#define pwm_left				OCR4B
#define direction_right_port	PORTG
#define direction_left_port		PORTE
#define direction_right_pin		PORTG5
#define direction_left_pin		PORTE3

void forward (unsigned char, unsigned char);
void backward (unsigned char, unsigned char);
void turn_right (unsigned char);
void turn_left (unsigned char);
void stop (void);

void forward (unsigned char velocity_right, unsigned char velocity_left) {
	pwm_right = velocity_right;
	pwm_left = velocity_left;
	direction_right_port &= ~(1<<direction_right_pin);
	direction_left_port &= ~(1<<direction_left_pin);
}

void backward (unsigned char velocity_right, unsigned char velocity_left) {
	pwm_right = 255 - velocity_right;
	pwm_left = 255 - velocity_left;
	direction_right_port |= (1<<direction_right_pin);
	direction_left_port |= (1<<direction_left_pin);
}

void turn_right (unsigned char velocity) {
	pwm_right = velocity;
	pwm_left = 255 - velocity;
	direction_right_port &= ~(1<<direction_right_pin);
	direction_left_port |= (1<<direction_left_pin);
}

void turn_left (unsigned char velocity) {
	pwm_right = 255 - velocity;
	pwm_left = velocity;
	direction_right_port |= (1<<direction_right_pin);
	direction_left_port &= ~(1<<direction_left_pin);
}

void stop(void) {
	pwm_right = 0;
	pwm_left = 0;
	direction_right_port &= ~(1<<direction_right_pin);
	direction_left_port &= ~(1<<direction_left_pin);
}

#endif /* DRIVE_FUNCTIONS_H_ */