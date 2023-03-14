#elif defined (__AVR_ATmega2560__)
#  include <avr/iom2560.h>

#define F_CPU 16000000UL //16mhz cpu
#include <avr/io.h> //io
#include <util/delay.h> //delay
#include <avr/interrupt.h> //interrupt
#include <util/twi.h> //two wire
#include "Init.h" //init
#include "USART_Functions.h" //usart
#include "Drive_Functions.h" //drive functions
#include "Interrupt_Service_Routines.h" //interrupt
#include "I2C.h" // I2C

/*----------baud definition----------*/
#define FOSC				16000000
#define BAUD				76800
#define MYUBRR			FOSC/16/BAUD-1

/*----------encoder related values----------*/
#define turn_encoder_right 94
#define turn_encoder_left 108
#define cm_steps 140
#define cm_steps_half 70
#define align_max_steps 70
#define align_max_steps_right 47
#define align_max_steps_left 60

#define drive_contact 15
#define drive_contact_ir 20
#define turn_right_contact 15
#define turn_left_contact 20
#define turn_right_contact_ir 25
#define turn_left_contact_ir 30

#define drive_before_ejection 50//40
#define drive_after_black 20
#define drive_after_touch 20
#define drive_after_ramp_up 80
#define drive_after_ramp_down 35
#define drive_after_ir_back 45
#define drive_after_wall_front 10

#define drive_too_few_steps 60

#define encoder_victm  10

#define encodeder_ir_too_few 40

#define encoder_ramp_too_late  100
#define encoder_ramp_in  40
#define encoder_ramp_out 20
#define ramp_long_threshold 300

#define ir_contact_steps 10

/*----------servo values----------*/
#define servo_close 2000
#define servo_close_us 1500
#define servo_open 1250
#define servo_open_us 1000

/*---------- I2C addresses----------*/
#define LCD_Address 0x3F;

/*----------speed related values----------*/
#define pwmR_normal 210
#define pwmR_slow 110
#define pwmL_normal 210
#define pwmL_slow 110

#define pwm_drive_slow 180

#define pwm_align 120

#define pwm_turn 180

#define pwm_turn_power 210

#define pwm_multiplier 1

/*----------sensor related values----------*/
#define wall_threshold_front 35 // maximum distance to wall at front and back
#define wall_threshold_side 35 // maximum distance to wall at both sides

#define wall_side 75 // optimal distance to wall at both sides
#define wall_front 80 // optimal distance to wall in the front
#define wall_back 70 // optimal distance to wall in the back
#define wall_drive_theshold 50	
#define wall_too_far_side 55
#define wall_too_close_side 75
#define wall_too_far_front 75
#define wall_too_close_front 85

#define black_threshold 100

#define color_green_checkpoint_theshold 70 // 170
#define color_green 7
#define color_green_high 17	//	25
#define color_green_low 2
#define color_red 35
#define color_red_high 50
#define color_red_low 20
#define color_blue 0
#define color_none 0
#define color_offset 25

/*----------mapping values----------*/
#define wf 0b00000001
#define wr 0b00000010
#define wb 0b00000100
#define wl 0b00001000
#define wall_map 0b00001111
#define victm_map 0b00010000
#define black_tile_map 0b00100000
#define checkpoint_map 0b01000000
#define known_map 0b10000000

#define start_position 12
#define check_start_tile_threshold 5

#define schiris_dumm 0

/*----------led----------*/
#define led_array					PORTL
#define led_array_on				PORTL |= 0b00011111
#define led_array_off				PORTL &= 0b11100000
#define led_array_1_on				PORTL |= (1 << PORTL0)
#define led_array_2_on				PORTL |= (1 << PORTL1)
#define led_array_3_on				PORTL |= (1 << PORTL2)
#define led_array_4_on				PORTL |= (1 << PORTL3)
#define led_array_5_on				PORTL |= (1 << PORTL4)
#define led_array_1_off				PORTL &= ~(1 << PORTL0)
#define led_array_2_off				PORTL &= ~(1 << PORTL1)
#define led_array_3_off				PORTL &= ~(1 << PORTL2)
#define led_array_4_off				PORTL &= ~(1 << PORTL3)
#define led_array_5_off				PORTL &= ~(1 << PORTL4)

#define	led_red_on					PORTL |= (1 << PORTL5) // pin 44
#define	led_red_off					PORTL &= ~(1 << PORTL5)

#define	led_green_on				PORTL |= (1 << PORTL6) // pin 43
#define	led_green_off				PORTL &= ~(1 << PORTL6)

#define	led_blue_on					PORTL |= (1 << PORTL7) // pin 42
#define	led_blue_off				PORTL &= ~(1 << PORTL7)

#define led_white_on				PORTL |= 0b11100000
#define led_white_off				PORTL &= 0b00011111

/*----------servo----------*/
#define servo_high					PORTH |= (1 << PORTH5)
#define servo_low					PORTH &= ~(1 << PORTH5) 

/*----------ir sensors-----------*/

#define ir_right_front				Analogue_value[0]
#define ir_right_back				Analogue_value[1]
#define ir_left_front				Analogue_value[2]
#define ir_left_back				Analogue_value[3]
#define ir_front_right				Analogue_value[4]
#define ir_front_left				Analogue_value[5]
#define ir_back_right				Analogue_value[6]
#define ir_back_left				Analogue_value[7]

/*----------touch sensors----------*/
#define touch_front_right			!(PINA &(1<<PA0))
#define touch_front_left			!(PINA &(1<<PA1))
#define touch_front					PINA &(0b00000011)
#define touch_back_right			!(PINA &(1<<PA2))
#define touch_back_left				!(PINA &(1<<PA3))
#define touch_back					PINA &(0b00001100)

/*----------angle sensors----------*/
#define angle_up					!(PINA & (1<<PA6))
#define angle_down					!(PINA & (1<<PA7))
#define angle						!(PINA & 0b11000000)

/*----------light sensors----------*/

#define light_right					Analogue_value[8]
#define light_left					Analogue_value[9]

/*----------data visualizer numbers----------*/
unsigned char Data[10];

/*----------orientation variables----------*/
unsigned char drive_steps = 0;
unsigned char turn_counter = 0;
unsigned char straight_counter = 0;
unsigned char overun = 0;
unsigned char black_tile = 0;
unsigned char black_tile_victm = 0;
unsigned char victm = 0;
unsigned char checkpoint = 0;
unsigned char white_tile = 0;
unsigned char last_victm = 1;
unsigned char last_victm_drive = 0;
unsigned char last_ramp = 0;
unsigned char victm_known = 0;
unsigned char black_tile_known = 0;
unsigned char driving_failed = 0;
unsigned char turning_failed = 0;
unsigned char stop_after_ir_side = 0;
//unsigned char ir_mesurement[5][8];
signed char wall_counter_front = 0;
signed char wall_counter_right = 0;
signed char wall_counter_back = 0;
signed char wall_counter_left = 0;
signed char checkpoint_counter_detect = 0;
unsigned char analogue_counter_last = 0;

unsigned char on_ramp = 0;

unsigned char turn_direction = 0;
unsigned char turn_direction_last = 0;

int encoder_counter_victm = 0;
unsigned char ramp = 0; // 0: no ramp	1: ramp up	2: ramp down
int encoder_ramp = 0;
unsigned char contact_counter = 0;
signed char contact_counter_right = 0;
signed char contact_counter_left = 0;
unsigned char ir_contact_right = 0;
unsigned char ir_contact_left = 0;
int encoder_counter_temp = 0;
int encoder_counter_temp_align_ir = 0;
int encoder_counter_temp_align_touch = 0;
int encoder_counter_temp_align_ir_side = 0;
int encoder_counter_before_ramp_in = 0;
int encoder_counter_temp_contact = 0;

unsigned int white_counter;
unsigned int black_counter;

unsigned char rescue_counter = 0;
unsigned char checkpoint_counter = 0;

/*----------mapping variables----------*/
unsigned char area = 0;
unsigned char last_area = 0;
unsigned char area_counter = 0;
signed char position_x = start_position;
signed char position_y = start_position;
signed char position_x_est = start_position;
signed char position_y_est = start_position;
signed char position_next_x = 0;
signed char position_next_y = 0;
signed char position_last_x = 0;
signed char position_last_y = 0;

signed char position_last_checkpoint_x = start_position;
signed char position_last_checkpoint_y = start_position;
signed char direction_last_checkpoint = 0;
unsigned char area_last_checkpoint = 0;
unsigned char area_counter_last_checkpoint = 0;
//unsigned char lack_of_progress = 0;
unsigned char last_lack_of_progress = 0;

unsigned char victm_counter = 0;
unsigned char black_tile_counter = 0;
unsigned char ejection_counter = 0;
unsigned char direction = 0;
unsigned char direction_est = 0;
unsigned char direction_next = 0;
unsigned char direction_last = 0;
unsigned char drive_counter = 0;

unsigned char first_loop = 1;
unsigned char mapping_disabled = 0;
unsigned char mapping_blacktile_disabled = 0;
unsigned char mapping_error_flag = 0;

unsigned char map[(start_position * 2)][(start_position * 2)][4];
unsigned char area_exit[6][5];
unsigned char wall = 0;
unsigned char wall_est = 0;
unsigned char wall_rel = 0;
unsigned char wall_abs = 0;
unsigned char wall_quick = 0;
unsigned char wall_temp = 0;
unsigned char wall_before_turn = 0;

unsigned char ramp_down_counter = 0;
unsigned char ramp_up_counter = 0;

unsigned char button_state = 0;

unsigned char start_tile_check_counter = 0;

/*---------other variables----------*/
unsigned char i;
unsigned char x;
unsigned char y;
unsigned char z;
unsigned int winkel;
	
/*unsigned char color_green_high;
unsigned char color_green_low;
unsigned char color_red_high;
unsigned char color_red_low;
unsigned char color_blue_high;
unsigned char color_blue_low;
unsigned char color_none_high;
unsigned char color_none_low;*/

/*----------functions----------*/
void USART_visualizer(void);
void DV_clear(void);
//void DISP_visualizer(void);
void drive(void);
void drive_ir(void);
void drive_victm(void);
void turn(unsigned char mode, unsigned char power_turn);
void align_ir (unsigned char mode = 0);
void align_ir_side (unsigned char side, unsigned char distance);
void align_touch(void);
//void align_switch(void);
void rescue(void);
void servo_home(void);
void eject();
void color_sensor_mode(unsigned char mode);
void check_for_victm(void);
void check_for_victm_drive(void);
void check_known_victm(void);
void add_victm(void);
bool check_known_black_tile(unsigned char mode);
void add_black_tile(void);
void delay_us(unsigned int duration);
void exit();
void detect_wall(void);
void detect_wall_quick(unsigned char direction_quick);
unsigned char calculate_absolute_wall(unsigned char direction_input, unsigned char wall_input);
unsigned char calculate_relative_wall(unsigned char direction_input, unsigned char wall_input, unsigned char wall_direction = 4);
void handle_map(void);
void check_map(void);
void check_for_checkpoint(void);
void recover_map(void);
//void I2C(void);
void blink(void);
void transmit_map(void);
void clear_black_tile(void);
void return_to_start(void);

/*----------main function----------*/

int main(void) {
	
	/*----------reset motors----------*/
	Stop();

	/*----------initalize----------*/
	Init();

	/*----------initalize USRT----------*/
	USART_Init(MYUBRR);

	/*----------interrupt setup----------*/
	// cli();
	sei();
	
	/*----------servo to home position----------*/
	cli();
	servo_home();
	sei();
	
	/*----------color sensor mode----------*/
	color_sensor_mode(0); // set color filter to green

	led_blue_on;
	_delay_ms(500);
	led_blue_off;
	led_red_on;
	_delay_ms(500);
	led_red_off;
	led_green_on;
	_delay_ms(500);
	led_green_off;

	/*color_green_high = color_green + color_offset;
	color_green_low = color_green - color_offset;
	color_red_high = color_red + color_offset;
	color_red_low = color_red - color_offset;
	color_blue_high = color_blue + color_offset;
	color_blue_low = color_blue - color_offset;
	color_none_high = color_none + color_offset;
	color_none_high = color_none - color_offset;*/

	/*----------Data Visualizer----------*/
	while(0) {
		USART_visualizer();
	}

	/*----------test program----------*/
	while(1){
		//Forward(255,255);
		//turn(0);
		//_delay_ms(3000);
		//eject();
		led_blue_on;
		_delay_ms(1000);
		led_blue_off;
		_delay_ms(1000);
	}

	/*..............................................................................................................................................................*/
	/*:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-main program start-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:*/
	/*''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''*/	
	while (!(PIND & 0b00000100)) {
		//led_green_on;
		Data[0] = color_bottom;
		Data[1] = ultrasonic_front;
		Data[2] = ir_front_right;
		Data[3] = ir_front_left;
		Data[4] = ir_right_front;
		Data[5] = ir_right_back;
		Data[6] = ir_left_front;
		Data[7] = ir_left_back;
		Data[8] = encoder_counter;
		Data[9] = angle_down;//color_bottom;
		
		if(color_bottom > color_green_low && color_bottom < color_green_high) {
			led_red_on;
		} else led_red_off;
		if(color_bottom > color_green_checkpoint_theshold) {
			led_green_on;
		} else led_green_off;
		if(light_right < black_threshold && light_left < black_threshold) {
			led_blue_on;
		} else led_blue_off;

	}
	led_green_off;
	_delay_ms(500);
	lack_of_progress = 0;
	while (1) {
		align_ir();
		check_map();
		USART_visualizer();
		if ((black_tile || (driving_failed && turn_direction == 1 && !last_lack_of_progress)) && !lack_of_progress) {			//check if driving failed or black tile was detected
			if (!(wall_rel & wl) && !check_known_black_tile(3)) { // no wall on left
				turn_direction = 3;
			} else if (!(wall_rel & wb) && !check_known_black_tile(2)) { // no wall behind
				turn_direction = 2;
			} else if (!(wall_rel & wr) && !check_known_black_tile(1)) { // no wall on right
				turn_direction = 1;
			}
		} else if (!lack_of_progress) {
			if(!(wall_rel & wr) && !check_known_black_tile(1)) { // no wall on right
				turn_direction = 1;
			} else if (!(wall_rel & wf) && !check_known_black_tile(0)) { // no wall in front
				turn_direction = 0;
			} else if (!(wall_rel & wl) && !check_known_black_tile(3)) { // no wall on left
				turn_direction = 3;
			} else if (!(wall_rel & wb)) {
				turn_direction = 2;
			} else {
				turn_direction = 4;
			}
		}
		USART_visualizer();
		if (checkpoint && position_x >=  (start_position - 1) && position_x <= (start_position + 1) && position_y >= (start_position - 1)  && position_y <= (start_position + 1) && !((direction + turn_direction) % 4) && !black_tile && drive_counter > 4 && !last_lack_of_progress && !area && (map[start_position][start_position][0] & wall_map) == wall && rescue_counter) exit();
		if (!lack_of_progress) {
			switch (turn_direction) {
				case 0:
					straight_counter++;
					align_ir();
					break;
				case 1:
					straight_counter = 0;
					turn(0, turning_failed);
					align_ir();
					break;
				case 2:
					straight_counter = 0;
					turn(1, turning_failed);
					align_ir();
					turn(1, turning_failed);
					align_ir();
					break;
				case 3:
					straight_counter = 0;
					turn(1, turning_failed);
					break;
				case 4:
					led_red_on;
					break;
			}
		}
		detect_wall_quick(0);
		if (checkpoint && wall_quick == (map[start_position][start_position][0] & wall_map) && rescue_counter /*rescue_counter >= 5* && checkpoint_counter > 3*/ && !lack_of_progress) {
			start_tile_check_counter++;
			direction_est = 0;
			position_x_est = start_position;
			position_y_est = start_position;
			led_green_on;
			_delay_ms(1000);
		}
		if (mapping_error_flag > 3) {
			clear_black_tile();
			led_white_on;
			mapping_error_flag = 0;
		}
		if (victm && !checkpoint && !lack_of_progress) {
			//check_known_victm();
			if (!(map[position_x][position_y][area] & victm_map)) {
				rescue();
				add_victm();
			}
		}
		//victm = 0;
		if (!check_known_black_tile(0) && !turning_failed && !lack_of_progress) {
			drive();
		}
		first_loop = 0;
	}
	/*............................................................................................................................................................*/
	/*:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-main program end-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:-:*/
	/*''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''*/
}

/*----------turn----------*/
void turn(unsigned char mode, unsigned char power_turn) {	// 0: turn right	1: turn left
	USART_visualizer();
	check_for_victm();
	encoder_counter_temp = encoder_counter; // save encoder_value
	encoder_counter = 0; // reset encoder counter
	switch (mode) {
		case 0:	
			direction_last = direction;
			direction += 1;
			if (start_tile_check_counter) direction_est += 1;
			while (encoder_counter < turn_encoder_right && !lack_of_progress) { // turn until counter reaches threshold
				if (!power_turn) Turn_right(pwm_turn);
				else Turn_right(pwm_turn_power);
				check_for_victm();
			}
			break;
		case 1:
			direction_last = direction;
			direction += 3;
			if (start_tile_check_counter) direction_est += 3;
			while (encoder_counter < turn_encoder_left && !lack_of_progress) {	//turn until counter reaches threshold
				if (!power_turn) Turn_left(pwm_turn);
				else Turn_left(pwm_turn_power);
				check_for_victm();
			}
	}
	Stop();
	detect_wall_quick(direction);
	if (wall != wall_quick && !last_ramp && wall != 0b00001010 && wall != 0b00000101) {
		USART_visualizer();
		led_blue_on;
		wall_before_turn = wall;
		align_ir();
		detect_wall();
		if (wall != wall_before_turn) {
			turning_failed++;
			for (i = 0; i < 4; i++) {
				detect_wall_quick(i);
				if (wall_quick == wall_before_turn) {
					direction = i;
					detect_wall();
					break;
				}
			}
		} else turning_failed = 0;
	} else {
		turning_failed = 0;
		led_blue_off;
	}
	//led_array_off;
	//led_array |= (1<<(direction % 4));
	led_array |= calculate_relative_wall(direction, wall, 4);
	USART_visualizer();
	encoder_counter = encoder_counter_temp;
}


/*----------aligns the vehicle to detected wall using the infrared sensors---------*/
void align_ir(unsigned char mode) {
	encoder_counter_temp_align_ir = encoder_counter;
	if ((!mode || mode == 1) && !lack_of_progress) {
		encoder_counter = 0;
		if(ir_right_front > wall_threshold_side && ir_right_back > wall_threshold_side) { //wall right
			if(ir_right_front > ir_right_back) {
				while(ir_right_front > ir_right_back && ir_right_front > wall_threshold_side && ir_right_back > wall_threshold_side && !lack_of_progress) { // front side closer
					if (encoder_counter > align_max_steps_left) {
						//if (!mode) align_touch();
						break;
					}
					Turn_left(pwm_align);
				}
				} else if(ir_right_back > ir_right_front) { // back side closer
				while(ir_right_back > ir_right_front && ir_right_front > wall_threshold_side && ir_right_back > wall_threshold_side && !lack_of_progress) {
					if (encoder_counter > align_max_steps_right) {
						//if (!mode) align_touch();
						break;
					}
					Turn_right(pwm_align);
				}
			}
		}  else if (ir_back_right > wall_threshold_front && ir_back_left > wall_threshold_front) { // wall back
			if (ir_back_right > ir_back_left) {
				while(ir_back_right > ir_back_left && ir_back_right > wall_threshold_front && ir_back_right > wall_threshold_front && !lack_of_progress) { // right side closer
					if (encoder_counter > align_max_steps_left) {
						//if (!mode) align_touch();
						break;
					}
					Turn_left(pwm_align);
				}
			} else if (ir_back_left > ir_back_right) { // left side closer
				while(ir_back_left > ir_back_right && ir_back_right > wall_threshold_front && ir_back_right > wall_threshold_front && !lack_of_progress) {
					if (encoder_counter > align_max_steps_right) {
						//if (!mode) align_touch();
						break;
					}
					Turn_right(pwm_align);
				}
			}
		} else if (ir_left_front > wall_threshold_side && ir_left_back > wall_threshold_side) { // wall left
		if(ir_left_front > ir_left_back) {
			while(ir_left_front > ir_left_back) { // front side closer
				if (encoder_counter > align_max_steps_right && !lack_of_progress) {
					//if (!mode) align_touch();
					break;
				}
				Turn_right(pwm_align);
			}
			} else if (ir_left_back > ir_left_front) { // back side closer
			while (ir_left_back > ir_left_front) {
				if (encoder_counter > align_max_steps_left && !lack_of_progress) {
					//if (!mode) align_touch();
					break;
				}
				Turn_left(pwm_align);
			}
		}
	}  else if (ir_front_right > wall_threshold_front && ir_front_left > wall_threshold_front) { // wall front
		if (ir_front_right > ir_front_left) {
			while(ir_back_right > ir_back_left && ir_front_right > wall_threshold_front && ir_front_right > wall_threshold_front && !lack_of_progress) { // right side closer
				if (encoder_counter > align_max_steps_right) {
					//if (!mode) align_touch();
					break;
				}
				Turn_right(pwm_align);
			}
		} else if (ir_front_left > ir_front_right) { // left side closer
			while(ir_back_left > ir_back_right && ir_front_right > wall_threshold_front && ir_front_right > wall_threshold_front && !lack_of_progress) {
				if (encoder_counter > align_max_steps_left) {
					//if (!mode) align_touch();
					break;
				}
				Turn_left(pwm_align);
			}
		}
	}
	}
	Stop();
	if ((!mode || mode == 2) && !lack_of_progress) {
		encoder_counter = 0;
		if (ir_front_right > wall_threshold_front && ir_front_right > wall_threshold_front) { // wall front
			if ((ir_front_right > wall_front || ir_front_left > wall_front) && !(ramp == 1)) {
				while((ir_front_right > wall_front || ir_front_left > wall_front) && ir_front_right > wall_threshold_front && ir_front_right > wall_threshold_front && !lack_of_progress) {
					if (encoder_counter > align_max_steps) {
						//align_touch();
						break;
					}
					Backward(pwmR_slow, pwmL_slow);
				}
				Stop();
				} else if (ir_front_right < wall_front || ir_front_left < wall_front) {
				while((ir_front_left < wall_too_far_front || ir_front_left < wall_too_far_front) && ir_front_right > wall_threshold_front && ir_front_right > wall_threshold_front && !lack_of_progress) {
					if (encoder_counter > align_max_steps) {
						//align_touch();
						break;
					}
					Forward(pwmR_slow, pwmL_slow);
				}
				Stop();
			}
		} else if (ir_back_right > wall_threshold_front && ir_back_left > wall_threshold_front) { // wall back
			if (ir_back_right > wall_back || ir_back_left > wall_back) {
				while((ir_back_right > wall_back || ir_back_left > wall_back) && ir_back_right > wall_threshold_front && ir_back_right > wall_threshold_front && !lack_of_progress) {
					if (encoder_counter > align_max_steps) {
						//align_touch();
						break;
					}
					Forward(pwmR_slow, pwmL_slow);
				}
				Stop();
			} else if (ir_front_right < wall_back || ir_front_left < wall_back) {
				while((ir_back_left < wall_back || ir_back_left < wall_back) && ir_back_right > wall_threshold_front && ir_back_right > wall_threshold_front && !lack_of_progress) {
					if (encoder_counter > align_max_steps) {
						//align_touch();
						break;
					}
					Backward(pwmR_slow, pwmL_slow);
				}
				Stop();
			}
		}
		if (!mode) align_ir(1);
	}
	encoder_counter = encoder_counter_temp_align_ir;
}

void align_ir_side (unsigned char side, unsigned char distance) {
	encoder_counter_temp_align_ir_side = encoder_counter;
	if (!side) {
		for (i = 0; i <= distance; i++) {
			encoder_counter = 0;
			while (encoder_counter < 5) {
				Turn_right(pwm_turn);
			}
			encoder_counter = 0;
			while (encoder_counter < 10) {
				Backward(pwmR_normal, pwmL_normal);
			}
			encoder_counter = 0;
			while (encoder_counter < 10) {
				Turn_left(pwm_turn);
			}
			encoder_counter = 0;
			while (encoder_counter < 5) {
				Forward(pwmR_normal, pwmL_normal);
			}
			Stop();
		}
	} else if (side == 1) {
		for (i = 0; i <= distance; i++) {
			encoder_counter = 0;
			while (encoder_counter < 5) {
				Turn_left(pwm_turn);
			}
			encoder_counter = 0;
			while (encoder_counter < 5) {
				Backward(pwmR_normal, pwmL_normal);
			}
			encoder_counter = 0;
			while (encoder_counter < 5) {
				Turn_right(pwm_turn);
			}
			encoder_counter = 0;
			while (encoder_counter < 5) {
				Forward(pwmR_normal, pwmL_normal);
			}
			Stop();
		}
	}
	encoder_counter = encoder_counter_temp_align_ir_side;
}

/*----------aligns the vehicle to detected wall using the touch sensors---------*/
void align_touch(void) {
	encoder_counter_temp_align_touch = encoder_counter;
	encoder_counter = 0;
	if (ir_front_right > wall_threshold_front && ir_front_right > wall_threshold_front && !lack_of_progress) { // wall front
		while (touch_back && encoder_counter < align_max_steps && !lack_of_progress) {
			Forward(pwmR_normal, pwmL_normal);
		}
		encoder_counter = 0;
		while (encoder_counter < drive_after_touch && !lack_of_progress) {
			Forward(pwmR_normal, pwmL_normal);
		}
		encoder_counter = 0;
		while(ir_front_right > wall_front && ir_front_left > wall_front && encoder_counter < align_max_steps) {
			Backward(pwmR_slow, pwmL_slow);
		}	
	} else if (ir_back_right > wall_threshold_front && ir_back_right > wall_threshold_front && !lack_of_progress) { // wall back
		while (touch_back && encoder_counter < align_max_steps && !lack_of_progress) {
			Backward(pwmR_normal, pwmL_normal);
		}
		encoder_counter = 0;
		while (encoder_counter < drive_after_touch && !lack_of_progress) {
			Backward(pwmR_normal, pwmL_normal);
		}
		encoder_counter = 0;
		while(ir_back_right > wall_front && ir_back_left > wall_front && encoder_counter < align_max_steps) {
			Forward(pwmR_slow, pwmL_slow && !lack_of_progress);
		}
	}
	Stop();
	encoder_counter = encoder_counter_temp_align_touch;
}

/*----------drive 30cm (one tile)----------*/
void drive() {
	USART_visualizer();
	//if (last_victm_drive) drive_steps = cm_steps - drive_before_ejection;
	//else drive_steps = cm_steps; 
	if (last_victm_drive) encoder_counter = drive_before_ejection;
	else encoder_counter = 0;
	encoder_counter_ir_contact_right = 0;
	encoder_ramp = 0;
	black_tile = 0;
	contact_counter = 0;
	contact_counter_left = 0;
	contact_counter_right = 0;
	ir_contact_right = 0;
	ir_contact_left = 0;
	victm = 0;
	ramp = 0;
	driving = 1;
	encoder_counter_before_ramp_in = 0;
	while((encoder_counter < cm_steps || angle_up) && !lack_of_progress) { // drive until encoder value is reached or wall is detected in front
		
		if(light_left < black_threshold || light_right < black_threshold) { // check for black values
			if (encoder_counter_black_tile > drive_after_black && !lack_of_progress) {
				led_blue_on;
				Stop();
				black_tile = 1;
				overun = encoder_counter;
				add_black_tile();
				encoder_counter = 0;
				while (encoder_counter < overun && !lack_of_progress) {
					Backward(pwmR_slow, pwmR_slow);
				}
				Stop();
				led_blue_off;
				break;
			}
		} else encoder_counter_black_tile = 0;
		if (ir_front_right > wall_drive_theshold && ir_front_left > wall_drive_theshold && ultrasonic_front < 10) {
			led_blue_on;
			if (encoder_counter_wall_front > drive_after_wall_front) break;
		} else encoder_counter_wall_front = 0;
		
		if ((ir_right_front < wall_threshold_side && ir_right_back > wall_threshold_side && encoder_counter > encodeder_ir_too_few) || stop_after_ir_side) {
			if (ir_right_front < wall_threshold_side && ir_right_back < wall_threshold_side) {
				encoder_counter = cm_steps - drive_after_ir_back;
				led_green_on;
				stop_after_ir_side = 0;
			} else stop_after_ir_side = 1;
		}
		
		if (angle_up || angle_down) {
			led_array_5_on;
			if (encoder_counter_ramp > encoder_ramp_in) {
				led_array_5_on;
				on_ramp = 1;
				if (angle_up) {
					ramp = 1;
					encoder_counter_before_ramp_in = encoder_counter;
					while(on_ramp && !lack_of_progress) {
						//Forward(pwmR_normal, pwmL_normal);
						drive_ir();
						if (!angle_up) {
							if (encoder_counter_ramp > encoder_ramp_out) {
								Stop();
								encoder_ramp = encoder_counter;
								encoder_counter = cm_steps - drive_after_ramp_up;
								led_array_5_off;
								ramp_up_counter++;
								on_ramp = 0;
							}
						} else encoder_counter_ramp = 0;
					}
				} else {
					ramp = 2;
					while(on_ramp && !lack_of_progress) {
						//Forward(pwmR_normal, pwmL_normal);
						drive_ir();
						if (!angle_down) {
							if (encoder_counter_ramp > encoder_ramp_out) {
								Stop();
								encoder_ramp = encoder_counter;
								encoder_counter = cm_steps - drive_after_ramp_down;
								led_array_5_off;
								ramp_down_counter++;
								on_ramp = 0;
							}
						} else if ((ir_right_front < wall_threshold_side && ir_right_back > wall_threshold_side) || stop_after_ir_side) {
							if (ir_right_front < wall_threshold_side && ir_right_back < wall_threshold_side) {
								encoder_counter = cm_steps - drive_after_ir_back;
								led_green_on;
								stop_after_ir_side = 0;
								ramp_down_counter++;
								on_ramp = 0;
							} else stop_after_ir_side = 1;
						} else encoder_counter_ramp = 0;
					}
				}
			}
		} else encoder_counter_ramp = 0;
		
		if (contact_counter < 6  && !lack_of_progress) {
			if (ir_front_right < wall_front || ir_front_left > wall_threshold_front || contact_counter_right > 2) encoder_counter_ir_contact_right = 0;	
			else if (encoder_counter_ir_contact_right > ir_contact_steps) {
				Stop();
				encoder_counter_ir_contact_right = 0;
				contact_counter_right++;
				contact_counter_left--;
				//align_ir_side(1, 10);
				encoder_counter_temp_contact = encoder_counter;
				encoder_counter = 0;
				while (encoder_counter < drive_contact && !lack_of_progress) {
					Backward(pwm_align, pwm_align);
				}
				Stop();
				encoder_counter = 0;
				while (encoder_counter < turn_left_contact && !lack_of_progress) {
					Turn_left(pwm_align);
				}
				Stop();
				encoder_counter = encoder_counter_temp_contact - drive_contact * 2;
				//contact_counter++;
			}
			if (touch_front_right && !touch_front_left) {
				Stop();
				encoder_counter_temp_contact = encoder_counter;
				encoder_counter = 0;
				while (encoder_counter < drive_contact && !lack_of_progress) {
					Backward(pwm_align, pwm_align);
				}
				Stop();
				encoder_counter = 0;
				while (encoder_counter < turn_left_contact && !lack_of_progress) {
					Turn_left(pwm_align);
				}
				Stop();
				encoder_counter = encoder_counter_temp_contact - drive_contact * 2;
				contact_counter++;
			}
			if (ir_front_left < wall_front || ir_front_right > (wall_threshold_front - 5) || contact_counter_left > 2) encoder_counter_ir_contact_left = 0;
			else if (encoder_counter_ir_contact_left > ir_contact_steps) {	
				Stop();
				encoder_counter_ir_contact_left = 0;
				contact_counter_left++;
				contact_counter_right--;
				//align_ir_side(0, 10);
				encoder_counter_temp_contact = encoder_counter;
				encoder_counter = 0;
				while (encoder_counter < drive_contact_ir && !lack_of_progress) {
					Backward(pwm_align, pwm_align);
				}
				Stop();
				encoder_counter = 0;
				while (encoder_counter < turn_right_contact_ir && !lack_of_progress) {
					Turn_right(pwm_align);
				}
				Stop();
				encoder_counter = encoder_counter_temp_contact - drive_contact * 2;
				//contact_counter++;
			}
			if (touch_front_left && !touch_front_right) {
				Stop();
				encoder_counter_temp_contact = encoder_counter;
				encoder_counter = 0;
				while (encoder_counter < drive_contact_ir && !lack_of_progress) {
					Backward(pwm_align, pwm_align);
				}
				Stop();
				encoder_counter = 0;
				while (encoder_counter < turn_right_contact_ir && !lack_of_progress) {
					Turn_right(pwm_align);
				}
				Stop();
				encoder_counter = encoder_counter_temp_contact - drive_contact * 2;
				contact_counter++;
			}
			if (contact_counter == 6) {
				align_ir(1);
			}
		}
		check_for_victm_drive();
		drive_ir();
	}
	Stop();
	driving = 0;
	if (ramp && !lack_of_progress) {
		led_green_on;
		_delay_ms(100);
		led_green_off;
		if (encoder_counter_before_ramp_in < drive_too_few_steps && straight_counter && ramp == 1) {
			position_x = position_last_x;
			position_y = position_last_y;
			led_blue_on;
		} else led_blue_off;
		//detect_wall_quick(direction);
		/*if (area && position_x >= (start_position - 1) && position_x <= (start_position + 1) && position_y >= (start_position - 1)  && position_y <= (start_position + 1) && area_exit[area][2] == ((direction + 2) % 4) && area_exit[area][4] != ramp) {
			last_area = area;
			position_last_x = position_x;
			position_last_y = position_y;
			position_x = area_exit[last_area][0];
			position_y = area_exit[last_area][1];
			area = area_exit[last_area][3];
		}
		else {
			area_counter++;
			last_area = area;
			area = area_counter;
			area_exit[area][0] = position_x;
			area_exit[area][1] = position_y;
			area_exit[area][2] = (direction % 4);
			area_exit[area][3] = last_area;
			area_exit[area][4] = ramp;
			position_last_x = position_x;
			position_last_y = position_y;
			position_x = start_position;
			position_y = start_position;
		}
		*/
		switch ((direction % 4)) {
			case 0:
				position_y += 2;
				break;
			case 1:
				position_x += 2;
				break;
			case 2:
				position_y -= 2;
				break;
			case 3:
				position_x -= 2;
				break;
		}
		if (start_tile_check_counter) {
			switch ((direction_est % 4)) {
				case 0:
				position_y_est += 2;
				break;
				case 1:
				position_x_est += 2;
				break;
				case 2:
				position_y_est -= 2;
				break;
				case 3:
				position_x_est -= 2;
				break;
			}
		}
	} else if (encoder_counter > drive_too_few_steps && !black_tile && !lack_of_progress) {
		driving_failed = 0;
		led_blue_off;
		position_last_x = position_x;
		position_last_y = position_y;
		switch ((direction % 4)) {
			case 0:
				position_y++;
				break;
			case 1:
				position_x++;
				break;
			case 2:
				position_y--;
				break;
			case 3:
				position_x--;
				break;
		}
		if (start_tile_check_counter) {
			switch ((direction_est % 4)) {
				case 0:
				position_y_est++;
				break;
				case 1:
				position_x_est++;
				break;
				case 2:
				position_y_est--;
				break;
				case 3:
				position_x_est--;
				break;
			}
		}
	} else if (encoder_counter < drive_too_few_steps  && !lack_of_progress) {
		driving_failed = 1;
		led_blue_on;
	}
	contact_counter = 0;
	encoder_counter = 0;
	stop_after_ir_side = 0;
	last_victm_drive = 0;
	led_green_off;
	led_array_5_off;
	led_red_off;
	drive_counter++;
	USART_visualizer();
}

void drive_victm(void) {
	encoder_counter = 0;
	black_tile_victm = 0;
	driving = 1;
	while(encoder_counter < drive_before_ejection && !lack_of_progress) { // drive until encoder value is reached or wall is detected in front
		if(light_left < black_threshold || light_right < black_threshold) { // check for black values
			if (encoder_counter_black_tile > drive_after_black) {
				led_blue_on;
				Stop();
				led_blue_on;
				eject();
				black_tile_victm = 1;
				black_tile = 1;
				overun = encoder_counter;
				add_black_tile();
				encoder_counter = 0;
				while (encoder_counter < overun) {
					Backward(pwmR_slow, pwmR_slow);
				}
				Stop();
				led_blue_off;
				break;
			}
		} else encoder_counter_black_tile = 0;
		drive_ir();
	}
	Stop();
	driving = 1;
	if (!black_tile_victm) {
		led_red_on;
		eject();
		last_victm_drive = 1;
	} else last_victm_drive = 0;
}

/*----------drives while orienting at wall----------*/
void drive_ir(void) {
	if (ir_right_front > wall_threshold_side && ir_right_back > wall_threshold_side) {
		if (ir_right_front > wall_too_close_side && ir_right_back > wall_too_close_side) {
			Forward(pwm_drive_slow, pwmL_normal);
		} else if (ir_right_front < wall_too_far_side && ir_right_front < wall_too_far_side) {
			Forward(pwmR_normal, pwm_drive_slow);
		} else if ( ir_right_front > ir_right_back) {
			Forward(pwm_drive_slow, pwmL_normal);
		} else if ( ir_right_front < ir_right_back) {
			Forward(pwmR_normal, pwm_drive_slow);
		}
	} else if (ir_left_front > wall_threshold_side && ir_left_back > wall_threshold_side) {
		if (ir_left_front > wall_too_close_side && ir_left_back > wall_too_close_side) {
			Forward(pwmR_normal, pwm_drive_slow);
		} else if (ir_right_front < wall_too_far_side && ir_left_back < wall_too_far_side) {
			Forward(pwm_drive_slow, pwmL_normal);
		} else if ( ir_left_front > ir_left_back) {
			Forward(pwmR_normal, pwm_drive_slow);
		} else if ( ir_left_front < ir_left_back) {
			Forward(pwm_drive_slow, pwmL_normal);
		}
	} else Forward(pwmR_normal, pwmL_normal);
}

/*----------checks if current tile contains victm using the color sensor----------*/
void check_for_victm(void) {
	if (color_bottom > color_green_low && color_bottom < color_green_high) victm = 1;
	//USART_visualizer();
} 

void check_for_victm_drive(void) {
	if (color_bottom > color_green_low && color_bottom < color_green_high && encoder_counter > cm_steps_half) victm = 1;
	//USART_visualizer();
}

/*void check_for_victm(void) {
	color_sensor_mode(0);
	if(color_bottom > color_green_low && color_bottom < color_green_high) {
		led_green_on;
		color_sensor_mode(1);
		if (color_bottom > color_red_low && color_bottom < color_red_high) {
			led_red_on;
			victm = 1;
		} else victm = 0;
	} else {
		victm = 0;
		led_red_off;
		led_green_off;
	}
}*/

/*----------checks if victm has been rescued before----------*/
void check_known_victm(void) {
	if (map[position_x][position_y][area] &= victm_map) {
		led_red_on;
		victm_known = 1;
	} else {
		victm_known = 0;
		led_red_off;
	}
}

/*----------adds current position to victm array----------*/
void add_victm(void) {
	if (!lack_of_progress) {
		map[position_x][position_y][area] |= victm_map;
	}
}

/*----------checks if a black tile is in turn direction----------*/
bool check_known_black_tile(unsigned char mode) {	// 0: front		1: right	2: back		3: left
	if (mapping_disabled || mapping_blacktile_disabled) return 0;
	else {
		direction_next = direction + mode;
		direction_next %= 4;
		switch (direction_next) {
			case 0:
				position_next_x = position_x;
				position_next_y = position_y + 1;
				break;
			case 1:
				position_next_x = position_x + 1;
				position_next_y = position_y;
				break;
			case 2:
				position_next_x = position_x;
				position_next_y = position_y - 1;
				break;
			case 3:
				position_next_x = position_x - 1;
				position_next_y = position_y;
				break;
		}
		if (position_next_x != start_position || position_next_y != start_position) {
			if(map[position_next_x][position_next_y][area] & black_tile_map) {
				black_tile_known = 1;
				led_blue_on;
				} else {
				black_tile_known = 0;
			}
			} else {
			black_tile_known = 0;
		}
		if (black_tile_known) return 1;
		else return 0;
	}
}

/*----------adds current position to black-tile array----------*/
void add_black_tile(void) {
	if (!lack_of_progress) {
		switch ((direction % 4))
		{
			case 0:
				map[position_x][position_y + 1][area] |= black_tile_map;
				break;
			case 1:
				map[position_x + 1][position_y][area] |= black_tile_map;
				break;
			case 2:
				map[position_x][position_y - 1][area] |= black_tile_map;
				break;
			case 3:
				map[position_x - 1] [position_y][area] |= black_tile_map;
				break;
		}
	}
	//black_tile_counter++;
}

/*----------checks if a wall is in the given direction----------*/
void detect_wall (void) {
	wall_counter_front = 0;
	wall_counter_right = 0;
	wall_counter_back = 0;
	wall_counter_left = 0;
	checkpoint_counter_detect = 0;
	analogue_counter = 0;
	analogue_counter_last = 255;
	/*while (analogue_counter < 5) {
		if (analogue_counter != analogue_counter_last) {
			if(ir_front_right > wall_threshold_front && ir_front_left > wall_threshold_front) wall_counter_front++;
			if(ir_right_front > wall_threshold_side && ir_right_back > wall_threshold_side) wall_counter_right++;
			if(ir_back_right > wall_threshold_front && ir_back_left > wall_threshold_front) wall_counter_back++;
			if(ir_left_front > wall_threshold_side && ir_left_back > wall_threshold_side) wall_counter_left++;
			if(color_bottom > color_green_checkpoint_theshold) checkpoint_counter++;
		}
	}*/
	// TEMP!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	for (i = 0; i < 5; i++) {
		if(ir_front_right > wall_threshold_front && ir_front_left > wall_threshold_front) wall_counter_front++;
		if(ir_right_front > wall_threshold_side && ir_right_back > wall_threshold_side) wall_counter_right++;
		if(ir_back_right > wall_threshold_front && ir_back_left > wall_threshold_front) wall_counter_back++;
		if(ir_left_front > wall_threshold_side && ir_left_back > wall_threshold_side) wall_counter_left++;
		if(color_bottom > color_green_checkpoint_theshold) checkpoint_counter_detect++;
		_delay_ms(20);
	}
	wall = 0;
	wall_rel = 0;
	if (wall_counter_front > 4) wall_rel |= wf;
	if (wall_counter_right > 4) wall_rel |= wr;
	if (wall_counter_back > 4) wall_rel |= wb;
	if (wall_counter_left > 4) wall_rel |= wl;
	if (checkpoint_counter_detect > 3){
		 checkpoint = 1;
		 led_white_on;
	} else {
		 checkpoint = 0;
		 led_white_off;
	}
	switch((direction % 4)) {
		case 0:
			wall = wall_rel;
			break;
		case 1:
			if (wall_rel & wf) wall |= wr;
			if (wall_rel & wr) wall |= wb;
			if (wall_rel & wb) wall |= wl;
			if (wall_rel & wl) wall |= wf;
			break;
		case 2:
			if (wall_rel & wf) wall |= wb;
			if (wall_rel & wr) wall |= wl;
			if (wall_rel & wb) wall |= wf;
			if (wall_rel & wl) wall |= wr;
			break;
		case 3:
			if (wall_rel & wf) wall |= wl;
			if (wall_rel & wr) wall |= wf;
			if (wall_rel & wb) wall |= wr;
			if (wall_rel & wl) wall |= wb;
			break;
	}
	if (start_tile_check_counter) {
		wall_est = 0;
		switch((direction_est % 4)) {
			case 0:
			wall_est = wall_rel;
			break;
			case 1:
			if (wall_rel & wf) wall_est |= wr;
			if (wall_rel & wr) wall_est |= wb;
			if (wall_rel & wb) wall_est |= wl;
			if (wall_rel & wl) wall_est |= wf;
			break;
			case 2:
			if (wall_rel & wf) wall_est |= wb;
			if (wall_rel & wr) wall_est |= wl;
			if (wall_rel & wb) wall_est |= wf;
			if (wall_rel & wl) wall_est |= wr;
			break;
			case 3:
			if (wall_rel & wf) wall_est |= wl;
			if (wall_rel & wr) wall_est |= wf;
			if (wall_rel & wb) wall_est |= wr;
			if (wall_rel & wl) wall_est |= wb;
			break;
		}
	}
	led_array_off;
	led_array |= wall_rel;
	USART_visualizer();
}	

void detect_wall_quick(unsigned char direction_quick) {
	wall_quick = 0;
	switch((direction_quick % 4)) {
		case 0:
			if (ir_front_right > wall_threshold_front && ir_front_left > wall_threshold_front) wall_quick |= wf;
			if (ir_right_front > wall_threshold_side && ir_right_back > wall_threshold_side) wall_quick |= wr;
			if (ir_back_right > wall_threshold_front && ir_back_left > wall_threshold_front) wall_quick |= wb;
			if (ir_left_front > wall_threshold_side && ir_left_back > wall_threshold_side) wall_quick |= wl;
			break;
		case 1:
			if (ir_front_right > wall_threshold_front && ir_front_left > wall_threshold_front) wall_quick |= wr;
			if (ir_right_front > wall_threshold_side && ir_right_back > wall_threshold_side) wall_quick |= wb;
			if (ir_back_right > wall_threshold_front && ir_back_left > wall_threshold_front) wall_quick |= wl;
			if (ir_left_front > wall_threshold_side && ir_left_back > wall_threshold_side) wall_quick |= wf;
			break;
		case 2:
			if (ir_front_right > wall_threshold_front && ir_front_left > wall_threshold_front) wall_quick |= wb;
			if (ir_right_front > wall_threshold_side && ir_right_back > wall_threshold_side) wall_quick |= wl;
			if (ir_back_right > wall_threshold_front && ir_back_left > wall_threshold_front) wall_quick |= wf;
			if (ir_left_front > wall_threshold_side && ir_left_back > wall_threshold_side) wall_quick |= wr;
			break;
		case 3:
			if (ir_front_right > wall_threshold_front && ir_front_left > wall_threshold_front) wall_quick |= wl;
			if (ir_right_front > wall_threshold_side && ir_right_back > wall_threshold_side) wall_quick |= wf;
			if (ir_back_right > wall_threshold_front && ir_back_left > wall_threshold_front) wall_quick |= wr;
			if (ir_left_front > wall_threshold_side && ir_left_back > wall_threshold_side) wall_quick |= wb;
			break;
	}
}

unsigned char calculate_absolute_wall(unsigned char direction_input, unsigned char wall_input) {
	switch((direction_input % 4)) {
		case 0:
			wall_abs = wall_input;
			break;
		case 1:
			if (wall_input & wf) wall_abs |= wr;
			if (wall_input & wr) wall_abs |= wb;
			if (wall_input & wb) wall_abs |= wl;
			if (wall_input & wl) wall_abs |= wf;
			break;
		case 2:
			if (wall_input & wf) wall_abs |= wb;
			if (wall_input & wr) wall_abs |= wl;
			if (wall_input & wb) wall_abs |= wf;
			if (wall_input & wl) wall_abs |= wr;
			break;
		case 3:
			if (wall_input & wf) wall_abs |= wl;
			if (wall_input & wr) wall_abs |= wf;
			if (wall_input & wb) wall_abs |= wr;
			if (wall_input & wl) wall_abs |= wb;
			break;
	}
	return wall_abs;
}

unsigned char calculate_relative_wall(unsigned char direction_input, unsigned char wall_input, unsigned char wall_direction) {
	switch (direction_input) {
		case 0:
			switch (wall_direction) {
				case 0:
					return (wall_input & wf);
					break;
				case 1:
					return (wall_input & wr);
					break;
				case 2:
					return (wall_input & wb);
					break;
				case 3:
					return (wall_input & wl);
					break;
				case 4:
					wall_rel = wall_input;
					return wall_rel;
					break;
				default:
					return 0;
					break;
			}
			break;
		case 1:
			switch (wall_direction) {
				case 0:
					return (wall_input & wr);
					break;
				case 1:
					return (wall_input & wb);
					break;
				case 2:
					return (wall_input & wl);
					break;
				case 3:
					return (wall_input & wf);
					break;
				case 4:
					if (wall_input & wf) wall_abs |= wr;
					if (wall_input & wr) wall_abs |= wb;
					if (wall_input & wb) wall_abs |= wl;
					if (wall_input & wl) wall_abs |= wf;
					return wall_rel;
					break;
				default:
					return 0;
					break;
			}
			break;
		case 2:
			switch (wall_direction) {
				case 0:
					return (wall_input & wb);
					break;
				case 1:
					return (wall_input & wl);
					break;
				case 2:
					return (wall_input & wf);
					break;
				case 3:
					return (wall_input & wr);
					break;
				case 4:
					if (wall_input & wf) wall_abs |= wb;
					if (wall_input & wr) wall_abs |= wl;
					if (wall_input & wb) wall_abs |= wf;
					if (wall_input & wl) wall_abs |= wr;
					return wall_rel;
					break;
				default:
					return 0;
					break;
			}
			break;
		case 3:
			switch (wall_direction) {
				case 0:
					return (wall_input & wl);
					break;
				case 1:
					return (wall_input & wf);
					break;
				case 2:
					return (wall_input & wr);
					break;
				case 3:
					return (wall_input & wb);
					break;
				case 4:
					if (wall_input & wf) wall_abs |= wl;
					if (wall_input & wr) wall_abs |= wf;
					if (wall_input & wb) wall_abs |= wr;
					if (wall_input & wl) wall_abs |= wb;
					return wall_rel;
					break;
				default:
					return 0;
					break;
			}
			break;
		default:
			return 0;
			break;
	}
}

void check_map(void) {
	if (lack_of_progress) {
		Stop();
		while (!(PIND & 0b00000100)) {
			led_green_on;
			_delay_ms(500);
			led_green_off;
			_delay_ms(500);
		}
		recover_map();
		//mapping_blacktile_disabled = 1;
		clear_black_tile();
		lack_of_progress = 0;
		last_lack_of_progress = 1;
		victm = 0;
		start_tile_check_counter = 0;
		black_tile = 0;
		drive();
	} else last_lack_of_progress = 0;
	detect_wall();
	if (start_tile_check_counter) {
		USART_visualizer();
		if ((map[position_x_est][position_y_est][0] & wall_map) == wall_est && !lack_of_progress) {
			led_white_off;
			led_green_on;
			_delay_ms(100);
			led_green_off;
			_delay_ms(100);
			led_green_on;
			_delay_ms(100);
			led_green_off;
			start_tile_check_counter++;
		} else {
			led_white_off;
			led_red_on;
			_delay_ms(100);
			led_red_off;
			_delay_ms(100);
			led_red_on;
			_delay_ms(100);
			led_red_off;
			start_tile_check_counter = 0;
		}
	}
	if (start_tile_check_counter >= check_start_tile_threshold && !lack_of_progress) {
		led_green_on;
		return_to_start();
	}
	if (map[position_x][position_y][area] && !lack_of_progress) {
		if ((map[position_x][position_y][area] & wall_map) == wall) {
			mapping_error_flag = 0;
		} else mapping_error_flag++;
		} else if (!lack_of_progress) {
		//map[position_x][position_y][area] &= ~(wall_map);
		map[position_x][position_y][area] |= (wall | known_map);
	}
	if (checkpoint && !last_lack_of_progress) {
		if (!(map[position_x][position_y][area] & checkpoint_map)) {
			checkpoint_counter++;
		}
		map[position_x][position_y][area] |= checkpoint_map;
		if (schiris_dumm) {
			if (!(map[position_x][position_y][area] & checkpoint_map)) {
				position_last_checkpoint_x = position_x;
				position_last_checkpoint_y = position_y;
				direction_last_checkpoint = direction;
				area_last_checkpoint = area;
				area_counter_last_checkpoint = area_counter;
			}
			} else {
			position_last_checkpoint_x = position_x;
			position_last_checkpoint_y = position_y;
			direction_last_checkpoint = direction;
			area_last_checkpoint = area;
			area_counter_last_checkpoint = area_counter;
		}
	}
}

void return_to_start(void) {
	direction = direction_est;
	position_x = position_x_est;
	position_y = position_y_est;
	align_ir();
	turn(0,0);
	align_ir();
	turn(0,0);
	drive();
	turn_direction = 2;
	while (!lack_of_progress) {
		align_ir();
		detect_wall();
		if ((black_tile || (driving_failed && turn_direction == 1)) /*&& !turning_failed*/ && !lack_of_progress) {			//check if driving failed or black tile was detected
			if (!(wall_rel & wr) && !check_known_black_tile(3)) { // no wall on left
				turn_direction = 1;
				} else if (!(wall_rel & wb) && !check_known_black_tile(2)) { // no wall behind
				turn_direction = 2;
				} else if (!(wall_rel & wl) && !check_known_black_tile(1)) { // no wall on right
				turn_direction = 3;
			}
			} else if (!lack_of_progress) {
			if(!(wall_rel & wl) && !check_known_black_tile(1)) { // no wall on right
				turn_direction = 3;
				} else if (!(wall_rel & wf) && !check_known_black_tile(0)) { // no wall in front
				turn_direction = 0;
				} else if (!(wall_rel & wr) && !check_known_black_tile(3)) { // no wall on left
				turn_direction = 1;
				} else if (!(wall_rel & wb)) {
				turn_direction = 2;
				} else {
				turn_direction = 4;
			}
		}
		USART_visualizer();
		if (checkpoint && !lack_of_progress) exit();
		if (!lack_of_progress) {
			switch (turn_direction) {
				case 0:
				straight_counter++;
				align_ir();
				break;
				case 1:
				straight_counter = 0;
				turn(0, turning_failed);
				align_ir();
				break;
				case 2:
				straight_counter = 0;
				turn(1, turning_failed);
				align_ir();
				turn(1, turning_failed);
				align_ir();
				break;
				case 3:
				straight_counter = 0;
				turn(1, turning_failed);
				break;
				case 4:
				led_red_on;
				break;
			}
		}
		//victm = 0;
		if (!lack_of_progress && !turning_failed) {
			drive();
		}
		black_tile_victm = 0;
	}
}

/*----------recovers the position and direction after a lag of progress-----------*/
void recover_map (void) {
	position_x = position_last_checkpoint_x;
	position_y = position_last_checkpoint_y;
	area = area_last_checkpoint;
	area_counter = area_counter_last_checkpoint;
	//detect_wall();
	detect_wall_quick(direction_last_checkpoint);
	if ((map[position_x][position_last_checkpoint_y][area] & wall_map) == 0b00001010 || (map[position_x][position_y][area] & wall_map) == 0b00000101 || (map[position_x][position_y][area] & wall_map) == wall_quick) {
		direction = direction_last_checkpoint;
		led_blue_off;
	} else {
		led_blue_on;
		for (i = 0; i < 4; i++) {
			detect_wall_quick(i);
			if ((map[position_x][position_y][area] & wall_map) == wall_quick) {
				direction = i;
				led_blue_off;
				break;
			}
		}
	}
	USART_visualizer();
}

/*----------changes color filter of the color sensor----------*/
void color_sensor_mode(unsigned char mode) { // 0: Green	1: Red		2: Blue		3: No Filter
		//PORTD |= (1 << PORTD7); // S0 HIGH
		PORTC |= (1 << PORTC0);
		PORTG &= ~(1 << PORTG2); // S1 LOW
	if (mode == 0) { // Green
		PORTG |= (1 << PORTG1); // S2 HIGH
		PORTG |= (1 << PORTG0); // S3 HIGH
	} else if(mode == 1) {	// Red
		PORTG &= ~(1 << PORTG1); // S2 LOW
		PORTG &= ~(1 << PORTG0); // S3 LOW
	} else if(mode == 2) { // Blue
		PORTG &= ~(1 << PORTG1); // S2 LOW
		PORTG |= (1 << PORTG0); // S3 HIGH
	} else if(mode == 3) {
		PORTG |= (1 << PORTG1); // S2 HIGH
		PORTG &= ~(1 << PORTG0); // S3 LOW
	}
}

/*-----------servo to home position----------*/
void servo_home(void) {
	for (unsigned char i = 0; i < 250; i++) {
		servo_high;
		_delay_us( servo_close );
		servo_low;
		_delay_ms( 1 );
	}
}

/*----------ejects rescue kit----------*/
void eject() {
	cli();
	servo_home();
	for (unsigned char i = 0; i < 250; i++) {
		servo_high;
		_delay_us( servo_open );
		servo_low;
		_delay_ms( 1 );
	}
	for (winkel = servo_open_us; winkel <= servo_close_us; winkel += 2) {
		servo_high;
		delay_us( winkel );
		servo_low;
		_delay_ms( 1 );
	}
	servo_home();
	sei();
}

/*----------blinks and ejects rescue kit----------*/
void rescue() {
	blink();
	//led_array_on; // ONLY FOR TESTING
	//_delay_ms(1000); // ONLY FOR TESTING
	//led_array_off; // ONLY FOR TESTING
	if (rescue_counter <= 12 && !lack_of_progress) {
		align_ir();
		if (ir_back_right < wall_threshold_front || ir_back_left < wall_threshold_front) {
			//encoder_counter = 0;
			//while (encoder_counter < drive_before_ejection && !lack_of_progress) {
				//Forward(pwmR_normal, pwmL_normal);
			//}
			//Stop();
			//last_victm_drive = 1;
			drive_victm();
		} else eject();
		//eject();
		//_delay_ms(500); // ONLY FOR TESTING
	}
	rescue_counter++;
}

void blink() {
	led_array_on;
	_delay_ms(100);
	led_array_off;
	_delay_ms(100);
	led_array_on;
	_delay_ms(100);
	led_array_off;
	led_array_1_on;
	_delay_ms(100);
	led_array_2_on;
	_delay_ms(100);
	led_array_3_on;
	_delay_ms(100);
	led_array_4_on;
	_delay_ms(100);
	led_array_5_on;
	for (unsigned char b = 0; b < 5; b++) {
		if (lack_of_progress) break;
		_delay_ms(860);
		led_array &= ~(1 << b);
	}
	//_delay_ms(860);
	//led_array_1_off;
	//_delay_ms(860);
	//led_array_2_off;
	//_delay_ms(860);
	//led_array_3_off;
	//_delay_ms(860);
	//led_array_4_off;
	//_delay_ms(860);
	//led_array_5_off;
}

/*----------the action which is executed if the robot reaches the start----------*/
void exit(void) {
	for(i = 0; i < 15; i++) {
		if (lack_of_progress) {
			break;
		}
		led_array_off;
		led_array_1_on;
		_delay_ms(200);
		led_array_1_off;
		led_array_2_on;
		_delay_ms(200);
		led_array_2_off;
		led_array_3_on;
		_delay_ms(200);
		led_array_3_off;
		led_array_4_on;
		_delay_ms(200);
		led_array_4_off;
	}
	_delay_ms(50);
	if (!lack_of_progress) {
		while (PIND & 0b00000100) {
			led_green_on;
		}
		led_green_off;
		_delay_ms(100);
		DV_clear();
		while (1) {
			for (i = 0; i < (start_position * 2); i++) {
				for (z = 0; z < (start_position * 2); z++) {
					if (map[i][z][0]) {
						Data[0] = i;
						Data[1] = z;
						Data[2] = map[i][z][area];
						while ((PIND & 0b00000100) == button_state);
						_delay_ms(100);
						button_state = (PIND & 0b00000100);
					}
				}
			}
		}	
	}
}

void clear_black_tile(void) {
	for (x = 0; x < (start_position * 2); x++) {
		for (y = 0; y < (start_position * 2); y++) {
			for (z = 0; z < 3; z++) {
				if (map[x][y][z] & black_tile_map) {
					map[x][y][z] &= ~(black_tile_map);
				}
			}
		}
	}
}

/*----------custom delay----------*/
void delay_us(unsigned int duration) {
	for (unsigned int d = 0; d <= duration; d++) {
		_delay_us(1);
	}
}

void DV_clear(void) {
	for (i = 0; i < sizeof(Data); i++) {
		Data[i] = 0;
	}
}

/*-----------usart visualizer----------*/
/*void USART_visualizer(void) {
	Data[0] = ir_front_right;
	Data[1] = ir_front_left;//ir_right_back;
	Data[2] = ir_right_front;//ir_left_front;
	Data[3] = ir_right_back;//ir_left_back;
	Data[4] = ir_back_right;
	Data[5] = ir_back_left;
	Data[6] = ir_left_front;
	Data[7] = ir_left_back;
	Data[8] = encoder_counter;
	Data[9] = US_Time;
}*/
void USART_visualizer(void) {
	Data[0] = (direction % 4);
	Data[1] = position_x;
	Data[2] = position_y;
	Data[3] = map[position_x][position_y][area];
	Data[4] = wall;//turn_direction;//direction_last_checkpoint;
	Data[5] = position_last_checkpoint_x;
	Data[6] = position_last_checkpoint_y;
	Data[7] = mapping_error_flag;//encoder_counter;
	Data[8] = 0;
	Data[9] = 0;
}
