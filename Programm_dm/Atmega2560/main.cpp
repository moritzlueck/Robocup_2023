#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <util/twi.h>
#include "init.h"
#include "usart_functions.h"
#include "drive_functions.h"
#include "interrupt_service_routine.h"
#include "i2c.h"

#define FOSC				16000000
#define BAUD				76800
#define MYUBRR			FOSC/16/BAUD-1

/*----------activations----------*/
#define area_activated 0
#define start_tile_recovery 1
#define start_tile_recovery_after_lop 1 //if enabled, after a LOP, the robot will check if the conditions of a return to the start tile are satisfied. However if the robot is interrupted by a LOP during the return process 2 times, the robot won't return to the starting tile
#define drive_after_lop 1
#define muliti_measurement 0
#define align_side 1	
#define align_side_after_ir_contact 1
#define align_side_using_ir_front 1
#define detect_obstacles 1
#define turning_failed_detection 1
#define clear_black_tile_after_ramp 1

#define display_enable 1

#define deleay_between_movement if(1) _delay_ms(100)

/*---------- I2C addresses----------*/
#define LCD_Address 0x3F;

/*----------encoder related values----------*/
#define encoder_drive 320
#define encoder_drive_half 165
#define encoder_drive_after_ir_back 70
#define encoder_turn_right 195
#define encoder_turn_left 205

#define encoder_align_max_drive 100
#define encoder_align_max_turn 75
#define encoder_drive_too_few_steps 130
#define encoder_gap_right_min_steps 100

#define encoder_drive_before_ejection 120

#define encoder_drive_after_contact 30
#define encoder_drive_after_contact_ir 40
#define encoder_turn_after_contact 40
#define encoder_turn_after_contact_ir 50

#define encoder_drive_after_black_detected 40
#define encoder_drive_after_contact_ir_detected 40
#define encoder_drive_after_angle_detected 80
#define encoder_drive_after_no_angle_detected 80
#define encoder_drive_after_wall_front_detected 20
#define encoder_drive_after_victm_detected 10

#define encoder_drive_after_ramp_up 40
#define encoder_drive_after_ramp_down 100

/*----------velocity related values----------*/
#define velocity_drive 210
#define velocity_drive_ir_slow 180
#define velocity_turn 220
#define velocity_power_turn 240

#define velocity_align_turn 130
#define velocity_align_drive 130

/*----------ir sensor related values----------*/
#define ir_wall_threshold 35	//note that the value is used in combination with a < operator

#define ir_wall_optimal 65
#define ir_wall_too_far 55
#define ir_wall_too_close 80

#define ir_wall_drive 55

/*----------ultrasonic sensor related values----------*/
#define ultrasonic_wall_theshold_front 26	//note that the value is used in combination with a < operator
#define ultrasonic_wall_threshold_side 25	//note that the value is used in combination with a < operator
#define ultrasonic_wall_drive 25	//note that the value is used in combination with a < operator

/*----------color sensor related values----------*/
#define color_victm_default 14
#define color_victm_high_default 20
#define color_victm_low_default 10
#define color_checkpoint_threshold_default 70
#define color_victm_range 6

#define greyscale_black_threshold_default 80
#define greyscale_black_range 25

/*----------servo values----------*/
#define servo_home_pulse 500
#define servo_open_pulse 1500
#define servo_signal_length 20

/*----------mapping values----------*/
#define start_position 16
#define area_amount 4
#define exit_radius 1

#define recover_start_tile_threshold 5

#define mapping_error_threshold 3

#define wf							0b00000001
#define wr							0b00000010
#define wb							0b00000100
#define wl							0b00001000
#define wall_map					0b00001111
#define victm_map					0b00010000
#define black_tile_map				0b00100000
#define checkpoint_map				0b01000000
#define visited_map					0b10000000

#define display_status_calibration	0b00000000
#define display_status_run			0b01000000
#define display_status_lop			0b10000000
#define display_status_end			0b11000000

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

#define	led_green_on				PORTL |= (1 << PORTL5) // pin 44
#define	led_green_off				PORTL &= ~(1 << PORTL5)
#define	led_blue_on					PORTL |= (1 << PORTL6) // pin 43
#define	led_blue_off				PORTL &= ~(1 << PORTL6)
#define	led_red_on					PORTL |= (1 << PORTL7) // pin 42
#define	led_red_off					PORTL &= ~(1 << PORTL7)
#define led_white_on				PORTL |= 0b11100000
#define led_white_off				PORTL &= 0b00011111

#define led_array_direction			PORTC

/*----------servo----------*/
#define servo_high					PORTH |= (1 << PORTH5)
#define servo_low					PORTH &= ~(1 << PORTH5)

/*----------color sensor configuration sensors----------*/
#define color_sensor_s2_high		PORTG |= (1 << PORTG1);
#define color_sensor_s3_high		PORTG |= (1 << PORTG0);
#define color_sensor_s2_low			PORTG &= ~(1 << PORTG1);
#define color_sensor_s3_low			PORTG &= ~(1 << PORTG0);

/*----------ir sensors-----------*/
#define ir_right_front				analogue_value[0]
#define ir_right_back				analogue_value[1]
#define ir_left_front				analogue_value[2]
#define ir_left_back				analogue_value[3]
#define ir_front_right				analogue_value[4]
#define ir_front_left				analogue_value[5]
#define ir_back_right				analogue_value[6]
#define ir_back_left				analogue_value[7]
#define ir_front_middle_right		analogue_value[10]
#define ir_front_middle_left		analogue_value[11]

/*----------greyscale sensors----------*/
#define greyscale_right				analogue_value[8]
#define greyscale_left				analogue_value[9]


/*----------touch sensors----------*/
#define touch_front_right			!(PINA &(1<<PA0))
#define touch_front_middle_right	!(PINA &(1<<PA1))
#define touch_front_middle_left		!(PINA &(1<<PA2))
#define touch_front_left			!(PINA &(1<<PA3))
#define touch_back_right			!(PINA &(1<<PA4))
#define touch_back_left				!(PINA &(1<<PA5))

/*----------angle sensors----------*/
#define angle_up					!(PINA & (1<<PA6))
#define angle_down					!(PINA & (1<<PA7))
#define angle						!(PINA & 0b11000000)

/*----------buttons/switches----------*/
#define lack_of_progress_switch		!(PINC & (1<<PC7))
#define calibration_button			!(PINA &(1<<PA4))

/*----------color sensor----------*/
unsigned char color_victm = color_victm_default;
unsigned char color_victm_high = color_victm_high_default;
unsigned char color_victm_low = color_victm_low_default;
unsigned char color_checkpoint = 0;
unsigned char color_checkpoint_threshold = color_checkpoint_threshold_default;
unsigned char color_white = 0;

unsigned char greyscale_black_right = 0;
unsigned char greyscale_black_left = 0;
unsigned char greyscale_black_threshold_right = greyscale_black_threshold_default;
unsigned char greyscale_black_threshold_left = greyscale_black_threshold_default;

/*----------data visualizer numbers----------*/
unsigned char data[10];

/*----------turning variables----------*/
unsigned char turn_direction = 0;

/*----------driving variables----------*/
bool stop_after_ir_side = 0;
bool stop_after_ir_side_done = 0;
bool eject_while_driving = 0;
unsigned char encoder_steps_before_black = 0;
unsigned char contact_counter_right = 0;
unsigned char contact_counter_left = 0;
bool wall_detected_while_driving = 0;

/*----------counter variables----------*/
unsigned char turn_right_counter = 0;
unsigned char straight_counter = 0;
unsigned char drive_counter = 0;
unsigned char victm_counter = 0;
unsigned char black_tile_counter = 0;
unsigned char checkpoint_counter = 0;

/*----------field element variables----------*/
bool victm_flag= 0;
bool victm_identified = 0;
bool black_tile = 0;
bool checkpoint = 0;
unsigned char ramp = 0;

/*----------error variables----------*/
bool driving_failed = 0;
bool driving_angle_detection_failed = 0;
bool turning_failed = 0;
bool lack_of_progress = 0;
bool recover_start_error = 0;
unsigned char lop_during_return_to_start_counter = 0;
unsigned char map_error_counter = 0;

/*----------mapping variables----------*/
unsigned char map[(start_position * 2)][(start_position * 2)][area_amount];
unsigned char area_exit[area_amount][5];

unsigned char position_x = start_position;
unsigned char position_y = start_position;
unsigned char position_x_est = start_position;
unsigned char position_y_est = start_position;
unsigned char position_x_est_last = start_position;
unsigned char position_y_est_last = start_position;
unsigned char position_x_next = 0;
unsigned char position_y_next = 0;
unsigned char position_x_last = start_position;
unsigned char position_y_last = start_position;

unsigned char direction = 0;
unsigned char direction_est = 0;
unsigned char direction_last = 0;
unsigned char direction_next = 0;

unsigned char area = 0;
unsigned char area_last = 0;
unsigned char area_counter = 0;

unsigned char last_checkpoint_position_x = start_position;
unsigned char last_checkpoint_position_y = start_position;
unsigned char last_checkpoint_direction = 0;
unsigned char last_checkpoint_area = 0;
unsigned char last_checkpoint_area_counter = 0;

unsigned char recover_start_tile_counter = 0;
unsigned char mapping_error_counter = 0;

/*---------path planning----------*/
unsigned char map_path[start_position * 2][start_position * 2][1];
unsigned char path_value_front = 0;
unsigned char path_value_right = 0;
unsigned char path_value_back = 0;
unsigned char path_value_left = 0;

unsigned char path_value_counter = 0;


/*---------wall detection variables----------*/
unsigned char wall = 0;
unsigned char wall_est = 0;
unsigned char wall_rel = 0;
unsigned char wall_abs = 0;
unsigned char wall_quick_abs = 0;
unsigned char wall_quick_rel = 0;
unsigned char wall_before_turn = 0;
unsigned char wall_after_turn = 0;

unsigned char wall_counter_front = 0;
unsigned char wall_counter_right = 0;
unsigned char wall_counter_back = 0;
unsigned char wall_counter_left = 0;
unsigned char checkpoint_counter_detect = 0;

unsigned char encoder_counter_before_ramp = 0;

/*----------encoder storage----------*/
signed int encoder_counter_temp_contact = 0;
signed int encoder_counter_temp_align_ir_side = 0;

/*----------display----------*/
unsigned char display_status = display_status_calibration;

/*---------testing variables----------*/
unsigned char temp_1 = 0;
unsigned char temp_2 = 0;

/*----------functions----------*/
void drive(void);
void drive_ir(void);
void turn(unsigned char mode, unsigned char power_turn);
void align_ir(unsigned char mode = 0b00001011);
void align_ir_side(unsigned char side_input, unsigned char distance_input);
void return_to_start(void);
void return_to_start_new(void);
void return_to_tile(unsigned char position_x_input, unsigned char position_y_input, unsigned char area_input);

void rescue(void);
void blink_rescue(void);
void servo_home(void);
void eject(void);
void exit(void);

void detect_wall(void);
void detect_wall_quick(unsigned char direction_input);
unsigned char calculate_absolute_wall(unsigned char direction_input, unsigned char wall_input);
unsigned char calculate_relative_wall(unsigned char direction_input, unsigned char wall_input, unsigned char wall_direction = 4);

void handle_map(void);
void recover_map(void);
void add_black_tile(void);
void add_victm(void);
void clear_black_tile(void);
bool check_known_black_tile(unsigned char direction_input);
void plan_path(unsigned char position_x_input, unsigned char position_y_input, unsigned char area_input);
void clear_map_path_array(void);
unsigned char get_min_side(unsigned char value_front, unsigned char value_right, unsigned char value_back, unsigned char value_left);
unsigned char get_next_position_x(unsigned char direction_input);
unsigned char get_next_position_y(unsigned char direction_input);

void set_data(void);
void set_display_data(unsigned char mode = 0);
void color_sensor_mode(unsigned char mode);

void reset_run_time(void);

void calibrate(void);

int main(void) {
    
	init();
	stop();
	led_blue_on;
	USART_Init(MYUBRR);
	servo_home();
	color_sensor_mode(0);
	sei();
	
	led_blue_on;
	_delay_ms(250);
	led_blue_off;
	led_red_on;
	_delay_ms(250);
	led_red_off;
	led_green_on;
	_delay_ms(250);
	led_green_off;
		
	while(0) {
		set_display_data();
		led_green_on;
		//turn(1,0);
		//_delay_ms(3000);
	} 
	
	if (calibration_button) calibrate();
	
	while (lack_of_progress_switch) {
		data[0] = map[32][0][0];//display_status;//color_victm;//ir_front_right;
		data[1] = map[255][0][0];//color_victm_high;//ir_front_left;
		data[2] = color_victm_low;//ir_right_front;
		data[3] = color_white;//ir_right_back;
		data[4] = color_checkpoint;//ir_back_right;
		data[5] = color_checkpoint_threshold;//ir_back_left;
		data[6] = greyscale_right;//ir_left_front;
		data[7] = greyscale_left;//ir_left_back;
		data[8] = color;
		data[9] = (angle_up || angle_down);
		
		set_display_data();
		
		if(calibration_button) {
			led_array_on;
			_delay_ms(100);
			led_array_off;
			_delay_ms(100);
			led_array_on;
			_delay_ms(100);
			led_array_off;
			for (unsigned char i = 0; i < 5; i++) {
				_delay_ms(100);
				led_array |= (1 << i);
			}
			for (unsigned char i = 0; i < 5; i++) {
				_delay_ms(860);
				led_array &= ~(1 << i);
			}
			eject();
			while (calibration_button);
			_delay_ms(50); // debounce
		}
		
		if(color > color_victm_low && color < color_victm_high) {
			led_red_on;
		} else led_red_off;
		if(color > color_checkpoint_threshold) {
			led_green_on;
		} else led_green_off;
		if(greyscale_right < greyscale_black_threshold_right && greyscale_left < greyscale_black_threshold_left) {
			led_blue_on;
		} else led_blue_off;
		
		if (ir_front_right > ir_wall_threshold && ir_front_left > ir_wall_threshold && ultrasonic_front < ultrasonic_wall_theshold_front) led_array_1_on;
		else led_array_1_off;
		if (ir_right_front > ir_wall_threshold && ir_right_back > ir_wall_threshold && ultrasonic_right < ultrasonic_wall_threshold_side) led_array_2_on;
		else led_array_2_off;
		if (ir_back_right > ir_wall_threshold && ir_back_left > ir_wall_threshold) led_array_3_on;
		else led_array_3_off;
		if (ir_left_front > ir_wall_threshold && ir_left_back > ir_wall_threshold && ultrasonic_left < ultrasonic_wall_threshold_side) led_array_4_on;
		else led_array_4_off;
	}
	reset_run_time();
	 _delay_ms(100);
	display_status = display_status_run;
	lack_of_progress_flag = 0;
	lack_of_progress = 0;
	led_array_direction |= (1<<0);
	
	while(1) {
		 handle_map();
		 set_display_data();
		 if ((black_tile || (driving_failed && wall_detected_while_driving && turn_direction == 1)) && !lack_of_progress_flag) {
			 if (!(wall_rel & wl) && !check_known_black_tile(3)) turn_direction = 3;
			 else if (!(wall_rel & wb) && !check_known_black_tile(2)) turn_direction = 2;
			 else if (!(wall_rel & wr) && !check_known_black_tile(1)) turn_direction = 1;
		 } else if (!lack_of_progress_flag) {
			 if (!(wall_rel & wr) && !check_known_black_tile(1)) turn_direction = 1;
			 else if (!(wall_rel & wf) && !check_known_black_tile(0)) turn_direction = 0;
			 else if (!(wall_rel & wl) && !check_known_black_tile(3)) turn_direction = 3;
			 else if (!(wall_rel & wb) && !check_known_black_tile(2)) turn_direction = 2;
		 }
		set_data();
		 if (victm_flag && !(map[position_x][position_y][area] & victm_map) && !checkpoint && !lack_of_progress_flag) {
			victm_identified = 1;
			blink_rescue();
			victm_flag = 0;
		 }
		 if (checkpoint && position_x >= (start_position - exit_radius) && position_x <= (start_position + exit_radius) && position_y >= (start_position - exit_radius) && position_y <= (start_position + exit_radius) && !((direction + turn_direction) % 4) && !area && (map[start_position][start_position][0] & wall_map) == wall && victm_counter && !lack_of_progress && !lack_of_progress_flag) exit();
		 if (!lack_of_progress_flag) {
			 switch(turn_direction) {
				 case 0:
					straight_counter++;
					turning_failed = 0;
					break;
				 case 1:
					turn(0, turning_failed);
					straight_counter = 0;
					turn_right_counter++;
					break;
				 case 2:
					turn(1, turning_failed);
					align_ir();
					turn(1, turning_failed);
					straight_counter = 0;
					turn_right_counter = 0;
					break;
				 case 3:
					turn(1, turning_failed);
					straight_counter = 0;
					turn_right_counter = 0;
					break;
				default:
					break;
			 }
		 }
		 align_ir(0xff);
		 detect_wall_quick(0);
		 if (checkpoint && wall_quick_abs == (map[start_position][start_position][0] & wall_map) && victm_counter >= 3 && checkpoint_counter > 2 && start_tile_recovery && !lack_of_progress_flag) {
			 recover_start_tile_counter++;
			 recover_start_error = 0;
			 direction_est = 0;
			 position_x_est = start_position;
			 position_y_est = start_position;
			 led_green_on;
			 _delay_ms(1000);
		 }
		 if (victm_identified && !lack_of_progress_flag) {
			 rescue();
			 victm_identified = 0;
		 } else if (victm_flag && !(map[position_x][position_y][area] & victm_map) && !checkpoint && !lack_of_progress_flag) {
			 blink_rescue();
			 rescue();
			 victm_flag = 0;
		 }
		 set_display_data();
		 deleay_between_movement;
		 if (/*!turning_failed && */!lack_of_progress_flag) drive();
		 deleay_between_movement; 
	}
}

void set_data(void) {
	data[0] = check_known_black_tile(1);
	data[1] = wall;
	data[2] = 0;
	data[3] = map_path[12][13][0];
	data[4] = map_path[13][12][0];
	data[5] = map_path[14][12][0];
	data[6] = map_path[12][11][0];
	data[7] = map_path[13][11][0];
	data[8] = map_path[14][11][0];
	data[9] = path_value_counter;
}

void drive(void) {
	encoder_counter = 0;
	black_tile = 0;
	victm_flag = 0;
	ramp = 0;
	driving = 1;
	stop_after_ir_side = 0;
	contact_counter_right = 0;
	contact_counter_left = 0;
	wall_detected_while_driving = 0;
	driving_angle_detection_failed = 0;
	led_white_off;
	while((encoder_counter < encoder_drive || angle_up || angle_down) && !lack_of_progress_flag) {
		
		drive_ir();
		PINB |= (1<<PINB7);
		
		if (eject_while_driving && encoder_counter > encoder_drive_before_ejection) {
			stop();
			eject();
			eject_while_driving = 0;
		}
		
		if (greyscale_right < greyscale_black_threshold_right || greyscale_left < greyscale_black_threshold_left || (detect_obstacles && !(wall_quick_rel & wr) && !(wall_quick_rel & wl) && ir_front_middle_right > ir_wall_threshold && ir_front_middle_left > ir_wall_threshold && ultrasonic_front < ultrasonic_wall_theshold_front && ir_front_right < ir_wall_threshold && ir_front_left < ir_wall_threshold)) {
			if (encoder_counter_black_tile > encoder_drive_after_black_detected && !lack_of_progress_flag) {
				led_blue_on;
				stop();
				black_tile = 1;
				add_black_tile();
				set_display_data(1);
				if (eject_while_driving) {
					eject();
					eject_while_driving = 0;
				}
				encoder_steps_before_black = encoder_counter;
				encoder_counter = 0;
				while (encoder_counter < encoder_steps_before_black && !lack_of_progress_flag) backward(velocity_align_drive, velocity_align_drive);
				stop();
				led_blue_off;
				break;
			}
		} else encoder_counter_black_tile = 0;
		
		if ((ir_front_right > ir_wall_drive || ir_front_left > ir_wall_drive) && ir_front_middle_right > ir_wall_threshold && ir_front_middle_left > ir_wall_threshold && ultrasonic_front < ultrasonic_wall_drive) {
			led_red_on;
			if (encoder_counter_wall_front > encoder_drive_after_wall_front_detected) {
				wall_detected_while_driving = 1;
				break;
			}
		} else encoder_counter_wall_front = 0;
		
		if ((ir_right_front < ir_wall_threshold && ir_right_back > ir_wall_threshold && encoder_counter > encoder_gap_right_min_steps) || stop_after_ir_side) {
			if (ir_right_front < ir_wall_threshold && ir_right_back < ir_wall_threshold && !stop_after_ir_side_done) {
				encoder_counter = encoder_drive - encoder_drive_after_ir_back;
				stop_after_ir_side = 0;
				stop_after_ir_side_done = 1;
			} else stop_after_ir_side = 1;
		}
		
		if (color < color_victm_high && color > color_victm_low && encoder_counter > encoder_drive_half && !lack_of_progress_flag) {
			if (encoder_counter_victm > encoder_drive_after_victm_detected) victm_flag = 1;
		} else encoder_counter_victm = 0;
		
		if (angle_up || angle_down) {
			led_array_5_on;
			if (encoder_counter_ramp > encoder_drive_after_angle_detected) {
				if (angle_up) {
					ramp = 1;
					encoder_counter_before_ramp = encoder_counter;
					while(!lack_of_progress_flag) {
						drive_ir();
						
						if (!angle_up) {
							if(encoder_counter_ramp > encoder_drive_after_no_angle_detected) {
								stop();
								encoder_counter = encoder_drive - encoder_drive_after_ramp_up;
								led_array_5_off;
								break;
							}
						} else encoder_counter_ramp = 0;
						if ((ir_right_front < ir_wall_threshold && ir_right_back > ir_wall_threshold && encoder_counter > encoder_gap_right_min_steps) || stop_after_ir_side) {
							if (ir_right_front < ir_wall_threshold && ir_right_back < ir_wall_threshold) {
								encoder_counter = encoder_drive - encoder_drive_after_ir_back;
								led_green_on;
								stop_after_ir_side = 0;
								break;
							} else stop_after_ir_side = 1;
						}
					}
				} else {
					ramp = 2;
					encoder_counter_before_ramp = encoder_counter;
					while(!lack_of_progress_flag) {
						drive_ir();
						if (!angle_down) {
							if(encoder_counter_ramp > encoder_drive_after_no_angle_detected) {
								stop();
								encoder_counter = encoder_drive - encoder_drive_after_ramp_down;
								led_array_5_off;
								break;
							}
						} else encoder_counter_ramp = 0;
						if ((ir_right_front < ir_wall_threshold && ir_right_back > ir_wall_threshold && encoder_counter > encoder_gap_right_min_steps) || stop_after_ir_side) {
							if (ir_right_front < ir_wall_threshold && ir_right_back < ir_wall_threshold) {
								encoder_counter = encoder_drive - encoder_drive_after_ir_back;
								led_green_on;
								stop_after_ir_side = 0;
								break;
							} else stop_after_ir_side = 1;
						}	
					}
				}
			}
		} else encoder_counter_ramp = 0;
		
		if (ir_front_right > ir_wall_optimal && ir_front_left < ir_wall_threshold && ultrasonic_front > ultrasonic_wall_theshold_front && contact_counter_right < 2 && !lack_of_progress_flag) {
			if (encoder_counter_ir_contact_right > encoder_drive_after_contact_ir_detected) {
				stop();
				encoder_counter_temp_contact = encoder_counter;
				if (align_side_after_ir_contact) {
					align_ir_side(1, 2);
					align_ir(0b00000001);
				} else {
					encoder_counter = 0;
					while (encoder_counter < encoder_drive_after_contact_ir && !lack_of_progress_flag) backward(velocity_align_drive, velocity_align_drive);
					stop();
					encoder_counter = 0;
					while (encoder_counter < encoder_turn_after_contact_ir) turn_left(velocity_align_turn);
					stop();
				}
				encoder_counter = encoder_counter_temp_contact - encoder_drive_after_contact_ir;
				contact_counter_right++;
			}
		} else encoder_counter_ir_contact_right = 0;
		if (touch_front_right && !touch_front_left && contact_counter_right < 4 && !lack_of_progress_flag) {
			stop();
			encoder_counter_temp_contact = encoder_counter;
			encoder_counter = 0;
			while (encoder_counter < encoder_drive_after_contact && !lack_of_progress_flag) backward(velocity_align_drive, velocity_align_drive);
			stop();
			encoder_counter = 0;
			while (encoder_counter < encoder_turn_after_contact && !lack_of_progress_flag) turn_left(velocity_align_turn);
			stop();
			encoder_counter = encoder_counter_temp_contact;
			contact_counter_right++;
		}
		if (ir_front_left > ir_wall_optimal && ir_front_right < ir_wall_threshold && ultrasonic_front > ultrasonic_wall_theshold_front && contact_counter_right < 2 && !lack_of_progress_flag) {
			if (encoder_counter_ir_contact_left > encoder_drive_after_contact_ir_detected) {
				stop();
				encoder_counter_temp_contact = encoder_counter;
				if (align_side_after_ir_contact) {
					align_ir_side(0, 2);
					align_ir(0b00000001);
				} else {
					encoder_counter = 0;
					while (encoder_counter < encoder_drive_after_contact_ir && !lack_of_progress_flag) backward(velocity_align_drive, velocity_align_drive);
					stop();
					encoder_counter = 0;
					while (encoder_counter < encoder_turn_after_contact_ir) turn_right(velocity_align_turn);
					stop();
				}
				encoder_counter = encoder_counter_temp_contact - encoder_drive_after_contact_ir;
				contact_counter_left++;
			}
		} else encoder_counter_ir_contact_left = 0;
		if (touch_front_left && !touch_front_right && contact_counter_left < 4 && !lack_of_progress_flag) {
			stop();
			encoder_counter_temp_contact = encoder_counter;
			encoder_counter = 0;
			while (encoder_counter < encoder_drive_after_contact && !lack_of_progress_flag) backward(velocity_align_drive, velocity_align_drive);
			stop();
			encoder_counter = 0;
			while (encoder_counter < encoder_turn_after_contact && !lack_of_progress_flag) turn_right(velocity_align_turn);
			stop();
			encoder_counter = encoder_counter_temp_contact;
			contact_counter_left++;
		}
	}
	stop();
	driving = 0;
	if (ramp && !lack_of_progress_flag) {
		if (encoder_counter_before_ramp < encoder_drive_too_few_steps && straight_counter && !lack_of_progress_flag) {
			driving_angle_detection_failed = 1;
			position_x = position_x_last;
			position_y = position_y_last;
			if (recover_start_tile_counter) {
				position_x_est = position_x_est_last;
				position_y_est = position_y_est_last;
			} 
			led_blue_on;
			_delay_ms(100);
			led_blue_off;
		} else {
			led_green_on;
			_delay_ms(100);
			led_green_off;
		}
		if (clear_black_tile_after_ramp) {
			clear_black_tile();
		}
		if (area_activated) {
			if (area && position_x >= (start_position - 1) && position_x <= (start_position + 1) && position_y >= (start_position - 1)  && position_y <= (start_position + 1) && area_exit[area][2] == ((direction + 2) % 4) && area_exit[area][4] != ramp && straight_counter) {
				area_last = area;
				position_x_last = position_x;
				position_y_last = position_y;
				position_x = area_exit[area_last][0];
				position_y = area_exit[area_last][1];
				area = area_exit[area_last][3];
			} else {
				area_counter++;
				area_last = area;
				area = area_counter;
				area_exit[area][0] = position_x;
				area_exit[area][1] = position_y;
				area_exit[area][2] = (direction % 4);
				area_exit[area][3] = area_last;
				area_exit[area][4] = ramp;
				position_x_last = position_x;
				position_y_last = position_y;
				position_x = start_position;
				position_y = start_position;
			}
		} else {
			position_x_last = position_x;
			position_y_last = position_y;
			const signed char dx[] = {0,2,0,-2};
			const signed char dy[] = {2,0,-2,0};
			position_x += dx[direction % 4];
			position_y += dy[direction % 4];
			if (recover_start_tile_counter) {
				position_x_est_last = position_x_est;
				position_y_est_last = position_y_est;
				position_x_est += dx[direction_est % 4];
				position_y_est += dy[direction_est % 4];
			}	
		}
	} else if (encoder_counter > encoder_drive_too_few_steps && !black_tile && !lack_of_progress_flag) {
		driving_failed = 0;
		led_blue_off;
		position_x_last = position_x;
		position_y_last = position_y;
		const signed char dx[] = {0,1,0,-1};
		const signed char dy[] = {1,0,-1,0};
		position_x += dx[direction % 4];
		position_y += dy[direction % 4];
		if (recover_start_tile_counter) {
			position_x_est_last = position_x_est;
			position_y_est_last = position_y_est;
			position_x_est += dx[direction_est % 4];
			position_y_est += dy[direction_est % 4];
		}
	} else if (encoder_counter < encoder_drive_too_few_steps && !black_tile && !lack_of_progress_flag) {
		driving_failed = 1;
		led_blue_on;
		_delay_ms(100);
	}
	drive_counter++;
	led_white_off;
}

void drive_ir(void) {
	if (ir_right_front > ir_wall_threshold && ir_right_back > ir_wall_threshold && ultrasonic_right < ultrasonic_wall_threshold_side) {
		if (ir_right_front > ir_wall_too_close && ir_right_back > ir_wall_too_close) {
			forward(velocity_drive_ir_slow, velocity_drive);
		} else if (ir_right_front < ir_wall_too_far && ir_right_front < ir_wall_too_far) {
			forward(velocity_drive, velocity_drive_ir_slow);
		} else if (ir_right_front > ir_right_back) {
			forward(velocity_drive_ir_slow, velocity_drive);
		} else if (ir_right_front < ir_right_back) {
			forward(velocity_drive, velocity_drive_ir_slow);
		}
	} else if (ir_left_front > ir_wall_threshold && ir_left_back > ir_wall_threshold && ultrasonic_left < ultrasonic_wall_threshold_side) {
		if (ir_left_front > ir_wall_too_close && ir_left_back > ir_wall_too_close) {
			forward(velocity_drive, velocity_drive_ir_slow);
		} else if (ir_right_front < ir_wall_too_far && ir_left_back < ir_wall_too_far) {
			forward(velocity_drive_ir_slow, velocity_drive);
		} else if (ir_left_front > ir_left_back) {
			forward(velocity_drive, velocity_drive_ir_slow);
		} else if (ir_left_front < ir_left_back) {
			forward(velocity_drive_ir_slow, velocity_drive);
		}
	} else forward(velocity_drive, velocity_drive);
}

void turn(unsigned char mode, unsigned char power_turn) {
	set_data();
	encoder_counter = 0;
	if (!mode) {
		direction_last = direction;
		direction += 1;
		if (recover_start_tile_counter) direction_est += 1;
		while (encoder_counter < encoder_turn_right && !lack_of_progress_flag) {
			if (power_turn) turn_right(velocity_power_turn);
			else turn_right(velocity_turn);
			if (color < color_victm_high && color > color_victm_low && !lack_of_progress_flag) victm_flag= 1;
		}
	} else {
		direction_last = direction;
		direction += 3;
		if (recover_start_tile_counter) direction_est += 3;
		while (encoder_counter < encoder_turn_left && !lack_of_progress_flag) {
			if (power_turn) turn_left(velocity_power_turn);
			else turn_left(velocity_turn);
			if (color < color_victm_high && color > color_victm_low && !lack_of_progress_flag) victm_flag= 1;
		}
	}
	stop();
	if (turning_failed_detection) {
		detect_wall_quick(direction);
		if (wall != wall_quick_abs && ramp != 1 && wall != 0b00001010 && wall != 0b00000101) {
			set_data();
			led_blue_on;
			wall_before_turn = wall;
			led_blue_off;
			align_ir();
			detect_wall();
			if (wall != wall_before_turn) {
				for (unsigned char i = 0; i < 4; i++) {
					if (calculate_absolute_wall(i, wall_rel) == wall_before_turn) {
						if (direction != i) turning_failed = 1;
						direction = i;
						detect_wall();
						break;
					}
				}
			} else turning_failed = 0;
		} else turning_failed = 0;
	}
	set_data();
	led_array_direction &= 0b11110000;
	led_array_direction |= (1<<(direction % 4));
}

void return_to_start(void) {
	direction = direction_est;
	position_x = position_x_est;
	position_y = position_y_est;
	clear_black_tile();
	align_ir();
	if (black_tile && !lack_of_progress_flag) {
		if (!(wall_rel & wr) && !check_known_black_tile(1)) turn(0,0);
		else if (!(wall_rel & wb) && !check_known_black_tile(2)) {
			 turn(0,0);
			 align_ir();
			 turn(0,0);
		}
		else if (!(wall_rel & wl) && !check_known_black_tile(3)) turn(1,0);
	} else if (!lack_of_progress_flag) {
		turn(0,0);
		align_ir();
		turn(0,0);
	}
	align_ir();
	deleay_between_movement;
	drive();
	turn_direction = 2;
	while(!lack_of_progress_flag) {
		align_ir();
		detect_wall();
		set_display_data();
		if ((black_tile || (driving_failed && turn_direction == 3)) && !lack_of_progress_flag) {
			if (!(wall_rel & wr) && !check_known_black_tile(1)) turn_direction = 1;
			else if (!(wall_rel & wb) && !check_known_black_tile(2)) turn_direction = 2;
			else if (!(wall_rel & wl) && !check_known_black_tile(3)) turn_direction = 3;
		} else if (!lack_of_progress_flag) {
			if (!(wall_rel & wl) && !check_known_black_tile(3)) turn_direction = 3;
			else if (!(wall_rel & wf) && !check_known_black_tile(0)) turn_direction = 0;
			else if (!(wall_rel & wr) && !check_known_black_tile(1)) turn_direction = 1;
			else if (!(wall_rel & wb) && !check_known_black_tile(2)) turn_direction = 2;
		}
		set_data();
		if (checkpoint && !lack_of_progress_flag) exit();
		if (!lack_of_progress_flag) {
			switch(turn_direction) {
				case 0:
					straight_counter++;
					turning_failed = 0;
					break;
				case 1:
					turn(0, turning_failed);
					align_ir();
					straight_counter = 0;
					turn_right_counter++;
					break;
				case 2:
					turn(0, turning_failed);
					align_ir();
					turn(0, turning_failed);
					align_ir();
					straight_counter = 0;
					turn_right_counter = 0;
					break;
				case 3:
					turn(1, turning_failed);
					align_ir();
					straight_counter = 0;
					turn_right_counter = 0;
					break;
				default:
					break;
			}
		}
		deleay_between_movement;
		if (!check_known_black_tile(0) && !turning_failed && !lack_of_progress_flag) drive();
		deleay_between_movement;
	}
}

void return_to_start_new(void) {
	direction = direction_est;
	position_x = position_x_est;
	position_y = position_y_est;
	area = 0; //TEMP!!!!
	set_data();
	return_to_tile(start_position, start_position, 0);
	if(!lack_of_progress_flag) exit();
}

void return_to_tile(unsigned char position_x_input, unsigned char position_y_input, unsigned char area_input) {
	//const signed char dx[] = {0,1,0,-1};
	//const signed char dy[] = {1,0,-1,0};
	plan_path(position_x_input, position_y_input, 0);
	while ((position_x != position_x_input || position_y != position_y_input) && !lack_of_progress_flag) {
		align_ir();
		detect_wall();
		path_value_front = map_path[get_next_position_x(0)][get_next_position_y(0)][0];
		path_value_right = map_path[get_next_position_x(1)][get_next_position_y(1)][0];
		path_value_back = map_path[get_next_position_x(2)][get_next_position_y(2)][0];
		path_value_left = map_path[get_next_position_x(3)][get_next_position_y(3)][0];
		//path_value_front = map_path[position_x + dx[direction % 4]][position_y + dy[direction % 4]][0];
		//path_value_right = map_path[position_x + dx[(direction + 1) % 4]][position_y + dy[(direction + 1) % 4]][0];
		//path_value_back = map_path[position_x + dx[(direction + 2) % 4]][position_y + dy[(direction + 2) % 4]][0];
		//path_value_left = map_path[position_x + dx[(direction + 3) % 4]][position_y + dy[(direction + 3) % 4]][0];
		turn_direction = get_min_side(path_value_front, path_value_right, path_value_back, path_value_left);	
		set_data();	
		if (!lack_of_progress_flag) {
			switch(turn_direction) {
				case 0:
					turning_failed = 0;
					break;
				case 1:
					turn(0, turning_failed);
					break;
				case 2:
					turn(0, turning_failed);
					align_ir();
					turn(0, turning_failed);
					break;
				case 3:
					turn(1, turning_failed);
					break;
				default:
					break;
			}
		}
		align_ir();
		deleay_between_movement;
		if (!turning_failed && !lack_of_progress_flag) drive();
		deleay_between_movement;
	}
}

void plan_path(unsigned char position_x_input, unsigned char position_y_input, unsigned char area_input) {
	path_value_counter = 0;
	clear_map_path_array();
	set_data();
	_delay_ms(5000);
	map_path[position_x_input][position_y_input][0] = 0;
	while (map_path[position_x][position_y][0] == 255) {
		for (unsigned char x = 0; x < (start_position * 2); x++) {
			for (unsigned char y = 0; y < (start_position * 2); y++) {
				if (map_path[x][y][0] == path_value_counter) {
					if (map_path[x][y + 1][0] == 255 && !(map[x][y][0] & wf) && !(map[x][y + 1][0] & black_tile_map) && map[x][y + 1][0] & visited_map) map_path[x][y + 1][0] = path_value_counter + 1;
					if (map_path[x + 1][y][0] == 255 && !(map[x][y][0] & wl) && !(map[x + 1][y][0] & black_tile_map) && map[x + 1][y][0] & visited_map) map_path[x + 1][y][0] = path_value_counter + 1;
					if (map_path[x][y - 1][0] == 255 && !(map[x][y][0] & wb) && !(map[x][y - 1][0] & black_tile_map) && map[x][y - 1][0] & visited_map) map_path[x][y - 1][0] = path_value_counter + 1;
					if (map_path[x - 1][y][0] == 255 && !(map[x][y][0] & wl) && !(map[x - 1][y][0] & black_tile_map) && map[x - 1][y][0] & visited_map) map_path[x + 1][y][0] = path_value_counter + 1;
				}
			}
		}
		path_value_counter++;
		set_data();
		_delay_ms(10000);
	}
	set_data();
	_delay_ms(10000);
}

void clear_map_path_array(void) {
	for (unsigned char x = 0; x < (start_position * 2); x++) {
		for (unsigned char y = 0; y < (start_position * 2); y++) {
			map_path[x][y][0] = 255;
		}
	}
}

unsigned char get_min_side(unsigned char value_front, unsigned char value_right, unsigned char value_back, unsigned char value_left) {
	unsigned char min_value = value_front;
	unsigned char min_side = 0;
	
	if (value_right < min_value) {
		min_value = value_right;
		min_side = 1;
	}
	if (value_left < min_value) {
		min_value = value_left;
		min_side = 3;
	}
	if (value_back < min_value) {
		min_value = value_back;
		min_side = 2;
	}
	return min_side;
}


void handle_map(void) {
	set_data();
	if (lack_of_progress_flag) {
		stop();
		display_status = display_status_lop;
		set_display_data();
		led_white_off;
		while(lack_of_progress_switch) {
			led_blue_on;
			_delay_ms(250);
			led_blue_off;
			_delay_ms(250);
		}
		_delay_ms(50);
		recover_map();
		clear_black_tile();
		lack_of_progress_flag = 0;
		lack_of_progress = 1;
		victm_flag= 0;
		victm_identified = 0;
		black_tile = 0;
		eject_while_driving = 0;
		recover_start_tile_counter = 0;
		display_status = display_status_run;
		if (drive_after_lop) drive();
		deleay_between_movement;
	} else lack_of_progress = 0;
	align_ir();
	detect_wall();
	set_data();
	if (recover_start_tile_counter && !lack_of_progress_flag) {
		if (recover_start_error && !(driving_failed || driving_angle_detection_failed)) recover_start_tile_counter = 0;
		else if ((map[position_x_est][position_y_est][0] & wall_map) == wall_est && !lack_of_progress_flag) {
			led_white_off;
			led_green_on;
			_delay_ms(100);
			led_green_off;
			_delay_ms(100);
			led_green_on;
			_delay_ms(100);
			led_green_off;
			recover_start_tile_counter++;
			recover_start_error = 0;
		} else {
			led_white_off;
			led_red_on;
			_delay_ms(100);
			led_red_off;
			_delay_ms(100);
			led_red_on;
			_delay_ms(100);
			led_red_off;
			recover_start_error = 1;
		}
		if (recover_start_tile_counter >= recover_start_tile_threshold && !lack_of_progress_flag) return_to_start();
	}
	if (map[position_x][position_y][area] && !lack_of_progress_flag) {
		if ((map[position_x][position_y][area] & wall_map) == wall) mapping_error_counter = 0;
		else {
			mapping_error_counter++;
			if(mapping_error_counter >= mapping_error_threshold) clear_black_tile();
		}
	} else if (!lack_of_progress_flag) map[position_x][position_y][area] |= (wall | visited_map);
	if(checkpoint && !lack_of_progress_flag) {
		if (!(map[position_x][position_y][area] & checkpoint_map)) checkpoint_counter++;
		map[position_x][position_y][area] |= checkpoint_map;
		last_checkpoint_position_x = position_x;
		last_checkpoint_position_y = position_y;
		last_checkpoint_direction = direction;
		last_checkpoint_area = area;
		last_checkpoint_area_counter = area_counter;
		led_white_on;
		_delay_ms(100);
		led_white_off;
	}
}

void recover_map(void) {
	position_x = last_checkpoint_position_x;
	position_y = last_checkpoint_position_y;
	direction = last_checkpoint_direction;
	area = last_checkpoint_area;
	area_counter = last_checkpoint_area_counter;
	detect_wall();
	if (!((map[position_x][position_y][area] & wall_map) == wall || (map[position_x][position_y][area] & wall_map) == 0b00001010 || (map[position_x][position_y][area] & wall_map) == 0b00000101)) {
		for (unsigned char i = 0; i < 4; i++) {
			if (calculate_absolute_wall(i, wall_rel) == (map[position_x][position_y][area] & wall_map)) {
				direction = i;
				break;
			}
		}
	}
	else {
		
	}
}

void align_ir (unsigned char mode) {
	bool alignment_drive_done = 0;
	bool alignment_side_done = 0;
	bool alignment_side_front_done = 0;
	detect_wall_quick(direction);
	if (mode & 0b00000001 && !lack_of_progress_flag) {
		encoder_counter = 0;
		if (wall_quick_rel & wr && !lack_of_progress_flag) {
			if (ir_right_front > ir_right_back && !lack_of_progress_flag) {
				while (ir_right_front > ir_right_back && ir_right_front > ir_wall_threshold && ir_right_back > ir_wall_threshold && encoder_counter < encoder_align_max_turn && !lack_of_progress_flag) turn_left(velocity_align_turn); 
			} else if (ir_right_back > ir_right_front && !lack_of_progress_flag) {
				while (ir_right_back > ir_right_front && ir_right_front > ir_wall_threshold && ir_right_back > ir_wall_threshold && encoder_counter < encoder_align_max_turn && !lack_of_progress_flag) turn_right(velocity_align_turn); 
			}
		} else if (wall_quick_rel & wf && !lack_of_progress_flag) {
			if (ir_front_right > ir_front_left && !lack_of_progress_flag) {
				while (ir_front_right > ir_front_left && ir_front_right > ir_wall_threshold && ir_front_left > ir_wall_threshold && encoder_counter < encoder_align_max_turn && !lack_of_progress_flag) turn_right(velocity_align_turn);
			} else if (ir_front_left > ir_front_right && !lack_of_progress_flag) {
				while (ir_front_left > ir_front_right && ir_front_right > ir_wall_threshold && ir_front_left > ir_wall_threshold && encoder_counter < encoder_align_max_turn && !lack_of_progress_flag) turn_left(velocity_align_turn);
			}
		} else if (wall_quick_rel & wb && !lack_of_progress_flag) {
			if (ir_back_right > ir_back_left && !lack_of_progress_flag) {
				while (ir_back_right > ir_back_left && ir_back_right > ir_wall_threshold && ir_back_left > ir_wall_threshold && encoder_counter < encoder_align_max_turn && !lack_of_progress_flag) turn_left(velocity_align_turn);
			} else if (ir_back_left > ir_back_right && !lack_of_progress_flag) {
				while (ir_back_left > ir_back_right && ir_back_right > ir_wall_threshold && ir_back_left > ir_wall_threshold && encoder_counter < encoder_align_max_turn && !lack_of_progress_flag) turn_right(velocity_align_turn);
			}
		} else if (wall_quick_rel & wl && !lack_of_progress_flag) {
			if (ir_left_front > ir_left_back && !lack_of_progress_flag) {
				while (ir_left_front > ir_left_back && ir_left_front > ir_wall_threshold && ir_left_back > ir_wall_threshold && encoder_counter < encoder_align_max_turn && !lack_of_progress_flag) turn_right(velocity_align_turn);
			} else if (ir_left_back > ir_left_front && !lack_of_progress_flag) {
				while (ir_left_back > ir_left_front && ir_left_front > ir_wall_threshold && ir_left_back > ir_wall_threshold && encoder_counter < encoder_align_max_turn && !lack_of_progress_flag) turn_left(velocity_align_turn);
			}
		}
	}
	stop();
	if (mode & 0b00000010 && align_side && !angle_up && !angle_down && !lack_of_progress_flag) {
		unsigned char c = 0;
		if (wall_quick_rel & wr) {
			if (ir_right_front > ir_wall_too_close && ir_right_back > ir_wall_too_close) {
				while (ir_right_front > ir_wall_too_close && ir_right_back > ir_wall_too_close && ir_right_front > ir_wall_threshold && ir_right_back > ir_wall_threshold && c < 3 && !lack_of_progress_flag) {
					align_ir_side(1, 1);
					c++;
				}
				alignment_side_done = 1;
			} else if (ir_right_front < ir_wall_too_far && ir_right_back < ir_wall_too_far) {
				while (ir_right_front < ir_wall_too_far && ir_right_back < ir_wall_too_far && ir_right_front > ir_wall_threshold && ir_right_back > ir_wall_threshold && c < 3 && !lack_of_progress_flag) {
					align_ir_side(0, 1);
					c++;
				}
				alignment_side_done = 1;
			}
		} else if (wall_quick_rel & wl) {
			if (ir_left_front > ir_wall_too_close && ir_left_back > ir_wall_too_close) {
				while (ir_left_front > ir_wall_too_close && ir_left_back > ir_wall_too_close && ir_left_front > ir_wall_threshold && ir_left_back > ir_wall_threshold && c < 3 && !lack_of_progress_flag) {
					align_ir_side(0, 1);
					c++;
				}
				alignment_side_done = 1;
			} else if (ir_right_front < ir_wall_too_far && ir_right_back < ir_wall_too_far) {
				while (ir_left_front < ir_wall_too_far && ir_left_back < ir_wall_too_far && ir_left_front > ir_wall_threshold && ir_left_back > ir_wall_threshold && c < 3 && !lack_of_progress_flag) {
					align_ir_side(1, 1);
					c++;
				}
				alignment_side_done = 1;
			}
		}
		if (mode & 0b00000001 && alignment_side_done && !lack_of_progress_flag) align_ir(0b00000001);
	}
	if (mode & 0b00000100 && align_side_using_ir_front && !angle_up && !angle_down && !(wall_quick_rel & (wf | wr | wl)) && !lack_of_progress_flag) {
		if (ir_front_right > ir_wall_threshold && ir_front_left < ir_wall_threshold && ir_front_middle_right < ir_wall_threshold && ir_front_middle_left < ir_wall_threshold && ultrasonic_front > ultrasonic_wall_theshold_front && !lack_of_progress_flag) {
			align_ir_side(1, 2);
		} else if (ir_front_left > ir_wall_threshold && ir_front_right < ir_wall_threshold && ir_front_middle_right < ir_wall_threshold && ir_front_middle_left < ir_wall_threshold && ultrasonic_front > ultrasonic_wall_theshold_front && !lack_of_progress_flag) {
			align_ir_side(0,2);
		} 
		if (mode & 0b00000001 && alignment_side_front_done && !lack_of_progress_flag) align_ir(0b00000001);
	}
	if (mode & 0b00001000 && !lack_of_progress_flag) {
		encoder_counter = 0;
		if (wall_quick_rel & wf && !lack_of_progress_flag) {
			if (ir_front_right > ir_wall_optimal && ir_front_left > ir_wall_optimal && ramp != 1) {
				while (ir_front_right > ir_wall_optimal && ir_front_left > ir_wall_optimal && ir_front_right > ir_wall_threshold && ir_front_left > ir_wall_threshold && encoder_counter < encoder_align_max_drive) backward(velocity_align_drive, velocity_align_drive);
				alignment_drive_done = 1;
			} else if (ir_front_right < ir_wall_optimal && ir_front_left < ir_wall_optimal) {
				while (ir_front_right < ir_wall_optimal && ir_front_left < ir_wall_optimal && ir_front_right > ir_wall_threshold && ir_front_left > ir_wall_threshold && encoder_counter < encoder_align_max_drive) forward(velocity_align_drive, velocity_align_drive);
				alignment_drive_done = 1;
			}
		} else if (wall_quick_rel & wb && !lack_of_progress_flag) {
			if (ir_back_right > ir_wall_optimal && ir_back_left > ir_wall_optimal) {
				while (ir_back_right > ir_wall_optimal && ir_back_left > ir_wall_optimal && ir_back_right > ir_wall_threshold && ir_back_left > ir_wall_threshold && encoder_counter < encoder_align_max_drive) forward(velocity_align_drive, velocity_align_drive);
				alignment_drive_done = 1;
			} else if (ir_back_right < ir_wall_optimal && ir_back_left < ir_wall_optimal) {
				while (ir_back_right < ir_wall_optimal && ir_back_left < ir_wall_optimal && ir_back_right > ir_wall_threshold && ir_back_left > ir_wall_threshold && encoder_counter < encoder_align_max_drive) backward(velocity_align_drive, velocity_align_drive);
				alignment_drive_done = 1;
			}
		}
		if (mode & 0b00000001 && alignment_drive_done && !lack_of_progress_flag) align_ir(0b00000001);
	}
	stop();
}

void align_ir_side (unsigned char side, unsigned char distance) {
	encoder_counter_temp_align_ir_side = encoder_counter;
	if (!side) {
		for (unsigned char i = 0; i <= distance; i++) {
			encoder_counter = 0;
			while (encoder_counter < 10) turn_left(velocity_turn);
			encoder_counter = 0;
			while (encoder_counter < 15) backward(velocity_drive, velocity_drive);
			encoder_counter = 0;
			while (encoder_counter < 10) turn_right(velocity_turn);
			encoder_counter = 0;
			while (encoder_counter < 15) forward(velocity_drive, velocity_drive);
			stop();
		}
	} else if (side == 1) {
		for (unsigned char i = 0; i <= distance; i++) {
			encoder_counter = 0;
			while (encoder_counter < 10) turn_right(velocity_turn);
			encoder_counter = 0;
			while (encoder_counter < 15) backward(velocity_drive, velocity_drive);
			encoder_counter = 0;
			while (encoder_counter < 10) turn_left(velocity_turn);
			encoder_counter = 0;
			while (encoder_counter < 15) forward(velocity_drive, velocity_drive);
			stop();
		}
	}
	encoder_counter = encoder_counter_temp_align_ir_side;
}

void detect_wall(void) {
	const signed char dx[] = {0,-1,0,1};
	const signed char dy[] = {-1,0,1,0};
	if (muliti_measurement) {
	 	wall_counter_front = 0;
	 	wall_counter_right = 0;
	 	wall_counter_back = 0;
	 	wall_counter_left = 0;
	 	checkpoint_counter_detect = 0;
	 	if (color > color_checkpoint_threshold) checkpoint_counter_detect++;
	 	for(unsigned char i = 0; i < 5; i++) {
		 	if (ir_front_right > ir_wall_threshold && ir_front_left > ir_wall_threshold && ultrasonic_front < ultrasonic_wall_theshold_front) wall_counter_front++;
		 	if (ir_right_front > ir_wall_threshold && ir_right_back > ir_wall_threshold && ultrasonic_right < ultrasonic_wall_threshold_side) wall_counter_right++;
		 	if (ir_back_right > ir_wall_threshold && ir_back_left > ir_wall_threshold) wall_counter_back++;
		 	if (ir_left_front > ir_wall_threshold && ir_left_back > ir_wall_threshold && ultrasonic_left < ultrasonic_wall_threshold_side) wall_counter_left++;
		 	_delay_ms(20);
	 	}
	 	if (color > color_checkpoint_threshold) checkpoint_counter_detect++;
	 	if (checkpoint_counter_detect == 2) {
		 	checkpoint = 1;
		 	led_white_on;
	 	} else checkpoint = 0;
	 	wall_rel = 0;
	 	if (wall_counter_front > 4) wall_rel |= wf;
	 	if (wall_counter_right > 4) wall_rel |= wr;
	 	if (wall_counter_back > 4) wall_rel |= wb;
	 	if (wall_counter_left > 4) wall_rel |= wl;
	} else {
		if (color > color_checkpoint_threshold) checkpoint = 1;
		else checkpoint = 0;
		wall_rel = 0;
		if (ir_front_right > ir_wall_threshold && ir_front_left > ir_wall_threshold && ultrasonic_front < ultrasonic_wall_theshold_front) wall_rel |= wf;
		if (ir_right_front > ir_wall_threshold && ir_right_back > ir_wall_threshold && ultrasonic_right < ultrasonic_wall_threshold_side) wall_rel |= wr;
		if (ir_back_right > ir_wall_threshold && ir_back_left > ir_wall_threshold && !(map[position_x + dx[direction]][position_y + dy[direction]][area] & checkpoint_map)) wall_rel |= wb;
		if (ir_left_front > ir_wall_threshold && ir_left_back > ir_wall_threshold && ultrasonic_left < ultrasonic_wall_threshold_side) wall_rel |= wl;
	}
	wall = calculate_absolute_wall(direction, wall_rel);
	wall_est = calculate_absolute_wall(direction_est, wall_rel);
	led_array_off;
	led_array |= wall_rel;
	set_data();
}

void detect_wall_quick(unsigned char direction_input) {
	wall_quick_rel = 0;
	if (ir_front_right > ir_wall_threshold && ir_front_left > ir_wall_threshold && ultrasonic_front < ultrasonic_wall_theshold_front) wall_quick_rel |= wf;
	if (ir_right_front > ir_wall_threshold && ir_right_back > ir_wall_threshold && ultrasonic_right < ultrasonic_wall_threshold_side) wall_quick_rel |= wr;
	if (ir_back_right > ir_wall_threshold && ir_back_left > ir_wall_threshold) wall_quick_rel |= wb;
	if (ir_left_front > ir_wall_threshold && ir_left_back > ir_wall_threshold && ultrasonic_left < ultrasonic_wall_threshold_side) wall_quick_rel |= wl;
	wall_quick_abs = calculate_absolute_wall(direction_input, wall_quick_rel);
	led_array_off;
	led_array |= wall_quick_rel;
}

unsigned char calculate_absolute_wall(unsigned char direction_input, unsigned char wall_input) {
	return ((((wall_input & wall_map) | ((wall_input & wall_map) << 4)) << (direction_input % 4)) >> 4) & wall_map;
}

void servo_home(void) {
	//cli();
	for (unsigned char i = 0; i < servo_signal_length; i++) {
		servo_high;
		_delay_us(servo_home_pulse);
		servo_low;
		_delay_us(20000);
	}
	//unsigned long temp_time = TCNT5 + ((servo_signal_length * (20000 + servo_home_pulse)) / 64);
	//run_time += temp_time / 15625;
	//TCNT5 = temp_time % 15625;
	//sei();
}

void eject(void) {
	//cli();
	servo_home();
	for (unsigned char i = 0; i < servo_signal_length; i++) {
		servo_high;
		_delay_us(servo_open_pulse);
		servo_low;
		_delay_ms(20);
	}
	servo_home();
	//sei();
}

void color_sensor_mode(unsigned char mode) {
	switch(mode) {
		case 0:
			color_sensor_s2_high;
			color_sensor_s3_high;
			break;
		case 1:
			color_sensor_s2_low;
			color_sensor_s3_high;
			break;
		case 2:
			color_sensor_s2_low;
			color_sensor_s3_high;
			break;
		case 3:
			color_sensor_s2_high;
			color_sensor_s3_low;
			break;
	}
}

void rescue(void) {
	if (!lack_of_progress_flag) {
		if (wall_quick_rel & wb) eject();
		else eject_while_driving = 1;
		add_victm();
	}
}

void blink_rescue(void) {
	led_array_on;
	_delay_ms(100);
	led_array_off;
	_delay_ms(100);
	led_array_on;
	_delay_ms(100);
	led_array_off;
	for (unsigned char i = 0; i < 5; i++) {
		if (lack_of_progress_flag) break;
		_delay_ms(100);
		led_array |= (1 << i);
	}
	for (unsigned char i = 0; i < 5; i++) {
		if (lack_of_progress_flag) break;
		_delay_ms(860);
		led_array &= ~(1 << i);
	}
}

void exit(void) {
	display_status = display_status_end;
	set_display_data();
	led_array_off;
	led_white_on;
	led_array_direction &= 0b11110000;
	while(!lack_of_progress_flag) {
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
}

void clear_black_tile(void) {
	for (unsigned char x = 0; x < (start_position * 2); x++) {
		for (unsigned char y = 0; y < (start_position * 2); y++) {
			for (unsigned char z = 0; z < area_amount; z++) map[x][y][z] &= ~(black_tile_map);
		}
	}
}

void add_black_tile(void) {
	if (!lack_of_progress_flag) {
		map[get_next_position_x(0)][get_next_position_y(0)][area] |= black_tile_map;
		/*switch ((direction % 4)) {
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
		}*/
	}
}

void add_victm(void) {
	if (!lack_of_progress_flag) {
		map[position_x][position_y][area] |= victm_map;
	}
	victm_counter++;
	set_display_data();
}

bool check_known_victm(void) {
	if (map[position_x][position_y][area] &= victm_map) {
		led_red_on;
		return 1;
	} else {
		led_red_off;
		return 0;
	}
}

bool check_known_black_tile(unsigned char direction_input) {
	position_x_next = get_next_position_x(direction_input);
	position_y_next = get_next_position_y(direction_input);
	if((map[position_x_next][position_y_next][area] & black_tile_map) && (position_x_next != start_position || position_y_next != start_position) && position_x_next < (start_position * 2) && position_y_next < (start_position * 2)) {
		led_blue_on;
		return 1;
	} else return 0;
}

unsigned char get_next_position_x (unsigned char direction_input) {
	const signed char dx[] = {0, 1, 0, -1};
	direction_next = (direction + direction_input) % 4;
	return position_x + dx[direction_next];
}

unsigned char get_next_position_y (unsigned char direction_input) {
	const signed char dy[] = {1, 0, -1, 0};
	direction_next = (direction + direction_input) % 4;
	return position_y + dy[direction_next];
}

void reset_run_time(void) {
	TCNT5 = 0;
	run_time = 0;	
}

void set_display_data(unsigned char mode) { //mode = 0: Normal operation	|	mode = 1: Black tile detected
	if (display_status) {
		if (mode) {
			display_data[0] = map[get_next_position_x(0)][get_next_position_y(0)][area];
			display_data[1] = get_next_position_x(0) | ((direction % 4) << 6);
			display_data[2] = get_next_position_y(0) | display_status;
		} else {
			display_data[0] = map[position_x][position_y][area];
			display_data[1] = position_x | ((direction % 4) << 6);
			display_data[2] = position_y | display_status;
		}
	} else {
		display_data[0] = color;
		display_data[1] = (greyscale_right + greyscale_left) / 2;
		display_data[2] = display_status;
	}
}

void calibrate(void) {
	led_array_5_on;
	while (calibration_button) set_display_data();
	_delay_ms(50); // debounce
	led_red_on;
	while (!(calibration_button)) set_display_data();
	_delay_ms(50); //debounce
	color_victm = color;
	led_red_off;
	while (calibration_button) set_display_data();
	_delay_ms(50); // debounce
	led_blue_on;
	while (!(calibration_button)) set_display_data();
	_delay_ms(50); //debounce
	greyscale_black_right = greyscale_right;
	greyscale_black_left = greyscale_left;
	led_blue_off;
	while (calibration_button) set_display_data();
	_delay_ms(50); // debounce
	led_white_on;
	while (!(calibration_button)) set_display_data();
	_delay_ms(50); //debounce
	color_white = color;
	led_white_off;
	while (calibration_button) set_display_data();
	_delay_ms(50); // debounce
	led_green_on;
	while (!(calibration_button)) set_display_data();
	_delay_ms(50); //debounce
	color_checkpoint = color;
	led_green_off;
	while (calibration_button) set_display_data();
	_delay_ms(50); // debounce
	color_victm_high = color_victm + color_victm_range;
	color_victm_low = color_victm - color_victm_range;	
	color_checkpoint_threshold = color_white + ((color_checkpoint - color_white) / 2); //middle between white and checkpoint
	greyscale_black_threshold_right = greyscale_black_right + greyscale_black_range;
	greyscale_black_threshold_left = greyscale_black_left + greyscale_black_range;
	led_array_off;
}