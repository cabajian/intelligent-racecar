/*
 * Freescale Cup Main Code
 *
 * Author:  Chris Abajian, Becky Reich
 * Created:  11/1/2020
 */

#include "MK64F12.h"
#include "uart.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "math.h"
#include "led.h"
#include "buttons.h"
#include "camera.h"
#include "pwm.h"
#include "main.h"

int drive_en = 0;

int motor_speed_min;
int motor_speed_max;

double Kp;
double Kp_motor;
double Ki_motor;

int abs_error_thresh;
int last_error = 0;
int last_pos = POS_CENTER;
int speed_sum = 0;
int speed_sum_modifier = 0;
int cam_num = 0;

int mode = MODE_SLOW;
int start_det_en = 0;

char str_main[100];
int dbg_cnt = 0;

/* Main */
int main(void) {
    // Initiliaze
    led_init();
    init_pwm();
    #if DEBUG == 0
    init_buttons();
    #elif DEBUG == 1
    uart0_init();
    uart3_init();
    #elif DEBUG == 2
    uart3_init();
    #endif
    // 
    move_servo(SERVO_DUTY_CENTER); // center wheels
    led_on(LED_GREEN); // designating SLOW mode
        
    // MANUAL
    #if DEBUG == 0
    car_start_switch(); // manual select
    init_camera();          // init camera (interrupts)

    // DEBUG: Matlab Plotting
    #elif DEBUG == 1
    init_camera();          // init camera (interrupts)
    for (;;) {
			for (int i = 0; i < 2000000; i++) {}
			send_line();
    }
		
    // DEBUG: BT UART Control
    #elif DEBUG == 2
    car_start_uart();   // Bluetooth select
    init_camera();          // init camera (interrupts)
    #endif
		
		// set constants depending on mode
		if (mode == MODE_FAST) {
			Kp = 0.1;
			Kp_motor = 0.6;
			Ki_motor = 0.2;
			abs_error_thresh = 10;
			motor_speed_min = 40;
			motor_speed_max = 70;
			speed_sum_modifier = 2;
			// FEATURE: start detection
			start_det_en = 1;
		} else if (mode == MODE_FAST2) {
			Kp = 0.1;
			Kp_motor = 0.6;
			Ki_motor = 0.2;
			abs_error_thresh = 10;
			motor_speed_min = 40;
			motor_speed_max = 70;
			speed_sum_modifier = 2;
			// FEATURE: start detection
			start_det_en = 1;
		} else if (mode == MODE_MEDIUM) {
			Kp = 0.1;
			Kp_motor = 0.6;
			Ki_motor = 0.15;
			abs_error_thresh = 10;
			motor_speed_min = 45;
			motor_speed_max = 60;
			speed_sum_modifier = 2;
			// FEATURE: start detection
			start_det_en = 1;
		} else if (mode == MODE_MEDIUM2) {
			Kp = 0.1;
			Kp_motor = 0.6;
			Ki_motor = 0.15;
			abs_error_thresh = 10;
			motor_speed_min = 45;
			motor_speed_max = 60;
			speed_sum_modifier = 2;
			// FEATURE: start detection
			start_det_en = 1;
		} else { // default to slow
			Kp = 0.1;
			Kp_motor = 0.6;
			Ki_motor = 0.0;
			abs_error_thresh = 10;
			motor_speed_min = 45;
			motor_speed_max = 50;
			speed_sum_modifier = 0;
			// FEATURE: start detection
			start_det_en = 0;
		}

    for(;;) {
			
			// DEBUG: BT UART command handling
			#if DEBUG == 2
			if (UART3_S1 & UART_S1_RDRF_MASK) {
				// Disable camera IRQ's when printing
				NVIC_DisableIRQ(FTM2_IRQn);
        char cmd = uart3_getchar();
				// Switch through commands
				switch (cmd) {
					case 'q':
						drive_en = 0;
					  sprintf(str_main, "\r\nDriving disabled!");
						uart3_put(str_main);
					  break;
				  case 'd':
						drive_en = 1;
					  sprintf(str_main, "\r\nDriving enabled!");
						uart3_put(str_main);
					  break;
				  case '1':
						Ki_motor += 0.01;
					  sprintf(str_main, "\r\nKi_motor = %g", Ki_motor);
            str_main[8] = '.'; // override decimal character because sprintf is using RS for some reason
						uart3_put(str_main);
					  break;
				  case '2':
						Ki_motor -= 0.01;
					  sprintf(str_main, "\r\nKi_motor = %g", Ki_motor);
            str_main[8] = '.'; // override decimal character because sprintf is using RS for some reason
						uart3_put(str_main);
					  break;
				  case '3':
						Kp_motor += 0.1;
					  sprintf(str_main, "\r\nKp_motor = %g", Kp_motor);
            str_main[8] = '.'; // override decimal character because sprintf is using RS for some reason
						uart3_put(str_main);
					  break;
				  case '4':
						Kp_motor -= 0.1;
					  sprintf(str_main, "\r\nKp_motor = %g", Kp_motor);
            str_main[8] = '.'; // override decimal character because sprintf is using RS for some reason
						uart3_put(str_main);
					  break;
				  case '5':
						abs_error_thresh += 1;
					  sprintf(str_main, "\r\nAbs. error threshold = %d", abs_error_thresh);
						uart3_put(str_main);
					  break;
				  case '6':
						abs_error_thresh -= 1;
					  sprintf(str_main, "\r\nAbs. error threshold = %d", abs_error_thresh);
						uart3_put(str_main);
					  break;
				  case '7':
						motor_speed_min += 1;
					  sprintf(str_main, "\r\nMotor Min Speed = %d", motor_speed_min);
						uart3_put(str_main);
					  break;
					case '8':
						motor_speed_min -= 1;
					  sprintf(str_main, "\r\nMotor Min Speed = %d", motor_speed_min);
						uart3_put(str_main);
					  break;
					case '9':
						motor_speed_max += 1;
					  sprintf(str_main, "\r\nMotor Max Speed = %d", motor_speed_max);
						uart3_put(str_main);
					  break;
					case '0':
						motor_speed_max -= 1;
					  sprintf(str_main, "\r\nMotor Max Speed = %d", motor_speed_max);
						uart3_put(str_main);
					  break;
					case 'b':
						scoreboard();
					  break;
					default:
						sprintf(str_main, "\r\nUnknown command");
						uart3_put(str_main);
				}
				// Re-enable interrupts
				NVIC_EnableIRQ(FTM2_IRQn);
			}
			#endif
			
    } //for
} //main

/* DEBUG: prints current moifier values. */
void scoreboard() {
		uart3_put("\r\n----- SCOREBOARD -----");
		sprintf(str_main, "\r\nMode: %d", mode);
		uart3_put(str_main);
		sprintf(str_main, "\r\nDriving enabled: %d", drive_en);
		uart3_put(str_main);
		sprintf(str_main, "\r\nKp: %g", Kp);
		str_main[7] = '.'; // override decimal character because sprintf is using RS for some reason
		uart3_put(str_main);
		sprintf(str_main, "\r\nKp_motor: %g", Kp_motor);
		str_main[7] = '.'; // override decimal character because sprintf is using RS for some reason
		uart3_put(str_main);
    sprintf(str_main, "\r\nErr. Thresh: %d", abs_error_thresh);
		uart3_put(str_main);
		sprintf(str_main, "\r\nMotor Min Speed: %d", motor_speed_min);
		uart3_put(str_main);
		sprintf(str_main, "\r\nMotor Max Speed: %d", motor_speed_max);
		uart3_put(str_main);
    uart3_put("\r\n----------------------");
}

/* Control logic for processing line data and updating servo/motor duty cycles. */
void control(int *line, int line_max_left, int line_max_right) {
    // line processing
    int line_binary[128];        // binary (thresholded) line
		int edge_rising[5] = {0};    // keep a collection of up to 5 rising edges
		int rising_idx = 0;
    int edge_falling[5] = {0};   // keep a collection of up to 5 falling edges
		int falling_idx = 0;
		int min_idx = CAM_LOW;       // rising edge closest to center
		int max_idx = CAM_HIGH;      // falling edge closest to center
		int edge_cnt = 0;            // total number of edges
    int error = 0;               // error between current and ideal center
		int position = POS_CENTER;
		// duty cycles
    double servo_duty = SERVO_DUTY_CENTER;
		double motor_duty_left = motor_speed_min;
		double motor_duty_right = motor_speed_min;
		int motor_dir_left = FORWARD;
		int motor_dir_right = FORWARD;
		
		// wait a few camera cycles upon start (waits for "good" data
		if (cam_num < 500) {
			cam_num++;
			return;
		}
		
		int dthresh_left = line_max_left - line_max_left/5;
		int dthresh_right = line_max_right - line_max_right/5;

    // calculate binary line using dynamic thresholds
		line_binary[0] = 0;
		line_binary[1] = 0;
		line_binary[2] = 0;
		// left side
		for (int i = 3; i <= 64; i++) {
			if (line[i] > dthresh_left) {
				line_binary[i] = 1;
			} else {
				line_binary[i] = 0;
			}
			// remove previous error in "101" and "1001" sequences
			if ((line_binary[i-2] == 1 && line_binary[i-1] == 0 && line_binary[i] == 1)) {
			  line_binary[i-1] = 1;
			}
		}
		// right side
    for (int i = 65; i <= 127; i++) {
			if (line[i] > dthresh_right) {
				line_binary[i] = 1;
			} else {
				line_binary[i] = 0;
			}
			// remove previous error in "101" and "1001" sequences
			if ((line_binary[i-2] == 1 && line_binary[i-1] == 0 && line_binary[i] == 1)) {
			  line_binary[i-1] = 1;
			}
		}

		// record rising/falling edges
		for (int i = 0; i < 128; i++) {
			int slope = line_binary[i+1] - line_binary[i];
			if (slope > 0) {
		    // rising edge
				edge_rising[rising_idx++] = i;
				if (rising_idx > 4) rising_idx = 4;   // safeguard
			} else if (slope < 0) {
				// falling edge
				edge_falling[falling_idx++] = i;
				if (falling_idx > 4) falling_idx = 4; // safeguard
			}
		}
		
		// edge detection
		edge_cnt = falling_idx + rising_idx;

		// start detection
		if (edge_cnt > 4 && cam_num > 10) {
			if (start_det_en) {
				min_idx = -1;
				max_idx = -1;
		  } else {
				min_idx = 0;
				max_idx = 0;
			}
		// looking at a corner, take the best (most visible) of two tracks
		} else if (edge_cnt == 4) {
			if (edge_falling[0]-edge_rising[0] > edge_falling[1]-edge_rising[1]) {
				// left edge width larger, use these values
				min_idx = edge_rising[0];
				max_idx = edge_falling[0];
			} else {
				// right edge width larger, use these values
				min_idx = edge_rising[1];
				max_idx = edge_falling[1];
			}
		} else {
			// normal condition, only seeing one track
			min_idx = edge_rising[0];
			max_idx = edge_falling[0];
		}
		
		// calculate error
		error = CAM_CENTER - (max_idx+min_idx)/2;
		
		// estimate position on track
		if (min_idx == -1 && max_idx == -1 && (last_pos == POS_CENTER)) { // prevent false positives
			// START DETECTION
			position = POS_START;
	  } else if (min_idx == 0 && max_idx == 0) {
			// looking at black, use last position
			position = last_pos;
		} else if (min_idx <= CAM_LOW+10 && max_idx >= CAM_HIGH-10) {
			position = POS_INTERSECTION;
		} else if (abs(error) <= abs_error_thresh) {
			position = POS_CENTER;
		} else {
			position = POS_CORNER;
		}

		if (drive_en == 1) {
			// DRIVING ENABLED
			
			switch (position) {
				case POS_START: // should only happen once
					drive_en = 0;
          // reverse!
					NVIC_DisableIRQ(FTM2_IRQn);
					move_motor_left(80, BACKWARD);
					move_motor_right(80, BACKWARD);
          if (mode == MODE_FAST2 || mode == MODE_MEDIUM2) {
						for (int i = 0; i < 1000000; i++) {}
					} else {
						for (int i = 0; i < 3000000; i++) {}
					}
					move_motor_left(0, BACKWARD);
					move_motor_right(0, BACKWARD);
					NVIC_EnableIRQ(FTM2_IRQn);
          break;
        case POS_INTERSECTION:
					// intersection
					servo_duty = SERVO_DUTY_CENTER;
					motor_duty_left = motor_speed_max;
					motor_duty_right = motor_speed_max;
					speed_sum += speed_sum_modifier;
					break;
			  case POS_CENTER:
					// center
					servo_duty = SERVO_DUTY_CENTER - Kp*error;
					motor_duty_left = motor_speed_max;
					motor_duty_right = motor_speed_max;
					speed_sum += speed_sum_modifier;
					break;
			  case POS_CORNER:
					// corner
					servo_duty = SERVO_DUTY_CENTER - Kp*error;
					motor_duty_left = motor_speed_min - Kp_motor*error - Ki_motor*speed_sum;  // differential drive
					motor_duty_right = motor_speed_min + Kp_motor*error - Ki_motor*speed_sum; // differential drive
					speed_sum -= 5*speed_sum_modifier;
					break;
			  default:
					// unknown
					servo_duty = SERVO_DUTY_CENTER;
					motor_duty_left = 0;
					motor_duty_right = 0;
					speed_sum = 0;
					break;
			}
			
			// clip servo/motor duty cycles
			if (speed_sum > 600) {
				speed_sum = 600;
			} else if (speed_sum < -100) {
				speed_sum = -100;
			}
			if (servo_duty > SERVO_DUTY_MAX) servo_duty = SERVO_DUTY_MAX;
			if (servo_duty < SERVO_DUTY_MIN) servo_duty = SERVO_DUTY_MIN;
			if (motor_duty_left > 100) motor_duty_left = 100;
			if (motor_duty_left < 0) {
				motor_duty_left = fabs(motor_duty_left); // if negative, switch direction
				motor_dir_left = BACKWARD;
			}
			if (motor_duty_right > 100) motor_duty_right = 100;
			if (motor_duty_right < 0) {
				motor_duty_right = fabs(motor_duty_right); // if negative, switch direction
				motor_dir_right = BACKWARD;
			}
			
			// update duty cycles
			move_servo(servo_duty);
			move_motor_left(motor_duty_left, motor_dir_left);
			move_motor_right(motor_duty_right, motor_dir_right);
      
			// update last error
			last_error = error;
			last_pos = position;
		} else {
			// DRIVING DISABLED
			move_motor_left(0, motor_dir_left);
			move_motor_right(0, motor_dir_right);
		}

}

/* Hardware control to start the car. Select mode using SW3, start car using SW3. */
void car_start_switch(void) {
    // Wait for SW2 to be pressed
    while (!is_sw2_pressed()) {
        // allow for change of mode
        if (is_sw3_pressed()) {
            // wait for SW3 to be released
            while (is_sw3_pressed()) {}
            mode++;
            // set new mode LED
            if (mode == MODE_FAST) {
                led_on(LED_RED);
            } else if (mode == MODE_FAST2) {
                led_on(LED_MAGENTA);
            } else if (mode == MODE_MEDIUM) {
                led_on(LED_YELLOW);
            } else if (mode == MODE_MEDIUM2) {
                led_on(LED_BLUE);
            } else {
                mode = MODE_SLOW; // prevent mode overflow (reset mode)
                led_on(LED_GREEN);
            }
        }
    }
		// enable driving
		drive_en = 1;
}

/* DEBUG: Software (BT UART) control to start the car. */
void car_start_uart() {
    int mode_sel = 1;
    char mode_ch;
        
    // Prompt
    uart3_put("\r\nEnter mode (f, m, s): ");
    // Wait for UART data
    while (mode_sel) {
			  // get data
        if (UART3_S1 & UART_S1_RDRF_MASK) {
            mode_sel = 0;
            mode_ch = uart3_getchar();
        }
    }
    // mode select
    if (mode_ch == 'f') {
        mode = MODE_FAST;
        led_on(LED_RED);
        uart3_put("fast");
    } else if (mode_ch == 'm') {
        mode = MODE_MEDIUM;
        led_on(LED_YELLOW);
        uart3_put("medium");
    } else {
        mode = MODE_SLOW;
        led_on(LED_GREEN);
        uart3_put("slow");
    }
		// enable driving
		drive_en = 1;
}
