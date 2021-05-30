#ifndef PWM_H_
#define PWM_H_

#define SERVO_DUTY_MIN     5.5
#define SERVO_DUTY_CENTER  7.5
#define SERVO_DUTY_MAX     9.5

#define FORWARD  1
#define BACKWARD 0

#define MOTOR_FREQ 10000
#define SERVO_FREQ 50

void init_pwm(void);
void init_FTM0(void);
void FTM0_set_duty_cycle(unsigned int duty_cycle, unsigned int frequency, int motor, int dir);
void init_FTM3(void);
void FTM3_set_duty_cycle(double duty_cycle, unsigned int frequency);
void move_servo(double duty);
void move_motor_left(unsigned int duty, int dir);
void move_motor_right(unsigned int duty, int dir);

#endif /* PWM_H_ */
