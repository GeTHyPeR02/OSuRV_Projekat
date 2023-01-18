
#ifndef BLDC_H
#define BLDC_H

#include "include/motor_ctrl.h" // dir_t

#define BLDC__N_CH 1

typedef enum {
	// PWM - HW_PWM__CH_0, GPIO18, pin 12
	// DIR - GPIO17, pin 11
	// PG - GPIO16, pin 36
	BLDC__CH_0
} bldc__ch_t;

typedef enum {
	CW = +1,
	CCW = -1
} dir_t;

int bldc__init(void);
void bldc__exit(void);

void bldc__set_dir(bldc__ch_t ch, dir_t dir);
void bldc__set_duty(bldc__ch_t ch, u16 duty_permille);
void bldc__get_pulse_cnt(bldc__ch_t ch, int64_t* pulse_cnt);

#endif // BLDC_H
