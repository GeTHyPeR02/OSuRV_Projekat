
#ifndef MOTOR_CLTR_PWM_MAP_H
#define MOTOR_CLTR_PWM_MAP_H

// For uint8_t.
#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
#endif


#define N_PWM_MAP 3
#define N_PWM_CH 6
#define N_PWM_SHARE 2

typedef enum {NONE, HW, SW} pwm_type_t;
typedef struct {
	pwm_type_t type;
	uint8_t ch;
} pwm_map_elem_t;

static const pwm_map_elem_t pwm_map[N_PWM_MAP][N_PWM_CH][N_PWM_SHARE] = {
	// To run both chassis and arm.
	// HW0 HW1 SW0 SW1 SW2 SW3
	{
		{{HW, 0}},
		{{HW, 1}},
		{{SW, 0}},
		{{SW, 1}},
		{{SW, 2}},
		{{SW, 3}}
	},
	// To run both chassis and arm, but more sever Parkinson.
	// SW0 SW1 SW2 SW3 HW0 HW1
	{
		{{SW, 0}},
		{{SW, 1}},
		{{SW, 2}},
		{{SW, 3}},
		{{HW, 0}},
		{{HW, 1}}
	},
	// More robust, but cannot run both chassis and arm.
	// HW0&SW2 HW1&SW3 SW0 SW1
	{
		{{HW, 0}, {SW, 2}},
		{{HW, 1}, {SW, 3}},
		{{SW, 2}},
		{{SW, 3}}
	}
};


#define ACTIVE_PWM_MAP 1


#endif // MOTOR_CLTR_PWM_MAP_H
