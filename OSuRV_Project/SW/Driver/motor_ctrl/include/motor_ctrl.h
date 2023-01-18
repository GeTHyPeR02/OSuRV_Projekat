
#ifndef MOTOR_CLTR_H
#define MOTOR_CLTR_H

// For uint16_t.
#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
#endif


#define DEV_NAME "motor_ctrl"
#define DEV_FN "/dev/motor_ctrl"

#include <linux/ioctl.h> // For _IOW and _IORW

typedef struct {
	uint8_t ch;
	uint16_t moduo;
} motor_ctrl__ioctl_arg_moduo_t;

#define MOTOR_CLTR__N_SERVO 6
#define MOTOR_CLTR__N_BLDC 1

typedef struct {
	int16_t pos_fb[MOTOR_CLTR__N_SERVO];
	int64_t pulse_cnt_fb[MOTOR_CLTR__N_BLDC];
} motor_ctrl__read_arg_fb_t;


#define IOCTL_MOTOR_CLTR_SET_MODUO \
	_IOW('s', 0, motor_ctrl__ioctl_arg_moduo_t)


#endif // MOTOR_CLTR_H
