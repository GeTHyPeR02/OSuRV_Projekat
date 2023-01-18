
#ifndef SERVO_FB_H
#define SERVO_FB_H


#include <linux/types.h>

#define SERVO_FB__N_CH 6

typedef enum {
	SERVO_FB__CH_0, // GPIO08, pin 24
	SERVO_FB__CH_1, // GPIO09, pin 21
	SERVO_FB__CH_2, // GPIO10, pin 19
	SERVO_FB__CH_3, // GPIO11, pin 23
	SERVO_FB__CH_4, // GPIO12, pin 32
	SERVO_FB__CH_5  // GPIO13, pin 33
} servo_fb__ch_t;

/**
 * Setup PWM feedback.
 */
int servo_fb__init(void);
void servo_fb__exit(void);

/**
 * @p pos_fb duty permilles.
 */
void servo_fb__get_pos_fb(servo_fb__ch_t ch, u16* pos_fb);

#endif // SERVO_FB_H
