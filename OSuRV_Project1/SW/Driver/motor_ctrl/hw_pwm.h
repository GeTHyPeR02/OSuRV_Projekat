
#ifndef HW_PWM_H
#define HW_PWM_H


#include <linux/types.h>

#define HW_PWM__N_CH 2

typedef enum {
	HW_PWM__CH_0, // GPIO18, pin 12
	HW_PWM__CH_1  // GPIO19, pin 35
} hw_pwm__ch_t;

/**
 * Setup PWM to 100kHz i.e. 10us.
 */
int hw_pwm__init(void);
void hw_pwm__exit(void);

void hw_pwm__set_moduo(hw_pwm__ch_t ch, uint32_t moduo);
/**
 * @a threshold How much of moduo PWM is ON.
 */
void hw_pwm__set_threshold(hw_pwm__ch_t ch, uint32_t threshold);


#endif // HW_PWM_H
