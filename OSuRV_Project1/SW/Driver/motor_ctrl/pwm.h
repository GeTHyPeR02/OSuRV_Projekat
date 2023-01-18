
#ifndef PWM_H
#define PWM_H


#include <linux/types.h>

typedef uint8_t pwm__ch_t;

/**
 * Setup PWM to 100kHz i.e. 10us.
 */
int pwm__init(void);
void pwm__exit(void);

void pwm__set_moduo(pwm__ch_t ch, uint32_t moduo);
/**
 * @a threshold How much of moduo PWM is ON.
 */
void pwm__set_threshold(pwm__ch_t ch, uint32_t threshold);


#endif // PWM_H
