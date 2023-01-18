
#include "sw_pwm.h"

#include "gpio.h"

#include <asm/io.h> // ioremap(), iounmap()
#include <linux/delay.h> // mdelay()
#include <linux/errno.h> // ENOMEM

#include <linux/ktime.h>
#include <linux/hrtimer.h>


static ktime_t timer_period;
static struct hrtimer timer;

uint8_t pins[] = {
	20,
	21
};

typedef struct {
	uint32_t cnt;
	uint32_t moduo;
	uint32_t threshold;
} sw_pwm_t;
static sw_pwm_t sw_pwms[SW_PWM__N_CH];

static enum hrtimer_restart timer_callback(struct hrtimer* timer) {
	uint8_t ch;
	
	hrtimer_forward_now(timer, timer_period);
	
	for(ch = 0; ch < SW_PWM__N_CH; ch++){
		if(sw_pwms[ch].cnt == 0){
			gpio__set(pins[ch]);
		}
		if(sw_pwms[ch].cnt == sw_pwms[ch].threshold){
			gpio__clear(pins[ch]);
		}
		
		sw_pwms[ch].cnt++;
		if(sw_pwms[ch].cnt == sw_pwms[ch].moduo){
			sw_pwms[ch].cnt = 0;
		}
	}
	
	return HRTIMER_RESTART;
}

int sw_pwm__init(void) {
	uint8_t ch;
	
	for(ch = 0; ch < SW_PWM__N_CH; ch++){
		sw_pwms[ch].cnt = 0;
		sw_pwms[ch].moduo = 0;
		sw_pwms[ch].threshold = 1;
		gpio__clear(pins[ch]);
		gpio__steer_pinmux(pins[ch], GPIO__OUT);
	}
	
	timer_period = ktime_set(0, 10000); // 10us
	
	hrtimer_init(&timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL_HARD);
	timer.function = &timer_callback;
	hrtimer_start(&timer, timer_period, HRTIMER_MODE_REL_HARD);
	
	return 0;
}

void sw_pwm__exit(void) {
	uint8_t ch;
	
	hrtimer_cancel(&timer);
	
	for(ch = 0; ch < SW_PWM__N_CH; ch++){
		gpio__clear(pins[ch]);
		gpio__steer_pinmux(pins[ch], GPIO__IN);
	}
}


void sw_pwm__set_moduo(sw_pwm__ch_t ch, uint32_t moduo) {
	sw_pwms[ch].moduo = moduo;
	sw_pwms[ch].cnt = 0; // Reseting cnt.
	gpio__clear(pins[ch]);
}

/**
 * @a ch [0, HW_PWM__N_CH]
 * @a duty_permille [0, 1000]
 */
void sw_pwm__set_threshold(sw_pwm__ch_t ch, uint32_t threshold) {
	sw_pwms[ch].threshold = threshold;
	sw_pwms[ch].cnt = 0; // Reseting cnt.
	gpio__clear(pins[ch]);
}
