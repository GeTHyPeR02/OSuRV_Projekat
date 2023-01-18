
#include "sw_pwm.h"

#include "gpio.h"

#include <asm/io.h> // ioremap(), iounmap()
#include <linux/delay.h> // mdelay()
#include <linux/errno.h> // ENOMEM

#include <linux/ktime.h> // ktime_t...
#include <linux/hrtimer.h> // hrtimer...


static const uint8_t pins[SW_PWM__N_CH] = {
	20,
	21,
	22,
	23
};

typedef struct {
	struct hrtimer timer; // Must be first in struct.
	uint8_t pin;
	bool on;
	uint32_t moduo;
	uint32_t threshold;
	spinlock_t interval_pending_lock;
	ktime_t on_interval_pending;
	ktime_t off_interval_pending;
	ktime_t on_interval;
	ktime_t off_interval;
} sw_pwm_t;
static sw_pwm_t sw_pwms[SW_PWM__N_CH];

static enum hrtimer_restart timer_callback(struct hrtimer* p_timer) {
	//TODO sw_pwm_t* ps = container_of(p_timer, sw_pwm_t, timer);
	sw_pwm_t* ps = (sw_pwm_t*)p_timer;
	unsigned long flags;

	ps->on = !ps->on;
	if(ps->on){
		// Changing interval at the end of period.
		spin_lock_irqsave(&ps->interval_pending_lock, flags);
		ps->on_interval = ps->on_interval_pending;
		ps->off_interval = ps->off_interval_pending;
		spin_unlock_irqrestore(&ps->interval_pending_lock, flags);

		hrtimer_forward_now(&ps->timer, ps->on_interval);
		gpio__set(ps->pin);
	}else{
		hrtimer_forward_now(&ps->timer, ps->off_interval);
		gpio__clear(ps->pin);
	}

	return HRTIMER_RESTART;
}

static void set_intervals(sw_pwm_t* ps) {
	unsigned long flags;

	// 10000 stands for 10 us.
	ktime_t on = ktime_set(0, (ps->threshold)*10000);
	ktime_t off = ktime_set(0, (ps->moduo - ps->threshold)*10000);

	spin_lock_irqsave(&ps->interval_pending_lock, flags);
	ps->on_interval_pending = on;
	ps->off_interval_pending = off;
	spin_unlock_irqrestore(&ps->interval_pending_lock, flags);
}

int sw_pwm__init(void) {
	uint8_t ch;
	sw_pwm_t* ps;

	for(ch = 0; ch < SW_PWM__N_CH; ch++){
		ps = &sw_pwms[ch];

		ps->pin = pins[ch];
		gpio__clear(ps->pin);
		gpio__steer_pinmux(ps->pin, GPIO__OUT);

		ps->on = true;

		spin_lock_init(&ps->interval_pending_lock);

		ps->moduo = 1000;
		ps->threshold = 0;
		set_intervals(ps);
		ps->on_interval = ps->on_interval_pending;
		ps->off_interval = ps->off_interval_pending;

		hrtimer_init(
			&ps->timer,
			CLOCK_MONOTONIC,
			HRTIMER_MODE_REL_HARD
		);
		ps->timer.function = &timer_callback;
		hrtimer_start(
			&ps->timer,
			ps->off_interval,
			HRTIMER_MODE_REL_HARD
		);
	}

	return 0;
}

void sw_pwm__exit(void) {
	uint8_t ch;
	sw_pwm_t* ps;

	for(ch = 0; ch < SW_PWM__N_CH; ch++){
		ps = &sw_pwms[ch];

		hrtimer_cancel(&ps->timer);

		gpio__clear(ps->pin);
		gpio__steer_pinmux(ps->pin, GPIO__IN);
	}
}


void sw_pwm__set_moduo(sw_pwm__ch_t ch, uint32_t moduo) {
	if(ch >= SW_PWM__N_CH){
		return;
	}
	sw_pwms[ch].moduo = moduo;
	set_intervals(&sw_pwms[ch]);
}

void sw_pwm__set_threshold(sw_pwm__ch_t ch, uint32_t threshold) {
	if(ch >= SW_PWM__N_CH){
		return;
	}
	sw_pwms[ch].threshold = threshold;
	set_intervals(&sw_pwms[ch]);
}
