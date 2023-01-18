
#include "sw_pwm.h"

#include "gpio.h"

#include <asm/io.h> // ioremap(), iounmap()
#include <linux/delay.h> // mdelay()
#include <linux/errno.h> // ENOMEM

#include <linux/ktime.h> // ktime_t...
#include <linux/hrtimer.h> // hrtimer...

//TODO Debug
#undef SW_PWM__N_CH
#define SW_PWM__N_CH 1
#include <linux/sched.h>

static const uint8_t pins[SW_PWM__N_CH] = {
	20,
	//21,
	//22,
	//23
};

#define DEBUG_TIME(t) \
	do{ \
		printk(KERN_DEBUG "%s():%d: %s = %lld", __func__, __LINE__, #t, t); \
	}while(0)


typedef struct {
	struct hrtimer timer; // Must be first in struct.
sw_pwm__ch_t ch;
	uint8_t pin;
	bool on;
	uint32_t moduo;
	uint32_t threshold;
	spinlock_t pending_lock;
#if 0
	ktime_t on_interval_pending;
	ktime_t off_interval_pending;
	ktime_t on_interval;
	ktime_t off_interval;
#else
	u64 delta_on_pending; // [ns]
	u64 delta_off_pending; // [ns]
	u64 delta_on; // [ns]
	u64 delta_off; // [ns]
	u64 t_event;
#endif
} sw_pwm_t;
static sw_pwm_t sw_pwms[SW_PWM__N_CH];

static enum hrtimer_restart timer_callback(struct hrtimer* p_timer) {
	//TODO sw_pwm_t* ps = container_of(p_timer, sw_pwm_t, timer);
	sw_pwm_t* ps = (sw_pwm_t*)p_timer;
	unsigned long flags;
	u64 step;
	int i;
//TODO Debug
/*
struct task_struct *p = current;
static atomic_t debug_iters;
if(atomic_read(&debug_iters) < 30){
	printk(KERN_DEBUG "%s(): ch = %d tid=%d", __func__, (int)ps->ch, p->pid);
	atomic_add(1, &debug_iters);
}
*/
#if 0
	ps->on = !ps->on;
	if(ps->on){
		// Changing interval at the end of period.
		spin_lock_irqsave(&ps->pending_lock, flags);
		ps->on_interval = ps->on_interval_pending;
		ps->off_interval = ps->off_interval_pending;
		spin_unlock_irqrestore(&ps->pending_lock, flags);
		
		hrtimer_forward_now(&ps->timer, ps->on_interval);
		
		gpio__set(ps->pin);
	}else{
		hrtimer_forward_now(&ps->timer, ps->off_interval);
		
		gpio__clear(ps->pin);
	}
#else
	DEBUG_TIME(ktime_get_ns());
/*
	ps->on = !ps->on;
	
	i = 0;
	while(ktime_get_ns() > ps->t_event){
		i++;
	}
	if(i == 0){
		printk(KERN_WARNING "late");
	}
	
	if(ps->on){
		gpio__set(ps->pin);
	}else{
		gpio__clear(ps->pin);
	}
	
	if(ps->on){
		// Changing interval at the end of period.
		spin_lock_irqsave(&ps->pending_lock, flags);
		ps->delta_on = ps->delta_on_pending;
		ps->delta_off = ps->delta_off_pending;
		spin_unlock_irqrestore(&ps->pending_lock, flags);
	}
	step = ps->on ? ps->delta_on : ps->delta_off;
	hrtimer_forward(&ps->timer, ns_to_ktime(ps->t_event), ns_to_ktime(step));
	ps->t_event += step;
*/
	hrtimer_forward(&ps->timer, ktime_get(), ns_to_ktime(ps->delta_off));
#endif
	return HRTIMER_RESTART;
}

#define DELTA_US 400
static void set_intervals(sw_pwm_t* ps) {
	unsigned long flags;

#if 0
	// 10000 stands for 10 us.
	ktime_t on = ktime_set(0, (ps->threshold)*10000);
	ktime_t off = ktime_set(0, (ps->moduo - ps->threshold)*10000);

	spin_lock_irqsave(&ps->pending_lock, flags);
	ps->on_interval_pending = on;
	ps->off_interval_pending = off;
	spin_unlock_irqrestore(&ps->pending_lock, flags);
#else
	u64 delta_on = max(
		0,
		(int32_t)ps->threshold - DELTA_US/10
	)*10000;
	u64 delta_off = max(
		0,
		(int32_t)ps->moduo - (int32_t)ps->threshold - DELTA_US/10
	)*10000;
	spin_lock_irqsave(&ps->pending_lock, flags);
	ps->delta_on_pending = delta_on;
	ps->delta_off_pending = delta_off;
	spin_unlock_irqrestore(&ps->pending_lock, flags);
#endif
}

int sw_pwm__init(void) {
	sw_pwm__ch_t ch;
	sw_pwm_t* ps;
	//TODO Debug
	struct task_struct *p = current;
	printk(KERN_DEBUG "%s(): tid=%d", __func__, p->pid);

	for(ch = 0; ch < SW_PWM__N_CH; ch++){
		ps = &sw_pwms[ch];

ps->ch = ch;
		ps->pin = pins[ch];
		gpio__clear(ps->pin);
		gpio__steer_pinmux(ps->pin, GPIO__OUT);

		ps->on = false;

		spin_lock_init(&ps->pending_lock);


		hrtimer_init(
			&ps->timer,
			CLOCK_MONOTONIC,
			HRTIMER_MODE_REL_HARD
		);
		ps->timer.function = &timer_callback;
		ps->moduo = 1000;
		ps->threshold = 0;
		set_intervals(ps);
		//ps->on_interval = ps->on_interval_pending;
		//ps->off_interval = ps->off_interval_pending;
		ps->delta_on = ps->delta_on_pending;
		ps->delta_off = ps->delta_off_pending;
		ps->t_event = ktime_get_ns();
		DEBUG_TIME(ps->t_event);
		hrtimer_start(
			&ps->timer,
			ns_to_ktime(ps->delta_off),
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
