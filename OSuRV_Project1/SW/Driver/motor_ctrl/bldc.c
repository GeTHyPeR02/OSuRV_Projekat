
#include "bldc.h"

#include "gpio.h"

#include <linux/printk.h>

#include <linux/gpio.h> // gpio stuff
#include <linux/interrupt.h> // irq stuff


typedef struct {
	uint8_t dir_pin;
	uint8_t pg_pin;
	const char* pg_label;
	int pg_irq;
	dir_t dir;
	volatile int64_t pulse_cnt;
} bldc_t;
static bldc_t bldc[] = {
	{
		17, // GPIO17, pin 11
		16, // GPIO16, pin 36
		"irq_gpio16",
		CW,
		0
	}
};


static irqreturn_t pg_isr(int irq, void* data) {
	bldc_t* p = (bldc_t*)data;
	p->pulse_cnt += p->dir;
	return IRQ_HANDLED;
}

int bldc__init(void) {
	int r;
	bldc__ch_t ch;
	
	for(ch = 0; ch < BLDC__N_CH; ch++){
		gpio__steer_pinmux(bldc[ch].dir_pin, GPIO__OUT);
		bldc__set_dir(ch, CW);
		
		
		gpio__steer_pinmux(bldc[ch].pg_pin, GPIO__IN);
		// Initialize GPIO ISR.
		r = gpio_request_one(
			bldc[ch].pg_pin,
			GPIOF_IN,
			bldc[ch].pg_label
		);
		if(r){
			printk(
				KERN_ERR DEV_NAME": %s(): gpio_request_one() failed!\n",
				__func__
			);
			goto exit;
		}
		
		bldc[ch].pg_irq = gpio_to_irq(bldc[ch].pg_pin);
		r = request_irq(
			bldc[ch].pg_irq,
			pg_isr,
			IRQF_TRIGGER_FALLING,
			bldc[ch].pg_label,
			&bldc[ch]
		);
		if(r){
			printk(
				KERN_ERR DEV_NAME": %s(): request_irq() failed!\n",
				__func__
			);
			goto exit;
		}
	}
	
exit:
	if(r){
		printk(KERN_ERR DEV_NAME": %s() failed with %d!\n", __func__, r);
		bldc__exit();
	}
	
	return r;
}

void bldc__exit(void) {
	bldc__ch_t ch;
	
	for(ch = 0; ch < BLDC__N_CH; ch++){
		disable_irq(bldc[ch].pg_irq);
		free_irq(bldc[ch].pg_irq, &bldc[ch]);
		gpio_free(bldc[ch].pg_pin);
	}
}


void bldc__set_dir(bldc__ch_t ch, dir_t dir) {
	if(ch >= BLDC__N_CH){
		return;
	}
	bldc[ch].dir = dir;
	if(dir == CW){
		gpio__set(bldc[ch].dir_pin);
	}else{
		gpio__clear(bldc[ch].dir_pin);
	}
}

void bldc__set_duty(bldc__ch_t ch, u16 duty_permille) {
	if(ch >= BLDC__N_CH){
		return;
	}
	// For SW PWM @ GPIO24, pin 18
}

void bldc__get_pulse_cnt(bldc__ch_t ch, int64_t* pulse_cnt) {
	if(ch >= BLDC__N_CH){
		return;
	}
	
	int64_t pulse_cnt2 = bldc[ch].pulse_cnt;
	// If inbetween reading pos,
	// pos is incremented by pos_pulse().
	if(pulse_cnt2 != bldc[ch].pulse_cnt){
		// Update it again,
		// because IRQ could changed it.
		// Do not need atomic,
		// because IRQ
		// will not occur for a long time.
		pulse_cnt2 = bldc[ch].pulse_cnt;
	}
	*pulse_cnt = pulse_cnt2;
}
