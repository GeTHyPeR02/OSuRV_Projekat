
#include "servo_fb.h"

#include "gpio.h"

#include <linux/printk.h>

#include <linux/gpio.h> // gpio sutff
#include <linux/interrupt.h> // irq stuff

#include "include/motor_ctrl.h" // DEV_NAME

static const uint8_t pins[SERVO_FB__N_CH] = {
	8,
	9,
	10,
	11,
	12,
	13
};

#define TIMES_EQU(t0, t1) ((t0).re == (t1).re || (t0).fe == (t1).fe)

typedef struct {
	uint8_t pin;
	int irq;
	u64 t_fe; // [ns]
	atomic64_t T_on; // [ns]
} servo_fb_t;
static servo_fb_t servo_fb[SERVO_FB__N_CH];


static irqreturn_t fb_isr(int irq, void* data) {
	u64 t = ktime_get_ns();
	servo_fb_t* p = (servo_fb_t*)data;
	bool s = gpio__read(p->pin);
	if(s){
		// It was rising edge.
		atomic64_set(&p->T_on, p->t_fe);
	}else{
		// It was falling edge.
		p->t_fe = t;
	}
	//TODO calc duty
	return IRQ_HANDLED;
}

#define LABEL_TEMPLATE "irq_servo_fb_X"

/**
 * Setup PWM feedback for 50 Hz.
 */
int servo_fb__init(void) {
	int r;
	servo_fb__ch_t ch;
	
	char label[] = LABEL_TEMPLATE;
	const int label_idx = sizeof(LABEL_TEMPLATE)-2; // positioned to X.
	
	for(ch = 0; ch < SERVO_FB__N_CH; ch++){
		servo_fb[ch].pin = pins[ch];
		label[label_idx] = '0' + ch;
		
		gpio__steer_pinmux(servo_fb[ch].pin, GPIO__IN);
		// Initialize GPIO ISR.
		r = gpio_request_one(
			servo_fb[ch].pin,
			GPIOF_IN,
			label
		);
		if(r){
			printk(
				KERN_ERR DEV_NAME": %s(): gpio_request_one() failed!\n",
				__func__
			);
			goto exit;
		}
		
		servo_fb[ch].irq = gpio_to_irq(servo_fb[ch].pin);
		r = request_irq(
			servo_fb[ch].irq,
			fb_isr,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			label,
			&servo_fb[ch]
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
		servo_fb__exit();
	}
	
	return 0;
}
void servo_fb__exit(void) {
	servo_fb__ch_t ch;

	for(ch = 0; ch < SERVO_FB__N_CH; ch++){
		disable_irq(servo_fb[ch].irq);
		free_irq(servo_fb[ch].irq, &servo_fb[ch]);
		gpio_free(servo_fb[ch].pin);
	}
}

#define INT64_T_ONE 1ULL
#define DIV_SHIFT 32
#define DIV_MUL ((INT64_T_ONE<<DIV_SHIFT)/20000)
#define DIV_ADD (INT64_T_ONE<<(DIV_SHIFT-1))

#define VOTERS 3
void servo_fb__get_pos_fb(servo_fb__ch_t ch, u16* pos_fb) {
	u64 T_on; // [ns]
	u16 duty; // [permille]
	
	// Make local copy of measurement.
	T_on = atomic64_read(&servo_fb[ch].T_on);
	
	// Hard-coded for 50 Hz.
	// duty = T_on/20000;
	duty = (T_on*DIV_MUL + DIV_ADD) >> DIV_SHIFT;
	
	*pos_fb = duty;
}
