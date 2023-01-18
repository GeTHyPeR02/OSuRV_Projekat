
#include "hw_pwm.h"

#include <asm/io.h> // ioremap(), iounmap()
#include <linux/delay.h> // mdelay()
#include <linux/errno.h> // ENOMEM

#include "gpio.h"
#include "include/motor_ctrl.h" // DEV_NAME

#define CLOCK_BASE           (BCM2708_PERI_BASE + 0x101000)                                                             //GPIO BAZA
#define PWM_BASE             (BCM2708_PERI_BASE + 0x20C000)   
#define PWM_ADDR_SPACE_LEN   0x28
#define CLOCK_ADDR_SPACE_LEN 0xC4

#define BCM_PASSWORD (0x5A << 24)


// PWM reg offsets.

// Control register
#define CTL_OFFSET 0x00000000
// Range1 register
#define RNG1_OFFSET 0x00000010
// Data1 register
#define DAT1_OFFSET 0x00000014
// Range2 register
#define RNG2_OFFSET 0x00000020
// Data2 register
#define DAT2_OFFSET 0x00000024
// Pwm Clock Control register
#define PWM_CLOCK_CONTROL 0x000000A0
// Pwm Clock Divider register
#define PWM_CLOCK_DIVIDER 0x000000A4

static void* virt_pwm_clock_base;
static void* virt_pwm_base;

static void set_CLK(uint16_t divider){
	divider &= 0xfff;

	// Stop PWM clock.
	iowrite32(
		BCM_PASSWORD | 0x01,
		virt_pwm_clock_base + PWM_CLOCK_CONTROL
	);
	mdelay(110);

	// Wait for busy to be 0.
	while( (ioread32(virt_pwm_clock_base + PWM_CLOCK_CONTROL) & 0x80) != 0 ) {
		mdelay(1);
	}

	iowrite32(
		BCM_PASSWORD | (divider << 12),
		virt_pwm_clock_base + PWM_CLOCK_DIVIDER
	);

	// Enable PWM clock and source = OSC
	iowrite32(
		BCM_PASSWORD | 0x11,
		virt_pwm_clock_base + PWM_CLOCK_CONTROL
	);
}

static void set_CTL(hw_pwm__ch_t ch, uint8_t field){
	unsigned int tmp;
	tmp = ioread32(virt_pwm_base + CTL_OFFSET);
	tmp &= ~((uint32_t)0xff << ch*8);
	tmp |= (uint32_t)field << ch*8;
	iowrite32(tmp, virt_pwm_base + CTL_OFFSET);
}


int hw_pwm__init(void) {
	int r = 0;

	virt_pwm_clock_base = ioremap(CLOCK_BASE, CLOCK_ADDR_SPACE_LEN);
	if(!virt_pwm_clock_base){
		r = -ENOMEM;
		goto exit;
	}

	virt_pwm_base = ioremap(PWM_BASE, PWM_ADDR_SPACE_LEN);
	if(!virt_pwm_base){
		r = -ENOMEM;
		goto exit;
	}
	
	// Set pinmux from GPIO to PWM.
	gpio__steer_pinmux(18, GPIO__ALT_FUN_5);
	gpio__steer_pinmux(19, GPIO__ALT_FUN_5);
	
	// 19.2MHz clock / 192 = 100kHz i.e. 10us.
	set_CLK(192);
	
	// 0x80 // MS transmission.
	// 0x01 // Enable channel.
	set_CTL(HW_PWM__CH_0, 0x81);
	set_CTL(HW_PWM__CH_1, 0x81);
	
exit:
	if(r){
		printk(KERN_ERR DEV_NAME": %s() failed with %d!\n", __func__, r);
		hw_pwm__exit();
	}
	return r;
}

void hw_pwm__exit(void) {
	if(virt_pwm_base){
		hw_pwm__set_threshold(HW_PWM__CH_0, 0);
		hw_pwm__set_threshold(HW_PWM__CH_1, 0);

		hw_pwm__set_moduo(HW_PWM__CH_0, 0x20);
		hw_pwm__set_moduo(HW_PWM__CH_1, 0x20);
		
		set_CTL(HW_PWM__CH_0, 0);
		set_CTL(HW_PWM__CH_1, 0);

		gpio__steer_pinmux(18, GPIO__ALT_FUN_0);
		gpio__steer_pinmux(19, GPIO__ALT_FUN_0);
		
		iounmap(virt_pwm_base);
		virt_pwm_base = 0;
	}
	if(virt_pwm_clock_base){
		iounmap(virt_pwm_clock_base);
		virt_pwm_clock_base = 0;
	}
}


void hw_pwm__set_moduo(hw_pwm__ch_t ch, uint32_t moduo) {
	const static uint32_t offsets[2] = {RNG1_OFFSET, RNG2_OFFSET};
	if(!virt_pwm_base){
		return;
	}
	if(ch >= HW_PWM__N_CH){
		return;
	}
	iowrite32(moduo, virt_pwm_base + offsets[ch]);
}

void hw_pwm__set_threshold(hw_pwm__ch_t ch, uint32_t threshold) {
	const static uint32_t offsets[2] = {DAT1_OFFSET, DAT2_OFFSET};
	if(!virt_pwm_base){
		return;
	}
	if(ch >= HW_PWM__N_CH){
		return;
	}
	// Set DAT register.
	iowrite32(threshold, virt_pwm_base + offsets[ch]);
}
