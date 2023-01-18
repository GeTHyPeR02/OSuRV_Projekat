

#include "gpio.h"

#include <asm/io.h> // ioremap(), iounmap()
#include <linux/errno.h> // ENOMEM

#include "include/motor_ctrl.h" // DEV_NAME

/*
NOTE: Check Broadcom BCM8325 datasheet, page 91+
	GPIO Base address is set to 0x7E20 0000,
	but it is VC CPU BUS address, while the
	ARM physical address is 0x3F20 0000, what
	can be seen in pages 5-7 of Broadcom
	BCM8325 datasheet, having in mind that
	total system ram is 0x3F000000 (1GB - 16MB)
	instead of 0x20000000 (512 MB)
*/
#define GPIO_BASE            (BCM2708_PERI_BASE + 0x200000)                                                             //PWM BAZA
#define GPIO_ADDR_SPACE_LEN  (0xB4)

// Virtual address where the physical GPIO address is mapped.
static void* virt_gpio_base;

int gpio__init(void) {
	int r = 0;
	
	virt_gpio_base = ioremap(GPIO_BASE, GPIO_ADDR_SPACE_LEN);
	if(!virt_gpio_base){
		r = -ENOMEM;
		goto exit;
	}

exit:
	if(r){
		printk(KERN_ERR DEV_NAME": %s() failed with %d!\n", __func__, r);
		gpio__exit();
	}
	return r;
}
void gpio__exit(void) {
	if(virt_gpio_base){
		iounmap(virt_gpio_base);
		virt_gpio_base = 0;
	}
}


/**
 * @return 1 to error to exit, 0 if all Ok.
 */
#define check_pin(pin) \
	(pin < 2 || 26 < pin) ? ( \
			printk( \
				KERN_WARNING DEV_NAME": %s(): %d out of range [2, 26]!\n", \
				__func__, \
				pin \
			), 1 \
		) : 0


typedef struct {
	uint8_t reg; // in bytes
	uint8_t shift; // in bits
} offsets_t;
static offsets_t gpfsel_offsets_table[] = {
	{0*4, 0*3}, // 0
	{0*4, 1*3}, // 1
	{0*4, 2*3}, // 2
	{0*4, 3*3}, // 3
	{0*4, 4*3}, // 4
	{0*4, 5*3}, // 5
	{0*4, 6*3}, // 6
	{0*4, 7*3}, // 7
	{0*4, 8*3}, // 8
	{0*4, 9*3}, // 9
	{1*4, 0*3}, // 10
	{1*4, 1*3}, // 11
	{1*4, 2*3}, // 12
	{1*4, 3*3}, // 13
	{1*4, 4*3}, // 14
	{1*4, 5*3}, // 15
	{1*4, 6*3}, // 16
	{1*4, 7*3}, // 17
	{1*4, 8*3}, // 18
	{1*4, 9*3}, // 19
	{2*4, 0*3}, // 20
	{2*4, 1*3}, // 21
	{2*4, 2*3}, // 22
	{2*4, 3*3}, // 23
	{2*4, 4*3}, // 24
	{2*4, 5*3}, // 25
	{2*4, 6*3} // 26
};

#define get_gpfsel_offsets(pin, reg, idx) \
	do{ \
		reg = gpfsel_offsets_table[pin].reg; \
		shift = gpfsel_offsets_table[pin].shift; \
	}while(0)


void gpio__steer_pinmux(uint8_t pin, gpio__pinmux_fun_t pinmux_fun) {
	uint8_t reg;
	uint8_t shift;
	uint32_t tmp;
	
	if(check_pin(pin)){
		return;
	}
	if(!virt_gpio_base){
		return;
	}

	get_gpfsel_offsets(pin, reg, idx);

	// Read whole register.
	tmp = ioread32(virt_gpio_base + reg);
	
	// Clear 3b field.
	tmp &= ~(0b111 << shift);
	
	// Set 3b with appropried function
	tmp |= pinmux_fun << shift;

	// Write back updated value.
	iowrite32(tmp, virt_gpio_base + reg);
}


#define GPSET0_OFFSET 0x1C
#define GPSET1_OFFSET 0x20
#define GPCLR0_OFFSET 0x28
#define GPCLR1_OFFSET 0x2C
#define GPLEV0_OFFSET 0x34
#define GPLEV1_OFFSET 0x38
#define GPPUD_OFFSET 0x94
#define GPPUDCLK0_OFFSET 0x98
#define GPPUDCLK1_OFFSET 0x9C

//TODO gpio__set_pull_resistor()

//TODO test these functions..
void gpio__set(uint8_t pin) {
#if 0
	// For pins [2, 52].
	uint8_t reg;
	uint8_t shift;

	if(check_pin(pin)){
		return;
	}
	if(!virt_gpio_base){
		return;
	}

	if(pin < 32){
		reg = GPSET0_OFFSET;
		shift = pin;
	}else{
		reg = GPSET1_OFFSET;
		shift = pin-32;
	}
	iowrite32(0x1 << shift, virt_gpio_base + reg);
#else
	// For pins [2, 26].
	if(check_pin(pin)){
		return;
	}
	if(!virt_gpio_base){
		return;
	}
	iowrite32(0x1 << pin, virt_gpio_base + GPSET0_OFFSET);
#endif
}

void gpio__clear(uint8_t pin) {
	if(check_pin(pin)){
		return;
	}
	if(!virt_gpio_base){
		return;
	}
	iowrite32(0x1 << pin, virt_gpio_base + GPCLR0_OFFSET);
}

uint8_t gpio__read(uint8_t pin) {
	uint32_t tmp;
	if(check_pin(pin)){
		return -1;
	}
	if(!virt_gpio_base){
		return -1;
	}
	tmp = ioread32(virt_gpio_base + GPLEV0_OFFSET);
	return tmp>>pin & 1;
}


