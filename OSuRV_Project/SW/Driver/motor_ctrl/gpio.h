
#ifndef GPIO_H
#define GPIO_H

#include <linux/types.h>

#define BCM2708_PERI_BASE 0x3F000000

int gpio__init(void);
void gpio__exit(void);

typedef enum {
	GPIO__IN = 0b000,
	GPIO__OUT = 0b001,
	GPIO__ALT_FUN_0 = 0b100,
	GPIO__ALT_FUN_1 = 0b101,
	GPIO__ALT_FUN_2 = 0b110,
	GPIO__ALT_FUN_3 = 0b111,
	GPIO__ALT_FUN_4 = 0b011,
	GPIO__ALT_FUN_5 = 0b010,
} gpio__pinmux_fun_t;

/**
 * Set pinmux i.e. functiona of pin to input, output, or some other peripheral.
 * @a pin [2, 26] TODO check it.
 */
void gpio__steer_pinmux(uint8_t pin, gpio__pinmux_fun_t pinmux_fun);

void gpio__set(uint8_t pin);
void gpio__clear(uint8_t pin);
uint8_t gpio__read(uint8_t pin);

#endif // GPIO_H
