
DEV_NAME := motor_ctrl
DEV_MAJOR := 260

TARGET := $(DEV_NAME).ko

MDIR := drivers/pwm

CURRENT := $(shell uname -r)
KDIR := /lib/modules/$(CURRENT)/build
PWD := $(shell pwd)
DEST := /lib/modules/$(CURRENT)/kernel/$(MDIR)
EXTRA_CFLAGS := -I$(PWD) -DDEV_MAJOR=$(DEV_MAJOR)

obj-m := motor_ctrl.o
motor_ctrl-objs := gpio.o hw_pwm.o sw_pwm.o pwm.o servo_fb.o bldc.o main.o

.PHONY: default
default: build

.PHONY: prerequisites
prerequisites:
	sudo apt -y install raspberrypi-kernel raspberrypi-kernel-headers

.PHONY: build
build: stop check_is_major_free
	$(MAKE) -C $(KDIR) M=$(PWD)

.PHONY: check_is_major_free
check_is_major_free:
	if [ `cat /proc/devices | grep -c $(DEV_MAJOR)` != 0 ]; \
	then \
		echo "Major $(DEV_MAJOR) is occupied!"; \
		exit 1; \
	fi


DEV_FN := /dev/$(DEV_NAME)
.PHONY: start
start:
	sudo insmod $(TARGET)
	sudo rm -f $(DEV_FN)
	sudo mknod $(DEV_FN) c $(DEV_MAJOR) 0
	sudo chmod a+rw $(DEV_FN)
	
.PHONY: stop
stop:
	if [ `lsmod | grep -c $(DEV_NAME)` != 0 ]; \
	then \
		sudo rmmod $(TARGET:%.ko=%); \
	fi

.PHONY: install
install:
	sudo cp $(TARGET) $(DEST)
	sudo depmod -a

.PHONY: clean
clean:
	rm -f *.o $(TARGET) .*.cmd .*.flags *.mod.c *. \
		modules.order Module.symvers *.mod

-include $(KDIR)/Rules.make
