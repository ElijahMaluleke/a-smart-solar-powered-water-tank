/********************************************************************************
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 ********************************************************************************/
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <stdio.h>
#include "vibration_motor.h"

#define VM_DEVICE_GPIO0 DT_NODELABEL(gpio0)
#define VM_DEVICE_GPIO1 DT_NODELABEL(gpio1)

/* const struct device *gpio_dev; */
const struct device *vm_gpio0_dev = DEVICE_DT_GET(VM_DEVICE_GPIO0);

/* const struct device *gpio_dev; */
const struct device *vm_gpio1_dev = DEVICE_DT_GET(VM_DEVICE_GPIO1);

/********************************************************************************
 * @} vibration_motor
 ********************************************************************************/
void vibration_motor_init(void) {
	/* set up VIBRATION MOTOR pins */
	gpio_pin_configure(vm_gpio0_dev, VIBRATION_MOTOR, GPIO_OUTPUT_INACTIVE);	
	gpio_pin_set(vm_gpio0_dev, VIBRATION_MOTOR, false);
}

/********************************************************************************
 * @} vibration_motor
 ********************************************************************************/
void vibration_motor(uint32_t VibrationCount, uint8_t SleepTime) {
	uint32_t i;
	
	for(i = 0; i < VibrationCount; i++) {
		gpio_pin_set(vm_gpio0_dev, VIBRATION_MOTOR, true);
		k_msleep(VIBRATION_MOTOR_ON_SLEEP_TIME * SleepTime);
		gpio_pin_set(vm_gpio0_dev, VIBRATION_MOTOR, false);
		k_msleep(VIBRATION_MOTOR_OFF_SLEEP_TIME);
	}
}
