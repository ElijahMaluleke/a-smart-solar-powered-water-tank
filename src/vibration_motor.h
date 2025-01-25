/********************************************************************************
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 ********************************************************************************/
#ifndef VIBRATION_MOTOR_H 
#define VIBRATION_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#define VIBRATION_MOTOR_ON_SLEEP_TIME	100
#define VIBRATION_MOTOR_OFF_SLEEP_TIME	50

#define VIBRATION_MOTOR					29

void vibration_motor_init(void);
void vibration_motor(uint32_t VibrationCount, uint8_t SleepTime);

#ifdef __cplusplus
}
#endif

#endif

/********************************************************************************
 * 
 ********************************************************************************/