/**
 * ESP-Drone Firmware - Servo Controller
 * 
 * Minimal controller that bypasses stabilization and directly maps
 * CRTP thrust/roll/pitch/yaw commands to MG995 servo motors.
 *
 * Copyright 2019-2020 Espressif Systems (Shanghai)
 * GPL3.0 License
 */

#ifndef SERVO_CONTROLLER_H_
#define SERVO_CONTROLLER_H_

#include <stdbool.h>
#include <stdint.h>

/**
 * Initialize the servo controller
 */
void servoControllerInit(void);

/**
 * Test the servo controller
 */
bool servoControllerTest(void);

#endif /* SERVO_CONTROLLER_H_ */
