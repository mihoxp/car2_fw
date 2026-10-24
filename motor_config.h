// Motor wiring for the YAHBOOM Pico car, taken from YAHBOOM's pico_car.py
// (Car_Run: R_A=GP10 and L_B=GP13 get PWM, R_B=GP11 and L_A=GP12 stay low).
// Set MOTOR_ENABLED to 0 to only print cmd_vel without driving the motors.
#pragma once

#define MOTOR_ENABLED   1

// Two-pin PWM driver per motor: forward = PWM on A, B low
#define LEFT_PIN_A      13
#define LEFT_PIN_B      12
#define RIGHT_PIN_A     10
#define RIGHT_PIN_B     11

#define MOTOR_PWM_HZ    1000   // same as YAHBOOM's library

// 1: print PWM registers and measured pin duty every 10th cmd_vel
#define PWM_DEBUG       0

// Set to 1 if a wheel spins the wrong way
#define LEFT_INVERT     0
#define RIGHT_INVERT    0

// Differential drive geometry
#define WHEEL_SEPARATION_M  0.10f   // distance between the two wheels
#define MAX_WHEEL_SPEED_MPS 0.50f   // wheel speed that maps to 100% duty

// Lowest duty that moves the car on the floor (measured 2026-10-24: 40% and
// 55% stall, 70% moves). Any non-zero command starts here.
#define MOTOR_MIN_DUTY      0.60f

// Stop the motors if no cmd_vel arrives for this long
#define CMD_TIMEOUT_MS      500
