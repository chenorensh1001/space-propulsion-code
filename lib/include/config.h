#ifndef CONFIG_H
#define CONFIG_H

/* --- IMU Configuration --- */
#define IMU_I2C_ADDRESS 0x68

/* --- Deployment motor / H-bridge configuration --- */

// Teensy pin 2 -> H-bridge IN1
// Teensy pin 3 -> H-bridge IN2
// Teensy pin 5 -> H-bridge IN3
// Teensy pin 6 -> H-bridge IN4
#define MOTOR_A_IN1_PIN 2
#define MOTOR_A_IN2_PIN 3
#define MOTOR_B_IN3_PIN 5
#define MOTOR_B_IN4_PIN 6

/* --- Motor deployment safety --- */

// 0 = software test only, motors will NOT be driven
// 1 = motors/H-bridge outputs are active
#define DEPLOYMENT_MOTOR_OUTPUT_ENABLED 1

/* --- Motor deployment timing --- */

// Start short for testing.
// Later, increase only if the mechanism needs more time.
#define DEPLOY_MOTOR_ON_TIME_MS 10000
#define RETRACT_MOTOR_ON_TIME_MS 0

/* --- Motor PWM --- */

// If your H-bridge motor supply is 5 V and your motors are 3 V,
// use about 150.
//
// If you power the H-bridge motor side with a true 3 V supply,
// you can use 255.
#define DEPLOY_MOTOR_PWM 250

/* --- LED Configuration --- */
#define LED_POWER_PIN 13
#define LED_STATUS_PIN 12
#define LED_STATUS_BLINK_INTERVAL_MS 250

/* --- BMP / sensor timing --- */
#define BMP_SAMPLE_INTERVAL_MS 20

#endif