#ifndef CONFIG_H
#define CONFIG_H


/* --- IMU Configuration --- */
#define IMU_I2C_ADDRESS 0x68


/* --- H-bridge motor pins --- */
#define MOTOR_A_IN1_PIN 2
#define MOTOR_A_IN2_PIN 3
#define MOTOR_B_IN3_PIN 5
#define MOTOR_B_IN4_PIN 6

/* --- LED Configuration --- */
#define LED_POWER_PIN 13
#define LED_STATUS_PIN 12
#define LED_STATUS_BLINK_INTERVAL_MS 250

/* --- Motor deployment timing --- */
#define DEPLOY_MOTOR_ON_TIME_MS 2000
#define DEPLOYMENT_MOTOR_OUTPUT_ENABLED 1

/* --- BMP / sensor timing --- */
#define BMP_SAMPLE_INTERVAL_MS 20

#endif