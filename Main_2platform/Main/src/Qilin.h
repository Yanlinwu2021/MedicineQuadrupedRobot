#ifndef QILIN_H
#define QILIN_H

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define PI 3.1415
#define FEMUR_L 60
#define TIBIA_L 95.334
#define TIBIA_ANGLE 17.543
#define Y_OFFSET 55
#define Z_OFFSET -90
#define J1_OFFSET 90
#define J2_OFFSET 20
#define J3_OFFSET 20
#define STEP_RADIUS 12

#define SERVO_MIN 150
#define SERVO_MAX 600

extern Adafruit_PWMServoDriver pwm;

extern double x_start[4];
extern double y_start[4];
extern double x_target[4];
extern double y_target[4];
extern double J1_fixed_angles[4];

void setup();
void setServoAngle(uint8_t num, uint16_t angle);
void move_joints(int leg, double j1, double j2, double j3);
void move_xyz(int leg, double x, double y, double z);

#endif // QILIN_H
