#pragma once
#include <Arduino.h>
#include <SCServo.h>
#include "variable.h"
#include "servos.h"


// Deklaracje zmiennych globalnych
extern SMS_STS st;
extern const int sts_id[8];
extern const int acc;
extern const int speed;
extern const int SERVO_TRIMS[8];
extern const int SERVO_LIMITS[8][2];

// Deklaracje funkcji
int angle_deg_to_servo(float deg);
int check_angle_limit(int id, int angle_deg);
void move_servo(int id, int angle_deg);
void move_servo_smooth(int id, int angle_deg);
void return_to_neutral();