#pragma once
#include <Arduino.h>
#include <math.h>
#include "variable.h"
#include "servos.h"

// Gait control zmienne
extern bool running;
extern unsigned long last_gait_time;
extern float gait_phase;
extern const unsigned long GAIT_DT;

// Tryby chodu
enum GaitMode {
    CREEP_FORWARD,
    CREEP_BACKWARD,
    CREEP_RIGHT,
    CREEP_LEFT,
    TROT_FORWARD,
    TROT_BACKWARD,
    TROT_LEFT,
    TROT_RIGHT,
    TROT_MOVE_RIGHT,
    TROT_MOVE_LEFT
};

// Parametry chodu
struct GaitParams {
    float x_amps[4];
    float z_amps[4];
    float x_offsets[4];
    float phase_offsets[4];
};

// Mapowanie serw
struct ServoMapping {
    int servo_id;
    const char* leg;
    const char* axis;
};

// Deklaracje tablic
extern const GaitParams GAIT_CONFIGS[];
extern const ServoMapping SERVO_MAPPING[8];

// Deklaracje funkcji
void creep_gait(float x_amp, float z_amp, float x_off, float z_off, float phase, float& z, float& x);
void trot_gait(float x_amp, float z_amp, float x_off, float z_off, float phase, float& z, float& x);
void calculate_gait_angles(GaitMode mode, float phase, float angles[4][2]);
void execute_gait(GaitMode mode);