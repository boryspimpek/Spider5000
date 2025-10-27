// gait.h

#include <math.h>

// Gait parameters
const int x_amp = 30;               // x amplitude
const int z_amp = 15;               // z amplitude
const int OFFSET_FRONT = 0;         // front leg offset
const int OFFSET_BACK = 45;         // back legs offset

// Gait control
const unsigned long GAIT_DT = 50;           // 50ms = 0.05s
bool running = false;
unsigned long last_gait_time = 0;
float gait_phase = 0.0;

enum GaitMode {
    CREEP_FORWARD,
    CREEP_BACKWARD,
    CREEP_RIGHT,
    CREEP_LEFT,
};

struct GaitParams {
    float x_amps[4];
    float z_amps[4];
    float x_offsets[4];
    float phase_offsets[4];
};

const GaitParams GAIT_CONFIGS[] = {
    // CREEP_FORWARD
    {
        {-x_amp, x_amp, -x_amp, x_amp},
        {z_amp, -z_amp, -z_amp, z_amp},
        {90 - OFFSET_FRONT, 90 + OFFSET_FRONT, 90 + OFFSET_BACK, 90 - OFFSET_BACK},
        // {0.00, 0.50, 0.25, 0.75}
        {0.75, 0.25, 0.00, 0.50}
    },
    // CREEP_BACKWARD
    {
        {x_amp, -x_amp, x_amp, -x_amp},
        {z_amp, -z_amp, -z_amp, z_amp},
        {90 - OFFSET_BACK, 90 + OFFSET_BACK, 90 + OFFSET_FRONT, 90 - OFFSET_FRONT},
        {0.50, 0.25, 0.00, 0.75}
    },
    // CREEP_RIGHT
    {
        {-x_amp, -x_amp, -x_amp, -x_amp},
        {z_amp, -z_amp, -z_amp, z_amp},
        {45+x_amp/2, 135+x_amp/2, 135+x_amp/2, 45+x_amp/2},
        {0.00, 0.50, 0.25, 0.75}
    },
    // CREEP_LEFT
    {
        {x_amp, x_amp, x_amp, x_amp},
        {z_amp, -z_amp, -z_amp, z_amp},
        {45-x_amp/2, 135-x_amp/2, 135-x_amp/2, 45-x_amp/2},
        {0.00, 0.50, 0.25, 0.75}
    }
};

struct ServoMapping {
    int servo_id;
    const char* leg;
    const char* axis;
};

const ServoMapping SERVO_MAPPING[8] = {
    {1, "lf", "x"}, {2, "lf", "z"},  // Servo 1: LF X, Servo 2: LF Z
    {3, "rf", "x"}, {4, "rf", "z"},  // Servo 3: RF X, Servo 4: RF Z
    {5, "lr", "x"}, {6, "lr", "z"},  // Servo 5: LR X, Servo 6: LR Z
    {7, "rr", "x"}, {8, "rr", "z"}   // Servo 7: RR X, Servo 8: RR Z
};

void creep_gait(float x_amp, float z_amp, float x_off, float z_off, float phase, float& z, float& x) {
    // LIFT (0-25% cyklu)
    if (phase < 0.25f) {
        z = z_off + z_amp * sin(phase / 0.25f * M_PI);
        x = x_off + x_amp * sin(phase / 0.25f * M_PI / 2.0f);
    }
    // RETURN (25-100% cyklu) 
    else {
        z = z_off;  // noga na ziemi
        float returning = (phase - 0.25f) / 0.75f;
        x = x_off + x_amp * (1.0f - returning);  
    }
}

void trot_gait(float x_amp, float z_amp, float x_off, float z_off, float phase, float& z, float& x) {
    // Faza podnoszenia (0-50% cyklu)
    if (phase < 0.5f) {
        z = z_off + z_amp * sin(phase * 2.0f * M_PI);
        x = x_off + x_amp * sin(phase * M_PI);
    }
    // Faza powrotu (50-100% cyklu)
    else {
        z = z_off;  // noga na ziemi
        x = x_off + x_amp * cos((phase - 0.5f) * M_PI);
    }
}

