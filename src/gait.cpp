#include "gait.h"

// Definicje zmiennych globalnych
const unsigned long GAIT_DT = 50;
bool running = false;
unsigned long last_gait_time = 0;
float gait_phase = 0.0;

// Definicje stałych konfiguracji
const GaitParams GAIT_CONFIGS[] = {
    // CREEP_FORWARD
    {
        {-x_amp, x_amp, -x_amp, x_amp},
        {z_amp, -z_amp, -z_amp, z_amp},
        {90 - OFFSET_FRONT, 90 + OFFSET_FRONT, 90 + OFFSET_BACK, 90 - OFFSET_BACK},
        {0.00, 0.50, 0.25, 0.75}
    },
    // CREEP_BACKWARD
    {
        {x_amp, -x_amp, +x_amp, -x_amp},
        {z_amp, -z_amp, -z_amp, z_amp},
        {90 - OFFSET_BACK, 90 + OFFSET_BACK, 90 + OFFSET_FRONT, 90 - OFFSET_FRONT},
        {0.25, 0.75, 0.00, 0.50}
    },
    // CREEP_LEFT
    {
        {-x_amp, -x_amp, -x_amp, -x_amp},
        {z_amp, -z_amp, -z_amp, z_amp},
        {45 + x_amp / 2, 135 + x_amp / 2, 135 + x_amp / 2, 45 + x_amp / 2},
        {0.00, 0.50, 0.25, 0.75}
    },
    // CREEP_RIGHT
    {
        {x_amp, x_amp, x_amp, x_amp},
        {z_amp, -z_amp, -z_amp, z_amp},
        {45 - x_amp / 2, 135 - x_amp / 2, 135 - x_amp / 2, 45 - x_amp / 2},
        {0.00, 0.50, 0.25, 0.75}
    },
    // TROT_FORWARD
    {
        {-x_amp, x_amp, -x_amp, x_amp},
        {z_amp, -z_amp, -z_amp, z_amp},
        {45+x_amp/2, 135-x_amp/2, 135+x_amp/2, 45-x_amp/2},
        {0.50, 0.00, 0.00, 0.50}
    },
    // TROT_BACKWARD
    {
        {x_amp, -x_amp, x_amp, -x_amp},
        {z_amp, -z_amp, -z_amp, z_amp},
        {45-x_amp/2, 135+x_amp/2, 135-x_amp/2, 45+x_amp/2},
        {0.50, 0.00, 0.00, 0.50}
    },
    // TROT_LEFT
    {
        {x_amp, x_amp, x_amp, x_amp},
        {z_amp, -z_amp, -z_amp, z_amp},
        {45 + x_amp / 2, 135 + x_amp / 2, 135 + x_amp / 2, 45 + x_amp / 2},
        {0.50, 0.00, 0.00, 0.50}
    },
    // TROT_RIGHT
    {
        {-x_amp, -x_amp, -x_amp, -x_amp},
        {z_amp, -z_amp, -z_amp, z_amp},
        {45 - x_amp / 2, 135 - x_amp / 2, 135 - x_amp / 2, 45 - x_amp / 2},
        {0.50, 0.00, 0.00, 0.50}
    }
};

const ServoMapping SERVO_MAPPING[8] = {
    {1, "lf", "x"}, {2, "lf", "z"},
    {3, "rf", "x"}, {4, "rf", "z"},
    {5, "lr", "x"}, {6, "lr", "z"},
    {7, "rr", "x"}, {8, "rr", "z"}
};

// Implementacja funkcji
void creep_gait(float x_amp, float z_amp, float x_off, float z_off, float phase, float& z, float& x) {
    if (phase < 0.25f) {
        z = z_off + z_amp * sin(phase / 0.25f * M_PI);
        x = x_off + x_amp * sin(phase / 0.25f * M_PI / 2.0f);
    } else {
        z = z_off;
        float returning = (phase - 0.25f) / 0.75f;
        x = x_off + x_amp * (1.0f - returning);
    }
}

void trot_gait(float x_amp, float z_amp, float x_off, float z_off, float phase, float& z, float& x) {
    if (phase < 0.5f) {
        z = z_off + z_amp * sin(phase * 2.0f * M_PI);
        x = x_off + x_amp * sin(phase * M_PI);
    } else {
        z = z_off;
        x = x_off + x_amp * cos((phase - 0.5f) * M_PI);
    }
}

void calculate_gait_angles(GaitMode mode, float phase, float angles[4][2]) {
    const GaitParams& params = GAIT_CONFIGS[mode];
    float dynamic_z_offsets[4] = {90 - h, 90 + h, 90 + h, 90 - h};

    for (int i = 0; i < 4; i++) {
        float current_phase = fmod(phase + params.phase_offsets[i], 1.0f);
        creep_gait(params.x_amps[i], params.z_amps[i], params.x_offsets[i],
                   dynamic_z_offsets[i], current_phase, angles[i][1], angles[i][0]);
    }
}

void execute_gait(GaitMode mode) {
    unsigned long current_time = millis();
    if (current_time - last_gait_time < GAIT_DT) return;

    last_gait_time = current_time;
    gait_phase = fmod(gait_phase + (GAIT_DT / 1000.0) / t_cycle, 1.0);

    float angles[4][2];
    calculate_gait_angles(mode, gait_phase, angles);

    for (int i = 0; i < 8; i++) {
        const ServoMapping& mapping = SERVO_MAPPING[i];
        int leg_index = -1;

        if (strcmp(mapping.leg, "lf") == 0) leg_index = 0;
        else if (strcmp(mapping.leg, "rf") == 0) leg_index = 1;
        else if (strcmp(mapping.leg, "lr") == 0) leg_index = 2;
        else if (strcmp(mapping.leg, "rr") == 0) leg_index = 3;

        if (leg_index != -1) {
            float angle = (strcmp(mapping.axis, "x") == 0)
                            ? angles[leg_index][0]
                            : angles[leg_index][1];
            move_servo(mapping.servo_id, (int)angle);
        }
    }
}