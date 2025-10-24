// The Bluetooth MAC address of the ESP32 must be uploaded 
// to the PS4 controller using the Sixaxis pairing tool.
// A0:DD:6C:85:54:9E 

#include <Arduino.h>
#include <math.h>
#include <SCServo.h>
#include <PS4Controller.h>
#include "board.h" // OLED display functions

// ============================================================================
// Pin Definitions
// ============================================================================
#define S_RXD 18
#define S_TXD 19
#define S_SCL 22
#define S_SDA 21
#define RGB_LED 23
#define NUMPIXELS 10

// ============================================================================
// Constants
// ============================================================================

// Servo settings
const int sts_id[8] = {1, 2, 3, 4, 5, 6, 7, 8};
const int acc = 250;
const int speed = 2400;

// Gait parameters
float h = 20;                    // height
const int x_amp = 30;               // x amplitude
const int z_amp = 15;               // z amplitude
const int OFFSET_FRONT = 5;         // front leg offset
const int OFFSET_BACK = 45;         // back legs offset

const float DEADZONE = 0.1;
const unsigned long GAIT_DT = 50;   // 50ms = 0.05s

// Servo limits and calibration
const int SERVO_LIMITS[8][2] = {
    {75, 150},   // servo 1
    {30, 140},   // servo 2
    {30, 105},   // servo 3
    {40, 150},   // servo 4
    {30, 105},   // servo 5
    {40, 150},   // servo 6
    {85, 150},   // servo 7
    {30, 140}    // servo 8
};

const int SERVO_TRIMS[8] = {100, 60, 140, 55, 0, 10, 0, 40};
const int NEUTRAL_ANGLES[8] = {100, 90, 80, 90, 60, 90, 120, 90};

// ============================================================================
// Type Definitions
// ============================================================================

enum GaitMode {
    CREEP_FORWARD,
    CREEP_BACKWARD,
    CREEP_RIGHT,
    CREEP_LEFT,
    CREEP_TROT_FORWARD,
    CREEP_TROT_BACKWARD,
    CREEP_TROT_RIGHT,
    CREEP_TROT_LEFT
};

struct ServoMapping {
    int servo_id;
    const char* leg;
    const char* axis;
};

struct GaitParams {
    float x_amps[4];        // LF, RF, LR, RR
    float z_amps[4];
    float x_offsets[4];
    float z_offsets[4];
    float phase_offsets[4];
};

// ============================================================================
// Configuration Arrays
// ============================================================================

const ServoMapping SERVO_MAPPING[8] = {
    {1, "lf", "x"}, {2, "lf", "z"},  // Servo 1: LF X, Servo 2: LF Z
    {3, "rf", "x"}, {4, "rf", "z"},  // Servo 3: RF X, Servo 4: RF Z
    {5, "lr", "x"}, {6, "lr", "z"},  // Servo 5: LR X, Servo 6: LR Z
    {7, "rr", "x"}, {8, "rr", "z"}   // Servo 7: RR X, Servo 8: RR Z
};

const GaitParams GAIT_CONFIGS[] = {
    // CREEP_FORWARD
    {
        {x_amp, -x_amp, x_amp, -x_amp},
        {z_amp, -z_amp, -z_amp, z_amp},
        {90 - OFFSET_FRONT, 90 + OFFSET_FRONT, 90 - OFFSET_BACK, 90 + OFFSET_BACK},
        {90-h, 90+h, 90+h, 90-h},
        {0.00, 0.50, 0.25, 0.75}
    },
    // CREEP_BACKWARD
    {
        {-x_amp, x_amp, -x_amp, x_amp},
        {z_amp, -z_amp, -z_amp, z_amp},
        {90 + OFFSET_BACK, 90 - OFFSET_BACK, 90 + OFFSET_FRONT, 90 - OFFSET_FRONT},
        {90-h, 90+h, 90+h, 90-h},
        {0.25, 0.75, 0.00, 0.50}
    },
    // CREEP_RIGHT
    {
        {x_amp, x_amp, x_amp, x_amp},
        {z_amp, -z_amp, -z_amp, z_amp},
        {135-x_amp/2, 45-x_amp/2, 45-x_amp/2, 135-x_amp/2},
        {90-h, 90+h, 90+h, 90-h},
        {0.00, 0.50, 0.25, 0.75}
    },
    // CREEP_LEFT
    {
        {-x_amp, -x_amp, -x_amp, -x_amp},
        {z_amp, -z_amp, -z_amp, z_amp},
        {135+x_amp/2, 45+x_amp/2, 45+x_amp/2, 135+x_amp/2},
        {90-h, 90+h, 90+h, 90-h},
        {0.00, 0.50, 0.25, 0.75}
    },
    // CREEP_TROT_FORWARD
    {
        {x_amp, -x_amp, x_amp, -x_amp},
        {z_amp, -z_amp, -z_amp, z_amp},
        {135-x_amp/2, 45+x_amp/2, 45-x_amp/2, 135+x_amp/2},
        {90-h, 90+h, 90+h, 90-h},
        {0.50, 0.00, 0.00, 0.50}
    },
    // CREEP_TROT_BACKWARD
    {
        {-x_amp, x_amp, -x_amp, x_amp},
        {z_amp, -z_amp, -z_amp, z_amp},
        {135+x_amp/2, 45-x_amp/2, 45-x_amp/2, 135-x_amp/2},
        {90-h, 90+h, 90+h, 90-h},
        {0.50, 0.00, 0.00, 0.50}
    },
    // CREEP_TROT_RIGHT
    {
        {-x_amp, -x_amp, -x_amp, -x_amp},
        {z_amp, -z_amp, -z_amp, z_amp},
        {135+x_amp/2, 45+x_amp/2, 45+x_amp/2, 135+x_amp/2},
        {90-h, 90+h, 90+h, 90-h},
        {0.50, 0.00, 0.00, 0.50}
    },
    // CREEP_TROT_LEFT
    {
        {x_amp, x_amp, x_amp, x_amp},
        {z_amp, -z_amp, -z_amp, z_amp},
        {135-x_amp/2, 45-x_amp/2, 45-x_amp/2, 135-x_amp/2},
        {90-h, 90+h, 90+h, 90-h},
        {0.50, 0.00, 0.00, 0.50}
    }
};

// ============================================================================
// Global Variables
// ============================================================================

// Servo control objects
SMS_STS st;

// Gait control
GaitMode current_gait_mode = CREEP_FORWARD;
bool is_gait_running = false;
unsigned long last_gait_time = 0;
float gait_phase = 0.0;

// Button states
bool last_circle = false;
bool last_up = false;
bool last_down = false;

int angle_deg_to_servo(float deg) {
    float rad = radians(deg);  // ° → rad
    int center = 2048;
    float scale = 2048.0 / PI;
    return 4095 - (int)round(rad * scale + center);
}

int check_angle_limit(int id, int angle_deg) {
    if (id < 1 || id > 8) return angle_deg;
    
    int min_angle = SERVO_LIMITS[id-1][0];
    int max_angle = SERVO_LIMITS[id-1][1];
    
    if (angle_deg < min_angle) {
        Serial.printf("Servo %d: kąt %d° poniżej minimum (%d°) — ograniczono.\n", id, angle_deg, min_angle);
        angle_deg = min_angle;
    } else if (angle_deg > max_angle) {
        Serial.printf("Servo %d: kąt %d° powyżej maksimum (%d°) — ograniczono.\n", id, angle_deg, max_angle);
        angle_deg = max_angle;
    }
    return angle_deg;
}

void move_servo(int id, int angle_deg) {
    int safe_angle = check_angle_limit(id, angle_deg);
    int pos = angle_deg_to_servo(safe_angle);
    int trimmed_pos = pos + SERVO_TRIMS[id-1];
    st.WritePosEx(id, trimmed_pos, speed, acc);
}

void creep_gait(float x_amp, float z_amp, float x_off, float z_off, float phase, float& z, float& x) {
    // LIFT (0-25% cyklu)
    if (phase < 0.25f) {
        z = z_off + z_amp * sin(phase / 0.25f * M_PI);
        x = x_off + x_amp * sin(phase / 0.25f * M_PI / 2.0f);
    }
    // RETURN (25-100% cyklu) 
    else {
        z = z_off;  // noga na ziemi
        float normalized_return_phase = (phase - 0.25f) / 0.75f;
        x = x_off + x_amp * (1.0f - normalized_return_phase);  // liniowy powrót
    }
}

void calculate_gait_angles(GaitMode mode, float phase, float angles[4][2]) {
    const GaitParams& params = GAIT_CONFIGS[mode];
    const char* legs[4] = {"lf", "rf", "lr", "rr"};
    
    for (int i = 0; i < 4; i++) {
        float current_phase = fmod(phase + params.phase_offsets[i], 1.0f);
        creep_gait(params.x_amps[i], params.z_amps[i], params.x_offsets[i], 
                  params.z_offsets[i], current_phase, angles[i][1], angles[i][0]);
    }
}

void execute_gait_step(GaitMode mode) {
    unsigned long current_time = millis();
    if (current_time - last_gait_time < GAIT_DT) return;
    
    last_gait_time = current_time;
    
    const float t_cycle = 2.0; // 2 second cycle
    gait_phase = fmod(gait_phase + (GAIT_DT / 1000.0) / t_cycle, 1.0);
    
    float angles[4][2]; // [leg_index][0=x, 1=z]
    calculate_gait_angles(mode, gait_phase, angles);
    
    const int leg_to_index[4] = {0, 1, 2, 3}; // lf, rf, lr, rr
    
    for (int i = 0; i < 8; i++) {
        const ServoMapping& mapping = SERVO_MAPPING[i];
        int leg_index = -1;
        
        if (strcmp(mapping.leg, "lf") == 0) leg_index = 0;
        else if (strcmp(mapping.leg, "rf") == 0) leg_index = 1;
        else if (strcmp(mapping.leg, "lr") == 0) leg_index = 2;
        else if (strcmp(mapping.leg, "rr") == 0) leg_index = 3;
        
        if (leg_index != -1) {
            float angle = 0;
            if (strcmp(mapping.axis, "x") == 0) {
                angle = angles[leg_index][0];
            } else if (strcmp(mapping.axis, "z") == 0) {
                angle = angles[leg_index][1];
            }
            move_servo(mapping.servo_id, (int)angle);
        }
    }
    
    // Serial.printf("Gait phase: %.2f\n", gait_phase);
}

void return_to_neutral() {
    for (int i = 0; i < 8; i++) {
        move_servo(i + 1, NEUTRAL_ANGLES[i]);
    }
    is_gait_running = false;
    // Serial.println("Returned to neutral position");
}

void process_PS4_input() {
    float lx = PS4.LStickX() / 128.0;
    float ly = PS4.LStickY() / 128.0;
    
    lx = (abs(lx) < DEADZONE) ? 0 : lx;
    ly = (abs(ly) < DEADZONE) ? 0 : ly;
    
    if (ly > 0.5) {
        current_gait_mode = CREEP_FORWARD;
        is_gait_running = true;
    } else if (ly < -0.5) {
        current_gait_mode = CREEP_BACKWARD;
        is_gait_running = true;
    } else if (lx > 0.5) {
        current_gait_mode = CREEP_RIGHT;
        is_gait_running = true;
    } else if (lx < -0.5) {
        current_gait_mode = CREEP_LEFT;
        is_gait_running = true;
    } else {
        is_gait_running = false;
        return_to_neutral();
    }
    
    if (is_gait_running) {
        execute_gait_step(current_gait_mode);
    }
}

void processButtons() {
    if (PS4.Circle() && !last_circle) {
        Serial.println("Circle pressed");
    }
    
    if (PS4.Up() && !last_up) { 
        h += 5;
        Serial.printf("Height increased to: %d\n", h);
    }
    
    if (PS4.Down() && !last_down) { 
        h -= 5;
        Serial.printf("Height decreased to: %d\n", h);
    }

    if (h < 0) h = 0;
    if (h > 0) h = 50;

    last_circle = PS4.Circle();
    last_up = PS4.Up();
    last_down = PS4.Down();
}

void onConnect() {
    ConnectedText();
    Serial.println("PS4 controller connected");
}

void onDisconnect() {
    Serial.println("PS4 controller disconnected");
    return_to_neutral();
}

void setup() {
    Serial.begin(115200);
    Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
    st.pSerial = &Serial1;
    
    PS4.attachOnConnect(onConnect);
    PS4.attachOnDisconnect(onDisconnect);
    PS4.begin(); 
    delay(1000);
    
    btMac();
    
    InitScreen();
    scanServos();
    displayResultsScreen();

    return_to_neutral();

    Serial.println("Inicjalizacja zakończona");
}

void loop() {
    if (PS4.isConnected()) {
        process_PS4_input();
        processButtons();
    } else {
        // Stop gait if controller disconnects
        if (is_gait_running) {
            is_gait_running = false;
            return_to_neutral();
        }
    }
    
    delay(20);
}