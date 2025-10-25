// The Bluetooth MAC address of the ESP32 must be uploaded 
// to the PS4 controller using the Sixaxis pairing tool.
// A0:DD:6C:85:54:9E 

#include <Arduino.h>
#include <math.h>
#include <PS4Controller.h>
#include "board.h" // OLED display functions
#include "servo.h"
#include "gait.h"

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
// Global Variables
// ============================================================================

// Gait control
GaitMode gait = CREEP_FORWARD;
bool running = false;
unsigned long last_gait_time = 0;
float gait_phase = 0.0;

// Button states
bool last_circle = false;
bool last_triangle = false;
bool last_cross = false;
bool last_up = false;
bool last_down = false;
bool last_left = false;
bool last_right = false;
float h = 20;
float t_cycle = 1.5;                          
const unsigned long GAIT_DT = 50;           // 50ms = 0.05s

void calculate_gait_angles(GaitMode mode, float phase, float angles[4][2]) {
    const GaitParams& params = GAIT_CONFIGS[mode];
    const char* legs[4] = {"lf", "rf", "lr", "rr"};
    
    // Dynamic z_offsets based on current h value
    float dynamic_z_offsets[4] = {90-h, 90+h, 90+h, 90-h};
    
    for (int i = 0; i < 4; i++) {
        float current_phase = fmod(phase + params.phase_offsets[i], 1.0f);
        creep_gait(params.x_amps[i], params.z_amps[i], params.x_offsets[i], 
                dynamic_z_offsets[i], current_phase, angles[i][1], angles[i][0]);
    }
}

void execute_gait_step(GaitMode mode) {
    unsigned long current_time = millis();
    if (current_time - last_gait_time < GAIT_DT) return;
    last_gait_time = current_time;
    gait_phase = fmod(gait_phase + (GAIT_DT / 1000.0) / t_cycle, 1.0);
    float angles[4][2]; // [leg_index][0=x, 1=z]

    calculate_gait_angles(mode, gait_phase, angles);
    // const int leg_to_index[4] = {0, 1, 2, 3}; // lf, rf, lr, rr // do usunięcia ta linijka
    
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
}

void return_to_neutral() {
    move_servo(1, 100);     // servo 1
    move_servo(2, 90 - h);  // servo 2
    move_servo(3, 80);      // servo 3
    move_servo(4, 90 + h);  // servo 4
    move_servo(5, 60);      // servo 5
    move_servo(6, 90 + h);  // servo 6
    move_servo(7, 120);     // servo 7
    move_servo(8, 90 - h);  // servo 8
    
    running = false;
}

void process_PS4_input() {
    float lx = (abs(PS4.LStickX()) < DEADZONE * 128) ? 0 : PS4.LStickX() / 128.0;
    float ly = (abs(PS4.LStickY()) < DEADZONE * 128) ? 0 : PS4.LStickY() / 128.0;    

    if (ly > 0.5) {gait = CREEP_FORWARD; running = true; }
    else if (ly < -0.5) {gait = CREEP_BACKWARD; running = true;}
    else if (lx > 0.5) {gait = CREEP_RIGHT; running = true;}
    else if (lx < -0.5) {gait = CREEP_LEFT; running = true;}
    else {running = false; return_to_neutral();}
    
    if (running) {execute_gait_step(gait);}
}

void processButtons() {
    if (PS4.Up() && !last_up) {h += 5;}
    if (PS4.Down() && !last_down) {h -= 5;}
    if (h < 0) h = 0;
    if (h > 50) h = 50;

    if (PS4.Left() && !last_left) {t_cycle -= 1;}
    if (PS4.Right() && !last_right) {t_cycle += 1;}
    if (t_cycle < 1.5) t_cycle = 1.5;
    if (t_cycle > 4.5) t_cycle = 4.5;

    last_cross = PS4.Cross();
    last_triangle = PS4.Triangle();
    last_up = PS4.Up();
    last_down = PS4.Down();
    last_left = PS4.Left();
    last_right = PS4.Right();
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
        if (running) {
            running = false;
            return_to_neutral();
        }
    }
    
    delay(20);
}