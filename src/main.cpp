// The Bluetooth MAC address of the ESP32 must be uploaded 
// to the PS4 controller using the Sixaxis pairing tool.
// A0:DD:6C:85:54:9E 

#include <Arduino.h>
#include <math.h>
#include <PS4Controller.h>
#include "board.h" // OLED display functions
#include "servo.h"
#include "gait.h"

// Pin Definitions
#define S_RXD 18
#define S_TXD 19
#define S_SCL 22
#define S_SDA 21
#define RGB_LED 23
#define NUMPIXELS 10

// Gait control
GaitMode gait = CREEP_FORWARD;

// Button states
bool last_circle = false;
bool last_triangle = false;
bool last_cross = false;
bool last_up = false;
bool last_down = false;
bool last_left = false;
bool last_right = false;

// Gait parameters
float h = 20;                       // Height
float t_cycle = 1.5;                // Cycle time                         

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
    move_servo(1, 45);     // servo 1
    move_servo(2, 90 - h);  // servo 2
    move_servo(3, 135);      // servo 3
    move_servo(4, 90 + h);  // servo 4
    move_servo(5, 135);      // servo 5
    move_servo(6, 90 + h);  // servo 6
    move_servo(7, 45);     // servo 7
    move_servo(8, 90 - h);  // servo 8
    
    running = false;
}

const int MIN_ANGLE = 30;
const int MAX_ANGLE = 140;

void process_PS4_input() {
    // Read and normalize stick values with deadzone
    float lx = (abs(PS4.LStickX()) < DEADZONE * 128) ? 0 : PS4.LStickX() / 128.0;
    float ly = (abs(PS4.LStickY()) < DEADZONE * 128) ? 0 : PS4.LStickY() / 128.0;    
    float rx = (abs(PS4.RStickX()) < DEADZONE * 128) ? 0 : PS4.RStickX() / 128.0;
    float ry = (abs(PS4.RStickY()) < DEADZONE * 128) ? 0 : PS4.RStickY() / 128.0;

    bool leftStickActive = (abs(lx) > DEADZONE || abs(ly) > DEADZONE);
    bool rightStickActive = (abs(rx) > DEADZONE || abs(ry) > DEADZONE);

    // Process left stick - movement
    if (leftStickActive) {
        running = true;
        if (ly > 0.5) { 
            gait = CREEP_FORWARD; 
        } else if (ly < -0.5) { 
            gait = CREEP_BACKWARD; 
        } else if (lx > 0.5) { 
            gait = CREEP_RIGHT; 
        } else if (lx < -0.5) { 
            gait = CREEP_LEFT; 
        }
        execute_gait_step(gait);
    } else {
        // Only stop running if right stick is also inactive
        if (!rightStickActive) {
            running = false;
        }
    }

    // Process right stick - height and tilt adjustment
    if (rightStickActive) {
        int baseAngle2 = 90 - h;  // Front-left
        int baseAngle4 = 90 + h;  // Front-right  
        int baseAngle6 = 90 + h;  // Rear-left
        int baseAngle8 = 90 - h;  // Rear-right
        
        int maxDeviation = 50;
        
        // Calculate tilt offsets - jedna strona w górę, druga w dół
        int frontTilt = -ry * maxDeviation;  // UP: front down (-), DOWN: front up (+)
        int rearTilt = ry * maxDeviation;    // UP: rear up (+), DOWN: rear down (-)
        int leftTilt = -rx * maxDeviation;   // LEFT: left down (-), RIGHT: left up (+)
        int rightTilt = rx * maxDeviation;   // LEFT: right up (+), RIGHT: right down (-)
        
        // Apply combined offsets - przeciwne ruchy dla przeciwległych nóg
        move_servo_smooth(2, constrain(baseAngle2 + frontTilt + leftTilt, MIN_ANGLE, MAX_ANGLE));  // Front-left
        move_servo_smooth(4, constrain(baseAngle4 - frontTilt - rightTilt, MIN_ANGLE, MAX_ANGLE)); // Front-right
        move_servo_smooth(6, constrain(baseAngle6 - rearTilt - leftTilt, MIN_ANGLE, MAX_ANGLE));   // Rear-left  
        move_servo_smooth(8, constrain(baseAngle8 + rearTilt + rightTilt, MIN_ANGLE, MAX_ANGLE));  // Rear-right
        
    } else if (!leftStickActive) {
        return_to_neutral();
    }
}

void processButtons() {
    if (PS4.Up() && !last_up) {h += 5; return_to_neutral();}
    if (PS4.Down() && !last_down) {h -= 5; return_to_neutral();}
    if (h < 0) h = 0;
    if (h > 50) h = 50;

    if (PS4.Left() && !last_left) {t_cycle -= 1;}
    if (PS4.Right() && !last_right) {t_cycle += 1;}
    if (t_cycle < 1.5) t_cycle = 1.5;
    if (t_cycle > 4.5) t_cycle = 4.5;

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