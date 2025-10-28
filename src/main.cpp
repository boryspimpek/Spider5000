// The Bluetooth MAC address of the ESP32 must be uploaded 
// to the PS4 controller using the Sixaxis pairing tool.
// A0:DD:6C:85:54:9E 

// Experimental version

//main.cpp
#include <Arduino.h>
#include <math.h>
#include <PS4Controller.h>
#include "board.h"
#include "servos.h"
#include "gait.h"
#include "variable.h"

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
    } 

    // Process right stick - height and tilt adjustment
    else if (rightStickActive) {
        running = false;  // Stop gait gdy używamy prawej gałki
        gait_phase = 0.0;

        int baseAngle2 = 90 - h;  // Front-left
        int baseAngle4 = 90 + h;  // Front-right  
        int baseAngle6 = 90 + h;  // Rear-left
        int baseAngle8 = 90 - h;  // Rear-right
        
        // Calculate tilt offsets - jedna strona w górę, druga w dół
        int frontTilt = -ry * maxDeviation;  // UP: front down (-), DOWN: front up (+)
        int rearTilt = ry * maxDeviation;    // UP: rear up (+), DOWN: rear down (-)
        int leftTilt = -rx * maxDeviation;   // LEFT: left down (-), RIGHT: left up (+)
        int rightTilt = rx * maxDeviation;   // LEFT: right up (+), RIGHT: right down (-)
        
        // Apply combined offsets - przeciwne ruchy dla przeciwległych nóg
        move_servo_smooth(2, (baseAngle2 + frontTilt + leftTilt));  // Front-left
        move_servo_smooth(4, (baseAngle4 - frontTilt - rightTilt)); // Front-right
        move_servo_smooth(6, (baseAngle6 - rearTilt - leftTilt));   // Rear-left  
        move_servo_smooth(8, (baseAngle8 + rearTilt + rightTilt));  // Rear-right
        
    }  else {
        running = false;
        gait_phase = 0.0;
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
        
        // Tylko gait (automatyczny chód) - gdy lewa gałka aktywna
        if (running) {
            execute_gait(gait);
        }
    } else {
        // Kontroler rozłączony - zatrzymaj wszystko
        if (running) {
            running = false;
            gait_phase = 0.0;
            return_to_neutral();
        }
    }
    
    delay(20);
}