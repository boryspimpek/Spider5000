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

GaitMode gait = CREEP_FORWARD;

// Definicje głównych flag
enum ModeFlag {
    MODE_DEFAULT = 0,
    MODE_TRIANGLE,
    MODE_SQUARE,
    MODE_CROSS,
    MODE_CIRCLE,
    MODE_COUNT
};

// Definicje podtrybów dla trybu TRIANGLE
enum TriangleSubMode {
    SUBMODE_A = 0,
    SUBMODE_B,
    SUBMODE_C,
    SUBMODE_D,
    SUBMODE_COUNT
};

// Globalne zmienne
ModeFlag currentMode = MODE_DEFAULT;
TriangleSubMode triangleSubMode = SUBMODE_A;
bool buttonsActive = true;
unsigned long lastModeChange = 0;
const unsigned long MODE_CHANGE_DELAY = 300; // ms

// Deklaracje funkcji
void processInput();
void processButtons();
void handleMainModeChange(ModeFlag newMode);
void handleTriangleSubModeChange(bool moveRight);

void handleMainModeChange(ModeFlag newMode) {
    unsigned long currentTime = millis();
    
    if (newMode != currentMode && buttonsActive && 
        (currentTime - lastModeChange) > MODE_CHANGE_DELAY) {
        
        currentMode = newMode;
        buttonsActive = false;
        lastModeChange = currentTime;
        
        // Reset podtrybu gdy zmieniamy główny tryb
        if (currentMode != MODE_TRIANGLE) {
            triangleSubMode = SUBMODE_A;
        }
        
        buttonsActive = true;
    }
}

void handleTriangleSubModeChange(bool moveRight) {
    unsigned long currentTime = millis();
    
    if (buttonsActive && (currentTime - lastModeChange) > MODE_CHANGE_DELAY) {
        if (moveRight) {
            // Zmiana w prawo (R1)
            triangleSubMode = (TriangleSubMode)((triangleSubMode + 1) % SUBMODE_COUNT);
        } else {
            // Zmiana w lewo (L1)
            triangleSubMode = (TriangleSubMode)((triangleSubMode - 1 + SUBMODE_COUNT) % SUBMODE_COUNT);
        }
        
        buttonsActive = false;
        lastModeChange = currentTime;
        
        buttonsActive = true;
    }
}

void processTriangleSubModes(int lx, int ly, int rx, int ry,
                            bool up, bool down, bool left, bool right, 
                            bool r1, bool r2, int r2Value, int l2Value) {
    
    switch(triangleSubMode) {
        case SUBMODE_A:
            // Podtryb A - podstawowe funkcje
            if (abs(lx) > 10) {
                Serial.printf("Triangle-A - Left X: %d\n", lx);
            }
            if (up) Serial.println("Triangle-A - UP: Move forward");
            if (down) Serial.println("Triangle-A - DOWN: Move backward");
            break;
            
        case SUBMODE_B:
            // Podtryb B - zaawansowane ruchy
            if (abs(ry) > 10) {
                Serial.printf("Triangle-B - Right Y: %d (Camera)\n", ry);
            }
            if (r2Value > 20) {
                Serial.printf("Triangle-B - R2: %d (Throttle)\n", r2Value);
            }
            break;
            
        case SUBMODE_C:
            // Podtryb C - kombinacje
            if (left && r1) {
                Serial.println("Triangle-C - LEFT+R1: Quick turn left");
            }
            if (right && r1) {
                Serial.println("Triangle-C - RIGHT+R1: Quick turn right");
            }
            if (abs(ly) > 30) {
                Serial.printf("Triangle-C - Speed: %d\n", ly);
            }
            break;
            
        case SUBMODE_D:
            // Podtryb D - precyzyjne sterowanie
            if (abs(lx) > 2) {
                Serial.printf("Triangle-D - Precise X: %d\n", lx);
            }
            if (abs(ly) > 2) {
                Serial.printf("Triangle-D - Precise Y: %d\n", ly);
            }
            if (l2Value > 10) {
                Serial.printf("Triangle-D - Brake: %d\n", l2Value);
            }
            break;
    }
    
    // Wspólne akcje dla wszystkich podtrybów TRIANGLE
    if (PS4.Touchpad()) {
        Serial.println("Triangle - Touchpad: Special action!");
    }
}

void process_PS4_input() {
    if (!PS4.isConnected()) return;
    
    // Odczyt wartości gałek z poprawną deadzone
    int lx_raw = PS4.LStickX();
    int ly_raw = PS4.LStickY();
    int rx_raw = PS4.RStickX();
    int ry_raw = PS4.RStickY();
    
    // Apply deadzone
    int lx = (abs(lx_raw) < DEADZONE * 128) ? 0 : lx_raw;
    int ly = (abs(ly_raw) < DEADZONE * 128) ? 0 : ly_raw;
    int rx = (abs(rx_raw) < DEADZONE * 128) ? 0 : rx_raw;
    int ry = (abs(ry_raw) < DEADZONE * 128) ? 0 : ry_raw;
    
    // Normalizacja
    float lx_norm = lx / 128.0;
    float ly_norm = ly / 128.0;
    float rx_norm = rx / 128.0;
    float ry_norm = ry / 128.0;
    
    bool leftStickActive = (abs(lx) > 0 || abs(ly) > 0);
    bool rightStickActive = (abs(rx) > 0 || abs(ry) > 0);

    // Odczyt przycisków R1, R2, strzałek
    bool r1 = PS4.R1();
    bool r2 = PS4.R2();
    bool up = PS4.Up();
    bool down = PS4.Down();
    bool left = PS4.Left();
    bool right = PS4.Right();

    // Wartość analogowa dla R2/L2 (0-255)
    int r2Value = PS4.R2Value();
    int l2Value = PS4.L2Value();

    // DEBUG: Wyświetl wartości dla testów
    static unsigned long lastDebug = 0;
    if (millis() - lastDebug > 500) {
        Serial.printf("LX: %d, LY: %d, RX: %d, RY: %d\n", lx, ly, rx, ry);
        Serial.printf("Up: %d, Down: %d, Left: %d, Right: %d\n", up, down, left, right);
        lastDebug = millis();
    }

    // Główne tryby
    switch(currentMode) {
        case MODE_DEFAULT:
            if (leftStickActive) {
                running = true;
                if (ly_norm > 0.5) gait = CREEP_FORWARD;
                else if (ly_norm < -0.5) gait = CREEP_BACKWARD;
                else if (lx_norm > 0.5) gait = CREEP_RIGHT;
                else if (lx_norm < -0.5) gait = CREEP_LEFT;
            }      
            else if (rightStickActive) {
                running = true;
                if (ry_norm > 0.5) gait = TROT_FORWARD;
                else if (ry_norm < -0.5) gait = TROT_BACKWARD;
                else if (rx_norm > 0.5) gait = TROT_RIGHT;
                else if (rx_norm < -0.5) gait = TROT_LEFT;
            }  
            else if (up) {
                h += 5; 
                return_to_neutral(); 
                if (h > 50) h = 50;
                Serial.printf("Height increased to: %d\n", h);
            }
            else if (down) {
                h -= 5; 
                return_to_neutral(); 
                if (h < 0) h = 0;
                Serial.printf("Height decreased to: %d\n", h);
            }
            else if (left) {
                t_cycle -= 1; 
                if (t_cycle < 1.5) t_cycle = 1.5;
                Serial.printf("Cycle time: %.1f\n", t_cycle);
            }
            else if (right) {
                t_cycle += 1; 
                if (t_cycle > 4.5) t_cycle = 4.5;
                Serial.printf("Cycle time: %.1f\n", t_cycle);
            }
            else {
                running = false; 
                gait_phase = 0.0; 
                return_to_neutral();
            }     
            break;

            
        case MODE_TRIANGLE:
            // W trybie TRIANGLE używamy podtrybów
            processTriangleSubModes(lx, ly, rx, ry, 
                                   up, down, left, right, r1, r2, r2Value, l2Value);
            break;
            
        case MODE_SQUARE:
            if (rightStickActive) {
                running = false;  // Stop gait gdy używamy prawej gałki
                gait_phase = 0.0;

                // Calculate tilt offsets - jedna strona w górę, druga w dół
                int frontTilt = -ry_norm * maxDeviation;  // UP: front down (-), DOWN: front up (+)
                int rearTilt = ry_norm * maxDeviation;    // UP: rear up (+), DOWN: rear down (-)
                int leftTilt = -rx_norm * maxDeviation;   // LEFT: left down (-), RIGHT: left up (+)
                int rightTilt = rx_norm * maxDeviation;   // LEFT: right up (+), RIGHT: right down (-)
                
                // Apply combined offsets - przeciwne ruchy dla przeciwległych nóg
                move_servo_smooth(2, (90 - h + frontTilt + leftTilt));  // Front-left
                move_servo_smooth(4, (90 + h - frontTilt - rightTilt)); // Front-right
                move_servo_smooth(6, (90 + h - rearTilt - leftTilt));   // Rear-left  
                move_servo_smooth(8, ( 90 - h + rearTilt + rightTilt));  // Rear-right    
            } else {
                running = false;
                gait_phase = 0.0;
                return_to_neutral();
            }        
                break;
            
        case MODE_CROSS:
            break;
            
        case MODE_CIRCLE:
            break;
    }
    if (PS4.L1() && PS4.R1()) {
        handleMainModeChange(MODE_DEFAULT);
        Serial.println("Reset to DEFAULT mode");
    }
}

void processButtons() {
    // Sprawdzanie przycisków do zmiany głównych flag
    if (PS4.Triangle()) {
        handleMainModeChange(MODE_TRIANGLE);
    } else if (PS4.Square()) {
        handleMainModeChange(MODE_SQUARE);
    } else if (PS4.Cross()) {
        handleMainModeChange(MODE_CROSS);
    } else if (PS4.Circle()) {
        handleMainModeChange(MODE_CIRCLE);
    }
    
    // Obsługa zmiany podtrybów w trybie TRIANGLE
    if (currentMode == MODE_TRIANGLE) {
        if (PS4.R1() && !PS4.L1()) { // Tylko R1 wciśnięte
            handleTriangleSubModeChange(true); // Zmiana w prawo
        } else if (PS4.L1() && !PS4.R1()) { // Tylko L1 wciśnięte
            handleTriangleSubModeChange(false); // Zmiana w lewo
        }
    }
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
    static unsigned long lastStatus = 0;
    
    if (millis() - lastStatus > 2000) {
        if (PS4.isConnected()) {
            Serial.println("PS4 Connected - Waiting for input...");
        } else {
            Serial.println("PS4 Disconnected");
        }
        lastStatus = millis();
    }
    
    if (PS4.isConnected()) {
        process_PS4_input();
        processButtons();
        
        if (running) {
            execute_gait(gait);
        }
    } else {
        if (running) {
            running = false;
            gait_phase = 0.0;
            return_to_neutral();
        }
    }
    
    delay(20);
}