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
#include "tricky.h"


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
    SUBMODE_TA = 0,
    SUBMODE_TB,
    // SUBMODE_TC,
    // SUBMODE_TD,
    SUBMODE_COUNT_T
};

// Globalne zmienne
ModeFlag currentMode = MODE_DEFAULT;
TriangleSubMode triangleSubMode = SUBMODE_TA;

GaitMode currentGait = TROT_FORWARD;

bool buttonsActive = true;
unsigned long lastModeChange = 0;
const unsigned long MODE_CHANGE_DELAY = 300; 

int activeleg = 1;

struct ControllerData {
    // Surowe wartości gałek
    int lx_raw, ly_raw, rx_raw, ry_raw;
    
    // Wartości po deadzone
    int lx, ly, rx, ry;
    
    // Znormalizowane wartości
    float lx_norm, ly_norm, rx_norm, ry_norm;
    
    // Przyciski
    bool l1, l2, r1, r2, up, down, left, right;
    
    // Wartości analogowe
    int r2Value, l2Value;
    
    // Flagi aktywności
    bool leftStickActive, rightStickActive;
};    

ControllerData read_PS4_input() {
    ControllerData data;
    
    // Odczyt wartości gałek z poprawną deadzone
    data.lx_raw = PS4.LStickX();
    data.ly_raw = PS4.LStickY();
    data.rx_raw = PS4.RStickX();
    data.ry_raw = PS4.RStickY();
    
    // Apply deadzone
    data.lx = (abs(data.lx_raw) < DEADZONE * 128) ? 0 : data.lx_raw;
    data.ly = (abs(data.ly_raw) < DEADZONE * 128) ? 0 : data.ly_raw;
    data.rx = (abs(data.rx_raw) < DEADZONE * 128) ? 0 : data.rx_raw;
    data.ry = (abs(data.ry_raw) < DEADZONE * 128) ? 0 : data.ry_raw;
    
    // Normalizacja
    data.lx_norm = data.lx / 128.0;
    data.ly_norm = data.ly / 128.0;
    data.rx_norm = data.rx / 128.0;
    data.ry_norm = data.ry / 128.0;
    
    // Odczyt przycisków R1, R2, strzałek
    data.l1 = PS4.L1();
    data.l2 = PS4.L2();
    data.r1 = PS4.R1();
    data.r2 = PS4.R2();
    data.up = PS4.Up();
    data.down = PS4.Down();
    data.left = PS4.Left();
    data.right = PS4.Right();
    
    // Wartość analogowa dla R2/L2
    data.r2Value = PS4.R2Value();
    data.l2Value = PS4.L2Value();
    
    // Flagi aktywności gałek
    data.leftStickActive = (abs(data.lx) > 0 || abs(data.ly) > 0);
    data.rightStickActive = (abs(data.rx) > 0 || abs(data.ry) > 0);
    
    return data;
}

void handleMainModeChange(ModeFlag newMode) {
    unsigned long currentTime = millis();
    
    if (newMode != currentMode && buttonsActive && 
        (currentTime - lastModeChange) > MODE_CHANGE_DELAY) {
        
        currentMode = newMode;
        buttonsActive = false;
        lastModeChange = currentTime;
        
        // Reset podtrybu gdy zmieniamy główny tryb
        if (currentMode != MODE_TRIANGLE) {
            triangleSubMode = SUBMODE_TA;
        }        
        buttonsActive = true;
    }
}

void processSquareMode(ControllerData data) {
    running = false;
    gait_phase = 0.0;

    // PRAWA GAŁKA - sterowanie aktywną nogą
    if (data.rightStickActive) {
        int coxa_servo = activeleg;
        int femur_servo = activeleg + 1;
        
        int coxa_move = 0;
        int femur_move = 0;
        
        switch(activeleg) {
            case 1: // Przednia lewa
                coxa_move = -data.rx_norm * maxDeviation;
                femur_move = data.ry_norm * maxDeviation;
                move_servo_smooth(coxa_servo, 45 + coxa_move);
                move_servo_smooth(femur_servo, 90 - h + femur_move);
                break;
            
            case 3: // Przednia prawa
                coxa_move = -data.rx_norm * maxDeviation;
                femur_move = -data.ry_norm * maxDeviation;
                move_servo_smooth(coxa_servo, 135 + coxa_move);
                move_servo_smooth(femur_servo, 90 + h + femur_move);
                break;
            
            case 5: // Tylna lewa
                coxa_move = data.rx_norm * maxDeviation;
                femur_move = -data.ry_norm * maxDeviation;
                move_servo_smooth(coxa_servo, 135 + coxa_move);
                move_servo_smooth(femur_servo, 90 + h + femur_move);
                break;
            
            case 7: // Tylna prawa
                coxa_move = data.rx_norm * maxDeviation;
                femur_move = data.ry_norm * maxDeviation;
                move_servo_smooth(coxa_servo, 45 + coxa_move);
                move_servo_smooth(femur_servo, 90 - h + femur_move);
                break;
        }
    }

    // LEWA GAŁKA - sterowanie pozostałymi 3 nogami (jak MODE_DEFAULT z R1)
    if (data.leftStickActive) {
        int front = data.ly_norm * maxDeviation;
        int rear = -data.ly_norm * maxDeviation;
        int left = data.lx_norm * maxDeviation;
        int right = -data.lx_norm * maxDeviation;

        // Steruj serwami coxa pozostałych nóg
        if (activeleg != 1) {
            move_servo_smooth(1, 45 + front + left);
        }
        if (activeleg != 3) {
            move_servo_smooth(3, 135 - front - right);
        }
        if (activeleg != 5) {
            move_servo_smooth(5, 135 - rear - left);
        }
        if (activeleg != 7) {
            move_servo_smooth(7, 45 + rear + right);
        }
    }

    // Jeśli żadna gałka nie jest aktywna - powrót do neutralnej
    if (!data.leftStickActive && !data.rightStickActive) {
        return_to_neutral();
    }
}

void process_PS4_input() {
    if (!PS4.isConnected()) return;

    if (PS4.Share()) {
        ShowVoltage();
    }

    ControllerData data = read_PS4_input();

    switch (currentMode) {
        case MODE_DEFAULT: 
            // running = false;  
            // gait_phase = 0.0;

            if (data.r1 && data.rightStickActive) {
                running = false;
                int front = data.ry_norm * maxDeviation;
                int rear  = -data.ry_norm * maxDeviation;
                int left  = -data.rx_norm * maxDeviation;
                int right = data.rx_norm * maxDeviation;

                move_servo_smooth(2, 90 - h + front + left);
                move_servo_smooth(4, 90 + h - front - right);
                move_servo_smooth(6, 90 + h - rear - left);
                move_servo_smooth(8, 90 - h + rear + right);
            }
            else if (data.r1 && data.leftStickActive) {
                running = false;
                int front = data.ly_norm * maxDeviation;
                int rear  = -data.ly_norm * maxDeviation;
                int left  = data.lx_norm * maxDeviation;
                int right = -data.lx_norm * maxDeviation;

                move_servo_smooth(1, 45 + front + left);
                move_servo_smooth(3, 135 - front - right);
                move_servo_smooth(5, 135 - rear - left);
                move_servo_smooth(7, 45 + rear + right);
            }
            else if (data.l1 && data.leftStickActive) {
                running = true;

                if (data.ly_norm > 0.5) currentGait = CREEP_FORWARD;
                else if (data.ly_norm < -0.5) currentGait = CREEP_BACKWARD;
                else if (data.lx_norm > 0.5) currentGait = CREEP_RIGHT;
                else if (data.lx_norm < -0.5) currentGait = CREEP_LEFT;
            }
            else if (data.leftStickActive) {
                running = true;

                if (data.ly_norm > 0.5) currentGait = TROT_FORWARD;
                else if (data.ly_norm < -0.5) currentGait = TROT_BACKWARD;
                else if (data.lx_norm > 0.5) currentGait = TROT_RIGHT;
                else if (data.lx_norm < -0.5) currentGait = TROT_LEFT;
            }
            else if (data.rightStickActive) {
                running = true;

                if (data.rx_norm > 0.5) currentGait = TROT_MOVE_RIGHT;
                else if (data.rx_norm < -0.5) currentGait = TROT_MOVE_LEFT;
            }
            else if (data.up) {
                h += 2;
                if (h > 50) h = 50;
                return_to_neutral();
                Serial.print(h);
            }
            else if (data.down) {
                h -= 2;
                if (h < -50) h = -50;
                return_to_neutral();
                Serial.print(h);
            }
            else {
                running = false;
                gait_phase = 0.0;
                return_to_neutral();
            }
            break;
               
        case MODE_TRIANGLE:
            if (data.l1) {
                if (data.left)      frontSteps();
                else if (data.right) playDead();
                else if (data.up)    hello();
                else if (data.down)  dive();
            }
            else if (data.r1) {
                if (data.left)      twoLegUp();
                else if (data.right) twoLegMove();
                else if (data.up)    bounce();
                else if (data.down)  sayNo();
            }
            else {
                if (data.left)      left();
                else if (data.right) right();
                else if (data.up)    front();
                else if (data.down)  back();
            }
            break;

        case MODE_SQUARE:
            processSquareMode(data);
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
    if (PS4.Triangle()) {
        handleMainModeChange(MODE_TRIANGLE);
        Serial.printf("Triangle mode");
    } else if (PS4.Square()) {
        handleMainModeChange(MODE_SQUARE);
        Serial.printf("Square mode");
    } else if (PS4.Cross()) {
        handleMainModeChange(MODE_CROSS);
        Serial.printf("Cross mode");
    } else if (PS4.Circle()) {
        handleMainModeChange(MODE_CIRCLE);
        Serial.printf("Circle mode");
    }
    
    if (currentMode == MODE_TRIANGLE) {
        static bool r1WasPressed = false;
        if (PS4.R1() && !r1WasPressed) {
            triangleSubMode = (TriangleSubMode)((triangleSubMode + 1) % SUBMODE_COUNT_T);
            Serial.printf("Triangle submode: %d\n", triangleSubMode);
            r1WasPressed = true;
        } else if (!PS4.R1()) {
            r1WasPressed = false;
        }
    }

    if (currentMode == MODE_SQUARE) {
        static bool r1WasPressed = false;
        if (PS4.R1() && !r1WasPressed) {
            activeleg += 2;
            if (activeleg > 7) activeleg = 1;
            Serial.printf("Active leg: %d\n", activeleg);
            r1WasPressed = true;
        } else if (!PS4.R1()) {
            r1WasPressed = false;
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
    if (PS4.isConnected()) {
        process_PS4_input();
        processButtons();
        
        if (running) {
            execute_gait(currentGait); 
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