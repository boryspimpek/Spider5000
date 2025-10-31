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
    SUBMODE_TC,
    // SUBMODE_TD,
    SUBMODE_COUNT_T
};

// Definicje podtrybów dla trybu DEFAULT
enum DefaultSubMode {
    SUBMODE_DA = 0,
    SUBMODE_DB,
    // SUBMODE_DC,
    // SUBMODE_DD,
    SUBMODE_COUNT_D
};

// Globalne zmienne
ModeFlag currentMode = MODE_DEFAULT;
TriangleSubMode triangleSubMode = SUBMODE_TA;
DefaultSubMode defaultSubMode = SUBMODE_DA;

GaitMode gaitDA = TROT_FORWARD;
GaitMode gaitDB = CREEP_FORWARD;

bool buttonsActive = true;
unsigned long lastModeChange = 0;
const unsigned long MODE_CHANGE_DELAY = 300; 

struct ControllerData {
    // Surowe wartości gałek
    int lx_raw, ly_raw, rx_raw, ry_raw;
    
    // Wartości po deadzone
    int lx, ly, rx, ry;
    
    // Znormalizowane wartości
    float lx_norm, ly_norm, rx_norm, ry_norm;
    
    // Przyciski
    bool r1, r2, up, down, left, right;
    
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
        else if (currentMode != MODE_TRIANGLE) {
            defaultSubMode = SUBMODE_DA;
        }
        
        buttonsActive = true;
    }
}

void handleTriangleSubModeChange(bool moveRight) {
    unsigned long currentTime = millis();
    
    if (buttonsActive && (currentTime - lastModeChange) > MODE_CHANGE_DELAY) {
        if (moveRight) {
            // Zmiana w prawo (R1)
            triangleSubMode = (TriangleSubMode)((triangleSubMode + 1) % SUBMODE_COUNT_T);
        } else {
            // Zmiana w lewo (L1)
            triangleSubMode = (TriangleSubMode)((triangleSubMode - 1 + SUBMODE_COUNT_T) % SUBMODE_COUNT_T);
        }
        
        buttonsActive = false;
        lastModeChange = currentTime;
        
        buttonsActive = true;
    }
}

void handleDefaultSubModeChange(bool moveRight) {
    unsigned long currentTime = millis();
    
    if (buttonsActive && (currentTime - lastModeChange) > MODE_CHANGE_DELAY) {
        if (moveRight) {
            // Zmiana w prawo (R1)
            defaultSubMode = (DefaultSubMode)((defaultSubMode + 1) % SUBMODE_COUNT_D);
        } else {
            // Zmiana w lewo (L1)
            defaultSubMode = (DefaultSubMode)((defaultSubMode - 1 + SUBMODE_COUNT_D) % SUBMODE_COUNT_D);
        }
        
        buttonsActive = false;
        lastModeChange = currentTime;
        
        buttonsActive = true;
    }
}

void processTriangleSubModes(ControllerData data) {
    
    switch(triangleSubMode) {
        case SUBMODE_TA:
            if (data.left) {
                pushup();
            }
            else if (data.right) {
                pushupOneLeg();
            }
            else if (data.up) {
                hello();
            }
            else if (data.down) {
                sit();
            }
            break;

        case SUBMODE_TB:
            if (data.left) {
                steps();
            }
            else if (data.right) {
                bounce();
            }
            else if (data.up) {
                dive();
            }
            else if (data.down) {
                sayNo();
            }
            break;
            
        case SUBMODE_TC:
            if (data.left) {
                downLeft();
            }
            else if (data.right) {
                downRight();
            }
            else if (data.up) {
                downFront();
            }
            else if (data.down) {
                downBack();
            }
            break;
            
        // case SUBMODE_TD:
        //     // Podtryb D - precyzyjne sterowanie
        //     if (abs(data.lx) > 2) {
        //         Serial.printf("Triangle-D");
        //     }
            
        //     break;
    }
}

void processDefaultSubModes(ControllerData data) {
    
    switch(defaultSubMode) {
        case SUBMODE_DA:
            if (data.leftStickActive) {
                running = true;
                if (data.ly_norm > 0.5) gaitDA = TROT_FORWARD;
                else if (data.ly_norm < -0.5) gaitDA = TROT_BACKWARD;
                else if (data.lx_norm > 0.5) gaitDA = TROT_RIGHT;
                else if (data.lx_norm < -0.5) gaitDA = TROT_LEFT;
            }
            else if (data.up) {
                h += 2; 
                return_to_neutral(); 
                if (h > 50) h = 50;
                Serial.print(h);
            }
            else if (data.down) {
                h -= 2; 
                return_to_neutral(); 
                if (h < 0) h = 0;
                Serial.print(h);
            }
            else if (data.left) {
                x_amp += 2; 
                return_to_neutral(); 
                if (x_amp > 30) x_amp = 30;
                Serial.print(x_amp);
            }
            else if (data.right) {
                x_amp -= 2; 
                return_to_neutral(); 
                if (x_amp < 15) x_amp = 15;
                Serial.print(x_amp);
            }
            else {
                running = false; 
                gait_phase = 0.0; 
                return_to_neutral();
            }     
            break;
            
        case SUBMODE_DB:
            if (data.leftStickActive) {
                running = true;
                if (data.ly_norm > 0.5) gaitDB = CREEP_FORWARD;
                else if (data.ly_norm < -0.5) gaitDB = CREEP_BACKWARD;
                else if (data.lx_norm > 0.5) gaitDB = CREEP_RIGHT;
                else if (data.lx_norm < -0.5) gaitDB = CREEP_LEFT;
            }  
            else if (data.up) {
                h += 2; 
                return_to_neutral(); 
                if (h > 50) h = 50;
                Serial.printf("Height increased to: %d\n", h);
            }
            else if (data.down) {
                h -= 2; 
                return_to_neutral(); 
                if (h < 0) h = 0;
                Serial.printf("Height decreased to: %d\n", h);
            }
            else {
                running = false; 
                gait_phase = 0.0; 
                return_to_neutral();
            }     
            break;
            
        // case SUBMODE_DC:
        //     if (data.left) {
        //         Serial.println("DC");
        //     }
        //     else {
        //         running = false; 
        //         gait_phase = 0.0; 
        //         return_to_neutral();
        //     }
        //     break;
            
        // case SUBMODE_DD:
        //     if (data.left) {
        //         Serial.printf("DD");
        //     }
        //     else {
        //         running = false; 
        //         gait_phase = 0.0; 
        //         return_to_neutral();
        //     }
        //     break;    
    }    
}

void process_PS4_input() {
    if (!PS4.isConnected()) return;

    // Globalne akcje przycisków (działają zawsze)
    if (PS4.Share()) {ShowVoltage();}

    // Odczyt wszystkich danych kontrolera
    ControllerData data = read_PS4_input();

    // DEBUG: Wyświetl wartości dla testów
    // static unsigned long lastDebug = 0;
    // if (millis() - lastDebug > 500) {
    //     Serial.printf("LX: %d, LY: %d, RX: %d, RY: %d\n", data.lx, data.ly, data.rx, data.ry);
    //     Serial.printf("Up: %d, Down: %d, Left: %d, Right: %d\n", data.up, data.down, data.left, data.right);
    //     lastDebug = millis();
    // }

    switch(currentMode) {
        case MODE_DEFAULT:
            processDefaultSubModes(data);
            break;

        case MODE_TRIANGLE:
            processTriangleSubModes(data);
            break;

        case MODE_SQUARE:
            // Wspólna inicjalizacja dla wszystkich warunków
            if (data.leftStickActive || data.rightStickActive) {
                running = false;
                gait_phase = 0.0;
            }

            // Prawa gałka - sterowanie pochyleniem
            if (data.rightStickActive) {
                int front = data.ry_norm * maxDeviation;
                int rear = -data.ry_norm * maxDeviation;
                int left = -data.rx_norm * maxDeviation;
                int right = data.rx_norm * maxDeviation;
                
                move_servo_smooth(2, (90 - h + front + left));
                move_servo_smooth(4, (90 + h - front - right));
                move_servo_smooth(6, (90 + h - rear - left));
                move_servo_smooth(8, (90 - h + rear + right));
            }

            // Lewa gałka - sterowanie ruchem podstawowym
            if (data.leftStickActive) {
                int front = data.ly_norm * maxDeviation;  
                int rear = -data.ly_norm * maxDeviation;    
                int left = data.lx_norm * maxDeviation;   
                int right = -data.lx_norm * maxDeviation;   
                
                move_servo_smooth(1, (45 + front + left));
                move_servo_smooth(3, (135 - front - right));
                move_servo_smooth(5, (135 - rear - left));
                move_servo_smooth(7, (45 + rear + right));
            }

            // Regulacja wysokości
            if (data.up) {
                h += 2; 
                if (h > 50) h = 50;
                Serial.printf("Height increased to: %d\n", h);
                return_to_neutral(); // Tylko raz po zmianie wysokości
            }
            if (data.down) {
                h -= 2; 
                if (h < 0) h = 0;
                Serial.printf("Height decreased to: %d\n", h);
                return_to_neutral(); // Tylko raz po zmianie wysokości
            }

            // Powrót do neutralnej tylko gdy żadna gałka nieaktywna
            if (!data.leftStickActive && !data. rightStickActive && !data.up && !data.down) {
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
    
    // Obsługa zmiany podtrybów w trybie TRIANGLE
    if (currentMode == MODE_TRIANGLE) {
        if (PS4.R1() && !PS4.L1()) { // Tylko R1 wciśnięte
            handleTriangleSubModeChange(true); // Zmiana w prawo
        } else if (PS4.L1() && !PS4.R1()) { // Tylko L1 wciśnięte
            handleTriangleSubModeChange(false); // Zmiana w lewo
        }
    }

    // Obsługa zmiany podtrybów w trybie DEFAULT
    if (currentMode == MODE_DEFAULT) {
        if (PS4.R1() && !PS4.L1()) { // Tylko R1 wciśnięte
            handleDefaultSubModeChange(true); // Zmiana w prawo
            Serial.printf("Default mode change right");

        } else if (PS4.L1() && !PS4.R1()) { // Tylko L1 wciśnięte
            handleDefaultSubModeChange(false); // Zmiana w lewo
            Serial.printf("Default mode change left");
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

// Funkcja pomocnicza do pobierania aktualnego gaitu
GaitMode getCurrentGait() {
    switch(defaultSubMode) {
        case SUBMODE_DA: return gaitDA;
        case SUBMODE_DB: return gaitDB;
        default: return CREEP_FORWARD;
    }
}

void loop() {    
    if (PS4.isConnected()) {
        process_PS4_input();
        processButtons();
        
        if (running) {
            GaitMode currentGait = getCurrentGait(); // Pobierz odpowiedni gait dla aktywnego submode'u
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