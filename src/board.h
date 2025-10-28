#ifndef BOARD_H
#define BOARD_H

#include <Wire.h>
#include "BluetoothSerial.h"
#include "esp_bt_device.h"
#include <Adafruit_SSD1306.h>
#include <SMS_STS.h>

// Konfiguracja pinów
#define S_RXD 18
#define S_TXD 19
#define S_SCL 22
#define S_SDA 21
#define RGB_LED 23
#define NUMPIXELS 10

// Konfiguracja wyświetlacza
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

// Zmienne globalne
extern BluetoothSerial SerialBT;
extern TaskHandle_t ScreenUpdateHandle;
extern TaskHandle_t ClientCmdHandle;
extern SMS_STS st;
extern Adafruit_SSD1306 display;

extern const int MAX_SERVOS;
extern bool servosFound[];
extern int foundCount;
extern bool scanComplete;
extern String btAddress;

// Deklaracje funkcji
void logo();
void InitScreen();
void scanServos();
void btMac();
void displayResultsScreen();
void ConnectedText();
void displayVoltageScreen(float voltage);        
void displayErrorScreen(const String& errorMessage);
void ShowVoltage(); 
#endif