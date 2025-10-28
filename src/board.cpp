#include "board.h"

// Definicje zmiennych globalnych
BluetoothSerial SerialBT;
TaskHandle_t ScreenUpdateHandle;
TaskHandle_t ClientCmdHandle;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

const int MAX_SERVOS = 10;
bool servosFound[MAX_SERVOS] = {false};
int foundCount = 0;
bool scanComplete = false;
String btAddress = "";

void logo() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(15, 5);
  display.println("ROBOTICS");

  display.setTextSize(1);
  display.setCursor(22, 25);
  display.println("CONTROL SYSTEM");
  display.drawLine(0, 35, 128, 35, SSD1306_WHITE);

  display.display(); 
}

void InitScreen() {
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
  }
  logo();
  delay(1000); 
}

void scanServos() {
  foundCount = 0;
  for (int id = 1; id <= MAX_SERVOS; id++) {
    int result = st.Ping(id);
    if (result != -1) {
      servosFound[id-1] = true;
      foundCount++;
    } else {
      servosFound[id-1] = false;
    }
    delay(50);
  }
}

void btMac() {
  if (!SerialBT.begin("ESP32_BT")) {
    Serial.println("Błąd inicjalizacji Bluetooth!");
    while (true);
  }

  const uint8_t* mac = esp_bt_dev_get_address();
  if (mac) {
    btAddress = "";
    for (int i = 0; i < 6; i++) {
      if (i != 0) btAddress += ":";
      if (mac[i] < 16) btAddress += "0";
      btAddress += String(mac[i], HEX);
    }
    btAddress.toUpperCase();

    Serial.print("Adres MAC Bluetooth ESP32: ");
    Serial.println(btAddress);
  } else {
    Serial.println("Nie udało się odczytać adresu MAC.");
    btAddress = "N/A";
    delay(50);
  }
}

void displayResultsScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  display.setCursor(0, 0);
  display.println("Scan results:");
  display.println("-------------------");
  
  if (foundCount == 0) {
    display.setCursor(0, 16);
    display.println("No servos detected!");
  } else {
    display.setCursor(0, 16);
    display.print("S: ");
    for (int i = 0; i < MAX_SERVOS; i++) {
      if (servosFound[i]) {
        display.print(i+1);
        display.print(" ");
      }
    }
  }

  display.setCursor(0, 25);
  display.print("BT: ");
  display.println(btAddress);

  display.display();
}

void ConnectedText() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(4, 5);
  display.println("CONTROLLER");

  display.setTextSize(1);
  display.setCursor(37, 25);
  display.println("CONNECTED");
  display.drawLine(0, 35, 128, 35, SSD1306_WHITE);

  display.display(); 
  delay(1000); 

  logo();
}

void displayVoltageScreen(float voltage) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  // Nagłówek
  display.setCursor(0, 0);
  display.println("Servo Voltage:");
  display.println("-------------------");
  
  // Wyświetlanie napięcia z precyzją do 2 miejsc po przecinku
  display.setTextSize(2);
  display.setCursor(10, 16);
  display.print(voltage, 2);
  display.setTextSize(1);
  display.print(" V");
  
  display.display();
  delay(3000); 
  logo();
}

void displayErrorScreen(const String& errorMessage) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  display.setCursor(0, 0);
  display.println("VOLTAGE ERROR:");
  display.println("-------------------");
  
  display.setCursor(0, 16);
  display.println(errorMessage);
  
  display.display();
  delay(3000); 
  logo();

}

void ShowVoltage() {
  float volt = st.ReadVoltage(1) / 10;  // Odczyt napięcia z serwa o ID 1
  
  if (volt == -1) {
    displayErrorScreen("Read error");
    Serial.println("Błąd odczytu napięcia z serwa!");
  } else if (volt == 0) {
    displayErrorScreen("No servo response");
    Serial.println("Serwo nie odpowiada!");
  } else {
    displayVoltageScreen(volt);
    
  }
}