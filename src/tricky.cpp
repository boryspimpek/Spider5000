#include <Arduino.h>
#include "servos.h"
#include "tricky.h"   // dołączenie nagłówka z deklaracją funkcji

void hello() {
  const ServoPos HELLO_POS[] = {
    {1, 45}, {2, 30}, {3, 135}, {4, 150},
    {5, 105}, {6, 70}, {7, 75}, {8, 110}
  };

  const int NUM_SERVOS = sizeof(HELLO_POS) / sizeof(HELLO_POS[0]);

  // Ustawienie pozycji "hello"
  for (int i = 0; i < NUM_SERVOS; i++) {
    move_servo(HELLO_POS[i].id, HELLO_POS[i].angle);
  }

  delay(500);
  move_servo(2, 90);

  // Machanie dwa razy
  for (int i = 0; i < 2; i++) {
    move_servo(1, 10);
    delay(500);
    move_servo(1, 60);
    delay(500);
  }

  delay(500);
  return_to_neutral();
}

void pushupOneLeg() {
  const ServoPos PUSHUP_POS[] = {
    {1, 90}, {2, 90}, {3, 90}, {4, 90},
    {5, 135}, {6, 110}, {7, 45}, {8, 70}
  };

  const int NUM_SERVOS = sizeof(PUSHUP_POS) / sizeof(PUSHUP_POS[0]);

  for (int i = 0; i < NUM_SERVOS; i++) {
    move_servo(PUSHUP_POS[i].id, PUSHUP_POS[i].angle);
  }

  for (int i = 0; i < 3; i++) {
    move_servo(2, 90);
    delay(700); 
    move_servo(2, 30);
    delay(700);
  }
  move_servo(2, 90); 

  for (int i = 0; i < 3; i++) {
    move_servo(4, 90);
    delay(700);
    move_servo(4, 150);
    delay(700);
  }

  return_to_neutral();
}

void pushup() {
    const ServoPos PUSHUP_POS[] = {
        {1, 90}, {2, 90}, {3, 90}, {4, 90},
        {5, 135}, {6, 110}, {7, 45}, {8, 70}
    };

    const int NUM_SERVOS = sizeof(PUSHUP_POS) / sizeof(PUSHUP_POS[0]);

    // Ustaw wszystkie serwa w pozycji startowej push-up
    for (int i = 0; i < NUM_SERVOS; i++) {
        move_servo(PUSHUP_POS[i].id, PUSHUP_POS[i].angle);
    }

    // Ruch dwóch serw jednocześnie (2 i 4) 3 razy
    for (int i = 0; i < 3; i++) {
        move_servo(2, 90);
        move_servo(4, 90);
        delay(700); // 0.7 sekundy

        move_servo(2, 30);
        move_servo(4, 150);
        delay(700);
    }

    return_to_neutral();
}
