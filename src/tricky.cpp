#include <Arduino.h>
#include "servos.h"
#include "tricky.h"   

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

void pushup() {
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
        move_servo(4, 90);
        delay(700); 

        move_servo(2, 30);
        move_servo(4, 150);
        delay(700);
    }

    return_to_neutral();
}

void bounce() {    
    // down
    move_servo(2, 90-h+15);
    move_servo(4, 90+h-15);
    move_servo(6, 90+h-15);
    move_servo(8, 90-h+15);
    delay(500);
    
    // up without 2 leg
    move_servo(2, 90-h+25);
    move_servo(4, 90+h+15);
    move_servo(6, 90+h+15);
    move_servo(8, 90-h-15);
    delay(500);
  
  //down
    move_servo(2, 90-h+15);
    move_servo(4, 90+h-15);
    move_servo(6, 90+h-15);
    move_servo(8, 90-h+15);
    delay(500);
    
    // up without 4 leg
    move_servo(2, 90-h-15);
    move_servo(4, 90+h-25);
    move_servo(6, 90+h+15);
    move_servo(8, 90-h-15);
    delay(500);
    
    // down
    move_servo(2, 90-h+15);
    move_servo(4, 90+h-15);
    move_servo(6, 90+h-15);
    move_servo(8, 90-h+15);
    delay(500);
    
    // up without 6 leg
    move_servo(2, 90-h-15);
    move_servo(4, 90+h+15);
    move_servo(6, 90+h-25);
    move_servo(8, 90-h-15);
    delay(500);
    
    // down
    move_servo(2, 90-h+15);
    move_servo(4, 90+h-15);
    move_servo(6, 90+h-15);
    move_servo(8, 90-h+15);
    delay(500);
    
    // up without 8 leg
    move_servo(2, 90-h-15);
    move_servo(4, 90+h+15);
    move_servo(6, 90+h+15);
    move_servo(8, 90-h+25);
    delay(500);
    
    return_to_neutral();
  }
  
void sayNo() {
  for (int i = 0; i < 1; i++) {
    move_servo(1, 75);
    move_servo(3, 165);
    move_servo(5, 165);
    move_servo(7, 75);
    delay(600);
    
    move_servo(1, 15);
    move_servo(3, 105);
    move_servo(5, 105);
    move_servo(7, 15);
    delay(600);
  }
  return_to_neutral();
}
  
void dive() {
  for (int i = 0; i < 3; i++) {
    move_servo(1, 75);
    move_servo(3, 105);
    move_servo(5, 165);
    move_servo(7, 15);
    
    move_servo(2, 90 - h - 20);
    move_servo(4, 90 + h + 20);
    move_servo(6, 90 + h - 20);
    move_servo(8, 90 - h + 20);
    delay(500);
    
    move_servo(1, 15);
    move_servo(3, 165);
    move_servo(5, 105);
    move_servo(7, 75);
    
    move_servo(2, 90 - h + 20);
    move_servo(4, 90 + h - 20);
    move_servo(6, 90 + h + 20);
    move_servo(8, 90 - h - 20);
    delay(500);
  }
  
  return_to_neutral();
}
  
void frontSteps() {
  for (int i = 0; i < 2; i++) {
        move_servo(2, 120);
        move_servo(4, 120);
        delay(700);

        move_servo(2, 60);
        move_servo(4, 60);
        delay(700);

    return_to_neutral();
  }
}

void playDead() {
    move_servo(1, 90);
    move_servo(2, 140);
    move_servo(3, 90);
    move_servo(4, 40);
    move_servo(5, 90);
    move_servo(6, 40);
    move_servo(7, 90);
    move_servo(8, 140);

    delay(3000);
    return_to_neutral();
}

void twoLegUp() {
    move_servo(1, 45);
    move_servo(2, 140);
    move_servo(3, 135);
    move_servo(4, 110);
    move_servo(5, 135);
    move_servo(6, 110);
    move_servo(7, 45);
    move_servo(8, 140);
    delay(700);

    for (int i = 0; i < 2; i++) {
        move_servo(4, 60);
        move_servo(6, 140);
        delay(700);

        move_servo(4, 140);
        move_servo(6, 60);
        delay(700);
    }

    return_to_neutral();
}

void twoLegMove() {
    move_servo(1, 45);
    move_servo(2, 70);
    move_servo(3, 135);
    move_servo(4, 40);
    move_servo(5, 135);
    move_servo(6, 40);
    move_servo(7, 45);
    move_servo(8, 70);
    delay(500);

    for (int i = 0; i < 2; i++) {
        move_servo(1, 75);
        move_servo(7, 75);
        delay(700);

        move_servo(1, 15);
        move_servo(7, 15);
        delay(700);
    }

    return_to_neutral();
}

void left() {
  Serial.println("Left");
}
void right() {
  Serial.println("Right");
}
void front() {
  Serial.println("Front");
}
void back() {
  Serial.println("Back");
}