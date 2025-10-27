// servo.h

#include <Arduino.h>
#include <SCServo.h>

// Servo control objects
SMS_STS st;

// Servo settings
const int sts_id[8] = {1, 2, 3, 4, 5, 6, 7, 8};
const int acc = 250;
const int speed = 2400;

const float DEADZONE = 0.2;

const int SERVO_TRIMS[8] = {-25, 15, 30, 0, -15, 0, -10, 45};

const int SERVO_LIMITS[8][2] = {
    {0, 90},                // servo 1
    {30, 140},              // servo 2
    {90, 180},              // servo 3
    {40, 150},              // servo 4
    {90, 180},              // servo 5
    {40, 150},              // servo 6
    {0, 90},                // servo 7
    {30, 140}               // servo 8
};    

int angle_deg_to_servo(float deg) {
    float rad = radians(deg);  // ° → rad
    int center = 2048;
    float scale = 2048.0 / PI;
    return 4095 - (int)round(rad * scale + center);
}

int check_angle_limit(int id, int angle_deg) {
    if (id < 1 || id > 8) return angle_deg;
    
    int min_angle = SERVO_LIMITS[id-1][0];
    int max_angle = SERVO_LIMITS[id-1][1];
    
    if (angle_deg < min_angle) {
        Serial.printf("Servo %d: kąt %d° poniżej minimum (%d°) — ograniczono.\n", id, angle_deg, min_angle);
        angle_deg = min_angle;
    } else if (angle_deg > max_angle) {
        Serial.printf("Servo %d: kąt %d° powyżej maksimum (%d°) — ograniczono.\n", id, angle_deg, max_angle);
        angle_deg = max_angle;
    }
    return angle_deg;
}

void move_servo(int id, int angle_deg) {
    int safe_angle = check_angle_limit(id, angle_deg);
    int pos = angle_deg_to_servo(safe_angle);
    int trimmed_pos = pos + SERVO_TRIMS[id-1];
    st.WritePosEx(id, trimmed_pos, speed, acc);
}

void move_servo_smooth(int id, int angle_deg) {
    int safe_angle = check_angle_limit(id, angle_deg);
    int pos = angle_deg_to_servo(safe_angle);
    int trimmed_pos = pos + SERVO_TRIMS[id-1];
    st.WritePosEx(id, trimmed_pos, 500, 50);
}
