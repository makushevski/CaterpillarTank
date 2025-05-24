#include <Arduino.h>

int ML_Ctrl = 4;
int ML_PWM = 6;
int MR_Ctrl = 2;
int MR_PWM = 5;

void move(const uint8_t valLeft, const int speedLeft, const uint8_t valRight, const int speedRight) {
    digitalWrite(ML_Ctrl, valLeft);
    analogWrite(ML_PWM, speedLeft);
    digitalWrite(MR_Ctrl, valRight);
    analogWrite(MR_PWM, speedRight);
    delay(2000);
}

void setup() {
    pinMode(ML_Ctrl, OUTPUT);
    pinMode(ML_PWM, OUTPUT);
    pinMode(MR_Ctrl, OUTPUT);
    pinMode(MR_PWM, OUTPUT);
}

void loop() {
    move(HIGH, 55, HIGH, 55);
    move(LOW, 200, LOW, 200);
    move(LOW, 200, HIGH, 55);
    move(HIGH, 55, LOW, 200);
    move(LOW, 0, LOW, 0);
}
