#include <Arduino.h>
#include <IRremote.h>

int RECV_PIN = 3;
IRrecv irrecv(RECV_PIN);
decode_results results;

bool flag = 0;
int LED_PIN = 9;

void setup() {
    Serial.begin(9600);
    pinMode(LED_PIN, INPUT);
    irrecv.enableIRIn();
}

void loop() {
    if (irrecv.decode(&results)) {
        if (results.value == 0xFF02FD & flag == 0) {
            digitalWrite(LED_PIN, HIGH);
            flag = 1;
        }
        else if (results.value == 0xFF02FD & flag == 1) {
            digitalWrite(LED_PIN, LOW);
            flag = 0;
        }

        Serial.println(results.value, HEX);
        Serial.println("flag = " + String(flag));
        irrecv.resume();
    }

    delay(100);
}
