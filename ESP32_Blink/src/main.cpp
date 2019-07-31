#include <Arduino.h>

#define LED0_PIN 13

void setup() {
  Serial.begin(115200);
  pinMode(LED0_PIN, OUTPUT);
}

long t_prev;
bool ledOn;

void loop() {
  long t = millis();
  
  if (t > t_prev) {
    if (ledOn) {
      digitalWrite(LED0_PIN, LOW);
      ledOn = false;
      Serial.println("OFF");
    }
    else {
      digitalWrite(LED0_PIN, HIGH);
      ledOn = true;
      Serial.println("ON");
    }
    t_prev += 1000;
  }
}