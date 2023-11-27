#include <Arduino.h>
#include <ESP32Servo.h>

esp32servo::Servo myservo;

void setup() {
  // put your setup code here, to run once:
  myservo.setPeriodHertz(50);
  myservo.attach(17);
  myservo.setupServoType(esp32servo::ServoType::Common360);
  // ledcSetup(0, 50, 16);
  // ledcAttachPin(17, 0);
}

void loop() {
  // put your main code here, to run repeatedly:
  // ledcWrite(0, 8192);
  // delay(125);
  // ledcWrite(0, 0);
  // delay(2000);
  // ledcWrite(0, 4096);
  // delay(250);
  // ledcWrite(0, 0);
  // delay(2000);
  // delay(2000);
  // ledcWrite(0, 8192);
  // delay(125);
  // ledcWrite(0, 0);
  // delay(2000);

  myservo.writeAngle(90);
  delay(2000);
  myservo.writeAngle(-90);
  delay(2000);
}
