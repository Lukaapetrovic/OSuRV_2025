
// Adafruit Motor Shield Library
#include <AFMotor.h>

AF_Stepper motor1(200, 1); // Unipolar
AF_Stepper motor2(200, 2); // Bipolar

void setup() {
  Serial.begin(9600);
  Serial.println("Stepper test!");

  motor1.setSpeed(50); // [RPM] 
  motor2.setSpeed(100); // [RPM] 
}

void loop() {
  Serial.println("Unipolar stepper");
  motor1.step(200, FORWARD, SINGLE); 
  Serial.println("Bipolar stepper");
  motor2.step(200, FORWARD, SINGLE); 
}
