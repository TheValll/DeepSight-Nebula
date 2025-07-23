#include <Servo.h>
Servo myservo;

int currentAngle = 90;

void setup() {
  Serial.begin(9600);
  myservo.attach(9);
  myservo.write(currentAngle);
  Serial.print("Angle initial : ");
  Serial.println(currentAngle);
}

void loop() {
  if (Serial.available() > 0) {
    String msg = Serial.readStringUntil('\n');
    msg.trim();

    int delta = msg.toInt(); 
    int newAngle = currentAngle + delta;

    if (newAngle >= 0 && newAngle <= 180) {
      currentAngle = newAngle;
      myservo.write(currentAngle);
      Serial.print("Nouvel angle : ");
      Serial.println(currentAngle);
    } else {
      Serial.println("Correction invalide. L'angle doit rester entre 0 et 180.");
    }
  }

  delay(50);
}
