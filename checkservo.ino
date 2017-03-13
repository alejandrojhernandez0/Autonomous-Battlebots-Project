#include <Servo.h> 

Servo servo; // objects for both servos
Servo servo2;

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Servo test!");

  servo.attach(9); // set up servos according to pins on adafruit motor shield
  servo2.attach(10);
  
}

void loop() {
  Serial.println(servo.read());
  Serial.println(servo2.read());
}
