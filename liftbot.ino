// Alex Hernandez 17' and Ray Rich-Fiondella 17' COM310 Robotics
#include <AFMotor.h>
#include <Servo.h> 
#include <NewPing.h>

//Default pin setup
#define TRIG_PIN A0
#define ECHO_PIN A1
#define MAX_DIST 1000
int N = A2;
int E = A3;
int S = A4;
int W = A5;

NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DIST); // set up sensor object

AF_DCMotor motor(2); // objects for both motors
AF_DCMotor motor2(4);

Servo servo; // objects for both servos
Servo servo2;

int i = 0;
int png = 0;
int s1pos = 0; // stores servo angle position for both servos
int s2pos = 0;

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps

  //IR Beacon pin setup
  pinMode(N, INPUT);
  pinMode(E, INPUT);
  pinMode(S, INPUT);
  pinMode(W, INPUT);
  delay(100);
  
  motor.setSpeed(255);     // set motor speeds (200 / 255)
  motor2.setSpeed(255);

  servo.attach(9); // set up servos according to pins on adafruit motor shield
  servo2.attach(10); 
}

void loop() {
  // lift the device when it detects robot in close range
  // liftUp();
  // liftDown();
  png = readPing();
  Serial.println(png);
  if (png > 8 & png < 15) {
    liftUp();
    liftDown();
  }
  else{
    motor.run(BACKWARD);
    motor2.run(BACKWARD);
  }

  // Testing
  
  /*motor.run(BACKWARD);
  motor2.run(BACKWARD);
  liftUp();
  liftDown();
  delay(5000);

  liftUp();
  delay(1000);
  liftDown();
  
  motor.run(FORWARD); 
  motor2.run(FORWARD);
  delay(5000);
  
  motor.run(RELEASE);      // stopped
  motor2.run(RELEASE);
  delay(1000);*/

  //Read IR Beacon
  float North = digitalRead(N);
  float East = digitalRead(E);
  float South = digitalRead(S);
  float West = digitalRead(W);
  Serial.println("N " + String(North));
  Serial.println("E " + String(East));
  Serial.println("S " + String(South));
  Serial.println("W " + String(West));
  Serial.println();
}
  
void liftUp() {
  // lifts up the device using the double servo system
  s1pos = -93;
  s2pos = 93;
  for (i = 1; i <= 25; i += 1) { // go up every 15ms 25 times
    // in steps of 1 degree
    servo.write(s1pos+1);              // increment positions over servos up one position
    servo2.write(s2pos-1);            // this incrementation is inverse for the other servo since
                                     // it is placed on backwards
    delay(15);                       // waits 15ms for the servo to reach the position
    s1pos = s1pos + 1;
    s2pos = s2pos - 1;
  }
  //reset servo position variables
  s1pos = -93;
  s2pos = 93;
}

void liftDown() {
  // lifts down the device using the double servo system
  s1pos = 133;
  s2pos = -133;
  for (i = 1; i <= 25; i += 1) { // go down every 15ms 25 times
    // in steps of 1 degree
    servo.write(s1pos-1);              // increment positions over servos down one position
    servo2.write(s2pos+1);
    delay(15);                       // waits 15ms for the servo to reach the position
    s1pos = s1pos - 1;
    s2pos = s2pos + 1;
  }
  s1pos = 133;
  s2pos = -133;
}

int readPing() { // return ping sensor distance in centimeters
  delay(70);
  unsigned int val = sonar.ping();
  int cm = val / US_ROUNDTRIP_CM;
  return cm;
}

