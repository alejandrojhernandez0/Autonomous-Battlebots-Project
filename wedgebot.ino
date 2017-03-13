// Alex Hernandez 17' and Ray Rich-Fiondella 17' COM310 Robotics
#include <AFMotor.h>
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

AF_DCMotor motor(3);
AF_DCMotor motor2(4);

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Motor test!");

  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  delay(100);
  
  motor.setSpeed(255);     // set the speed to 200/255
  motor2.setSpeed(255);
}

void loop() {
  //Use ping data, movement
  Serial.println(readPing());
  motor.run(FORWARD);
  motor2.run(FORWARD);
  delay(5000);
  
  motor.run(BACKWARD); 
  motor2.run(BACKWARD);
  delay(5000);
  
  motor.run(RELEASE);      // stopped
  motor2.run(RELEASE);
  delay(1000);

  //Take data from IR Beacon
  int North = digitalRead(N);
  int East = digitalRead(E);
  int South = digitalRead(S);
  int West = digitalRead(W);
  Serial.println("N " + String(North));
  Serial.println("E " + String(East));
  Serial.println("S " + String(South));
  Serial.println("W " + String(West));
  Serial.println();
  delay(2000);
}

int readPing() { // return ping sensor distance in centimeters
  delay(70);
  unsigned int val = sonar.ping();
  int cm = val / US_ROUNDTRIP_CM;
  return cm;
}

