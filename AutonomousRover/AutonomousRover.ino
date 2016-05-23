
#include <AFMotor.h>

AF_DCMotor front_left(1);
AF_DCMotor front_right(2);

const int TRIG = 12;
const int ECHO = 7;

const int ROVER_SPEED = 75;
const int ROVER_TURN_SPEED = 100;

void setup() {
  Serial.begin(9600);

  pinMode (TRIG, OUTPUT);
  pinMode (ECHO, INPUT);

  front_left.setSpeed(1);
  front_left.run(RELEASE);

  front_right.setSpeed(1);
  front_right.run(RELEASE);
}

void loop() {
  move(ping());
}

int ping() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  pinMode(ECHO, INPUT);
  long duration = pulseIn(ECHO, HIGH);
  long cm = duration / 29 / 2;
  
  Serial.print(cm);
  Serial.println(" cm");
  
  return cm;
}

void move(long distanceToObstacle) {
  long rand = random(100);
  if (distanceToObstacle < 50) {
      turnRight();
  } else {
    moveForward();
  }
}

void setRoverSpeed(int speed) {
  Serial.print("Setting speed ");
  Serial.println(speed);

  front_left.setSpeed(speed);
  front_right.setSpeed(speed);
  
  delay(5);
}

void moveForward() {
  Serial.println("Moving forward");
  
  setRoverSpeed(ROVER_SPEED);

  front_left.run(FORWARD);
  front_right.run(FORWARD);
}

void stop() {
  Serial.println("Stopping");
  setRoverSpeed(0);
}

void turnRight() {
  Serial.println("Turning right");

  front_left.setSpeed(ROVER_TURN_SPEED);
  front_left.run(FORWARD);

  front_right.setSpeed(0);
  front_right.run(FORWARD);
}

void turnLeft() {
  Serial.println("Turning left");

  front_left.setSpeed(0);
  front_left.run(FORWARD);

  front_right.setSpeed(ROVER_TURN_SPEED);
  front_right.run(FORWARD);
}

