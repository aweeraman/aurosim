
#include <AFMotor.h>
#include <TimerOne.h>

AF_DCMotor front_left(1);
AF_DCMotor front_right(2);

const int TRIG = 12;
const int ECHO = 7;

const int ROVER_SPEED = 120;
const int ROVER_TURN_SPEED = 100;

const int ENCODER_PIN_RIGHT = 9;
const int ENCODER_PIN_LEFT = A2;

volatile int right_counter = 0;
volatile int left_counter = 0;

void setup() {
  Serial.begin(9600);

  pinMode (TRIG, OUTPUT);
  pinMode (ECHO, INPUT);
  
  front_left.setSpeed(1);
  front_left.run(RELEASE);

  front_right.setSpeed(1);
  front_right.run(RELEASE);

  pciSetup(ENCODER_PIN_RIGHT);
  pciSetup(ENCODER_PIN_LEFT);

  Timer1.initialize(1000000); // set timer for 1sec
  Timer1.attachInterrupt( timerIsr ); // enable the timer
}

void loop() {
  move(ping());
}

void timerIsr()
{
  Timer1.detachInterrupt();  //stop the timer
  Serial.print("Motor Speed: Right="); 
  int right_rotation = (right_counter / 2 / 20);
  int left_rotation = (left_counter / 2 / 20);
  Serial.print(right_counter, DEC);  
  Serial.print(", LEFT="); 
  Serial.println(left_counter, DEC);
  right_counter=0;  //  reset counter to zero
  left_counter=0;
  Timer1.attachInterrupt( timerIsr );  //enable the timer
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
  
  //Serial.print(cm);
  //Serial.println(" cm");
  
  return cm;
}

void move(long distanceToObstacle) {
  long rand = random(100);
  
//  if (distanceToObstacle < 50) {
//    turnRight();
//  } else {
//    moveForward();
//  }

// Move in a straight line for the moment
  moveForward();
}

void setRoverSpeed(int speed) {

  front_left.setSpeed(speed);
  front_right.setSpeed(speed);
  
  //delay(5);
}

void moveForward() {
  setRoverSpeed(ROVER_SPEED);

  front_left.run(FORWARD);
  front_right.run(FORWARD);
}

void stop() {
  setRoverSpeed(0);
}

void turnRight() {
  front_left.setSpeed(ROVER_TURN_SPEED);
  front_left.run(FORWARD);

  front_right.setSpeed(0);
  front_right.run(FORWARD);
}

void turnLeft() {
  front_left.setSpeed(0);
  front_left.run(FORWARD);

  front_right.setSpeed(ROVER_TURN_SPEED);
  front_right.run(FORWARD);
}

void pciSetup(byte pin)
{
    *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
    PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
    PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}

ISR (PCINT0_vect) // handle pin change interrupt for D0 to D7 here
{
    right_counter++;
}  

ISR (PCINT1_vect) // handle pin change interrupt for D8 to D13 here
{    
    left_counter++;
}

