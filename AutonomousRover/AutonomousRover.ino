#include <PID_v1.h>
#include <AFMotor.h>
#include <TimerOne.h>

AF_DCMotor front_left(1);
AF_DCMotor front_right(2);

const int TRIG = 10;
const int ECHO = 7;

const int ROVER_SPEED = 100;
const int ROVER_TURN_SPEED = 100;

const int ENCODER_PIN_RIGHT = 9;
const int ENCODER_PIN_LEFT = A2;

volatile int right_counter = 0;
volatile int left_counter = 0;

double right_motor_setpoint, right_motor_input, right_motor_output;
PID right_motor_PID(&right_motor_input, &right_motor_output, &right_motor_setpoint, 2, 5, 1, DIRECT);

double left_motor_setpoint, left_motor_input, left_motor_output;
PID left_motor_PID(&left_motor_input, &left_motor_output, &left_motor_setpoint, 2, 5, 1, DIRECT);

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

  right_motor_input = analogRead(0);
  right_motor_setpoint = 25;
  right_motor_PID.SetMode(AUTOMATIC);

  left_motor_input = analogRead(1);
  left_motor_setpoint = 25;
  left_motor_PID.SetMode(AUTOMATIC);
}

void loop() {

  move(ping());

  right_motor_input = analogRead(0);
  right_motor_PID.Compute();
  analogWrite(3, right_motor_output);

  left_motor_input = analogRead(1);
  left_motor_PID.Compute();
  analogWrite(11, left_motor_output);
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
  
  Serial.print(cm);
  Serial.println(" cm");
  
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

