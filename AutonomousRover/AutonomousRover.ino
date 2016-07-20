#include <AFMotor.h>
#include <PID_v1.h>
#include <TimerOne.h>

// Initialize the two DC wheel motors
AF_DCMotor front_left(1);
AF_DCMotor front_right(2);

// Set to true to print debug messages to the serial line
boolean debug = true;

// Pins for the wheel encoders
const int ENCODER_PIN_RIGHT = 9;
const int ENCODER_PIN_LEFT = A2;

// Pins for setting the direction on the two motors
const int DIRECTION_PIN_RIGHT = 12;
const int DIRECTION_PIN_LEFT = 13;

// PWM pins for speed control on the two motors
const int SPEED_PIN_RIGHT = 3;
const int SPEED_PIN_LEFT = 11;

// Pins for the ultrasonic sensor
const int TRIG = 10;
const int ECHO = 2;

// Constants for motor speed during different behaviors
const int SPEED_CRUISING = 10;
const int SPEED_HALT = 0;
const int SPEED_TURN = 10;

// Counters for the wheel encoders
volatile int right_counter = 0, last_right_counter = 0;
volatile int left_counter = 0, last_left_counter = 0;

// Timer variables
unsigned long encoderTimer = 0L;
unsigned long debugTimer = 0L;
unsigned long stopTimer = 0L;

// Used by the ultrasonic echo interrupt service routines
volatile long echo_end = 0L;
volatile long echo_start = 0L;
volatile long echo_duration = 0L;
volatile long echo_distance = 0;

// Definition of different behaviors
const int EXPLORE = 1;
const int COLLISION_AVOIDANCE = 2;
const int STOP = 3;

// Set the default behavior to explore mode
int behavior = EXPLORE;

// PID control for the right motor
double right_motor_setpoint, right_motor_input, right_motor_output;
PID right_motor_PID(&right_motor_input, &right_motor_output, &right_motor_setpoint, 2, 5, 1, DIRECT);

// PID control for the left motor
double left_motor_setpoint, left_motor_input, left_motor_output;
PID left_motor_PID(&left_motor_input, &left_motor_output, &left_motor_setpoint, 2, 5, 1, DIRECT);

void setup()
{

  Serial.begin(9600);

  // Print usage information to the Serial line
  usage();

  pinMode (TRIG, OUTPUT);
  pinMode (ECHO, INPUT);

  front_left.setSpeed(1);
  front_left.run(RELEASE);
  digitalWrite(DIRECTION_PIN_LEFT, LOW);

  front_right.setSpeed(1);
  front_right.run(RELEASE);
  digitalWrite(DIRECTION_PIN_RIGHT, LOW);

  // Enable Pin Change Interrupts for the encoder pins
  pciSetup(ENCODER_PIN_RIGHT);
  pciSetup(ENCODER_PIN_LEFT);

  // Setup the PID control parameters for the right motor
  right_motor_setpoint = SPEED_CRUISING;
  right_motor_PID.SetMode(AUTOMATIC);
  right_motor_PID.SetOutputLimits(0, 1024);

  // Setup the PID control parameters for the left motor
  left_motor_setpoint = SPEED_CRUISING;
  left_motor_PID.SetMode(AUTOMATIC);
  left_motor_PID.SetOutputLimits(0, 1024);

  Timer1.initialize(50000);
  Timer1.attachInterrupt(timerCallback);

  // Setup the external interrupt to capture the ultrasonic echo
  attachInterrupt(digitalPinToInterrupt(ECHO), readEcho, CHANGE);
}

void loop()
{

  // Behavior state machine
  if (behavior == EXPLORE || behavior == COLLISION_AVOIDANCE) {

    // Set the motors to move forward
    front_left.run(FORWARD);
    front_right.run(FORWARD);

    // Use the PID control to set the speed on the right and left motors
    // to match the setpoint
    
    right_motor_input = last_right_counter;
    while (!right_motor_PID.Compute());
    digitalWrite(SPEED_PIN_RIGHT, right_motor_output);
  
    left_motor_input = last_left_counter;
    while (!left_motor_PID.Compute());
    digitalWrite(SPEED_PIN_LEFT, left_motor_output);

    // Collision avoidance
    if (echo_distance < 30) {
      right_motor_setpoint = SPEED_TURN;
      left_motor_setpoint = SPEED_HALT;
    } else {
      right_motor_setpoint = SPEED_CRUISING;
      left_motor_setpoint = SPEED_CRUISING;
    }
    
    if (debug) {
      Serial.print("Echo distance: ");
      Serial.println(echo_distance);
  
      Serial.print("Encoder Right: ");
      Serial.print(last_right_counter);
      Serial.print(" Left: ");
      Serial.println(last_left_counter);
  
      Serial.print("Motor Right Input: ");
      Serial.print(right_motor_input);
      Serial.print(" Output: ");
      Serial.println(right_motor_output);
  
      Serial.print("Motor Left Input: ");
      Serial.print(left_motor_input);
      Serial.print(" Output: ");
      Serial.println(left_motor_output);
    }
  } else if (behavior == STOP) {
    digitalWrite(SPEED_PIN_RIGHT, LOW);
    digitalWrite(SPEED_PIN_LEFT, LOW);
    
    if (millis() - stopTimer >= 1000L) {
      behavior = EXPLORE;
    }
  }

  // Process serial input
  // WARNING: may slow the main loop, so exercise restraint
  if (Serial.available()) {
    processSerialInput();
  }
}

// Print info on commands and usage on the serial line
void usage()
{
  Serial.println("Autonomous Rover");
  Serial.println("Available commands:");
  Serial.println("D: toggle debug messages");
}

// Process input on the serial line
void processSerialInput()
{
  char cmd = Serial.read();

  switch (cmd) {
    case 'D':
      debug = !debug;
      Serial.println("Toggling debug mode");
      break;
  }

  // Ignore everything till the LF
  while (Serial.read() != 10);
}

// Create the ping waveform for the ultrasonic signal and capture the encoder
// values on a perodic basis
void timerCallback()
{

  // The pulse to be sent to the ultrasonic sensor
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  // Capture the encoder counter values and reset the counter.
  // This is used for PID control on the motors
  last_right_counter = right_counter;
  last_left_counter = left_counter;
  right_counter = 0;
  left_counter = 0;
}

// Read the echo from the ping signal and capture the duration
// of the response and convert it to a distance value from a
// potential point of collision
void readEcho()
{
  switch (digitalRead(ECHO))
  {
    case HIGH:
      echo_end = 0;
      echo_start = micros();
      break;

    case LOW:
      echo_end = micros();
      echo_duration = echo_end - echo_start;
      break;
  }

  echo_distance = echo_duration / 29 / 2;
}

// Enable the Pin Change Interrupt for the specified pin
void pciSetup(byte pin)
{
    *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
    PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
    PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}

// Interrupt service routine for the right wheel encoder
ISR (PCINT0_vect)
{
    right_counter++;
}
 
// Interrupt service routine for the left wheel encoder
ISR (PCINT1_vect)
{    
    left_counter++;
}

