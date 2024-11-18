#include <SoftwareSerial.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
// 108,70

#define enablePin 26
#define stepPin 27
#define dirPin 28
#define ms1Pin 29
#define ms2Pin 30

#define m2enablePin 31
#define m2stepPin 32
#define m2dirPin 33
#define m2ms1Pin 34
#define m2ms2Pin 35


#define RIGHT_LED 9
#define LEFT_LED 8

// Define the RX and TX pins connected to the HC-05
#define HC05_RX 10
#define HC05_TX 11
#define HC05_EN 12
#define HC05_ONOFF 13
#define BLUETOOTH_DELAY 10
#define MOTOR_ITERATIONS 500

#define BOARD_WIDTH 108
#define BOARD_HEIGHT 70
// #define STEPS_PER_MM 20
#define STEPS_PER_REV 200

#define STEPS_10CM 6130

// Create a SoftwareSerial object for the HC-05
SoftwareSerial bluetooth(HC05_RX, HC05_TX);

String msg;
AccelStepper motor1(1, stepPin, dirPin);
AccelStepper motor2(1, m2stepPin, m2dirPin);

AccelStepper* steppers[2] = {
  &motor1,
  &motor2
};

float markerX = BOARD_WIDTH/2;
float markerY = BOARD_HEIGHT/2;

void getPosition(int m1){
  Serial.print("Position is ");
  float position = (STEPS_10CM / 10);
  position = m1 / position;
  Serial.print(position);
  Serial.print("\n");
}

void setup() {
  pinMode(ms1Pin, OUTPUT);
  pinMode(ms2Pin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enablePin, OUTPUT);

  pinMode(m2ms1Pin, OUTPUT);
  pinMode(m2ms2Pin, OUTPUT);
  pinMode(m2stepPin, OUTPUT);
  pinMode(m2dirPin, OUTPUT);
  pinMode(m2enablePin, OUTPUT);

  // Set LEDs as output
  pinMode(LEFT_LED, OUTPUT);
  pinMode(RIGHT_LED, OUTPUT);

  // Initialize Bluetooth
  Serial.begin(9600);
  bluetooth.begin(9600);

  // Power on the HC-05
  pinMode(HC05_EN, OUTPUT);
  pinMode(HC05_ONOFF, OUTPUT);
  digitalWrite(HC05_ONOFF, HIGH);

  motor1.setMaxSpeed(7000);
  motor2.setMaxSpeed(7000);
  for(int i = 0; i < 2; i++){
    steppers[i]->setMaxSpeed(7000);
    steppers[i]->setSpeed(4000);
    steppers[i]->setAcceleration(300);
    steppers[i]->setCurrentPosition(0);
  }
}

bool isMovingLeft = false;
bool isMovingRight = false;
bool isMovingUp = false;
bool isMovingDown = false;
int count = 0;
int leftMoveCounter = 0;

void loop() {
  // motor1
  digitalWrite(ms1Pin, HIGH);
  digitalWrite(ms2Pin, HIGH);
  digitalWrite(enablePin, LOW);
  motor1.setSpeed(4000);

  // motor2
  digitalWrite(m2ms1Pin, HIGH);
  digitalWrite(m2ms2Pin, HIGH);
  digitalWrite(m2enablePin, LOW);
  motor2.setSpeed(4000);
  // motor1.runSpeed();

  // Read from the HC-05 and write to the Serial Monitor
  if(count == BLUETOOTH_DELAY){
    if (bluetooth.available()) {
      msg = bluetooth.readStringUntil('\n');
      msg.trim();
      Serial.println("Message received from Master: " + msg);
    }
    count = 0;
  }

  if (msg == "LEFT") {
    Serial.println("Moving both motors LEFT");
    digitalWrite(LEFT_LED, HIGH);
    digitalWrite(RIGHT_LED, LOW);
    isMovingRight = false;
    isMovingLeft = true;
    // Step both motors in the forward direction
  } 
  else if (msg == "RIGHT") {
    Serial.println("Moving both motors RIGHT");
    digitalWrite(LEFT_LED, LOW);
    digitalWrite(RIGHT_LED, HIGH);
    // digitalWrite(dirPin, HIGH);
    isMovingRight = true;
    isMovingLeft = false;
    // Step both motors in the backward direction
    // motor1.setSpeed(-4000);
    for(int i = 0; i < 2; i++)
      steppers[i]->setSpeed(-4000);
  } 
  else if(msg == "Down"){
    Serial.println("Moving Both motors down");
    digitalWrite(LEFT_LED, LOW);
    digitalWrite(RIGHT_LED, LOW);

    isMovingUp = false;
    isMovingRight = false;
    isMovingLeft = false;
    isMovingDown = true;

    steppers[0]->setSpeed(-4000);
    steppers[1]->setSpeed(4000);
  }
  else if(msg == "Up"){
    Serial.println("Moving motors up");
    digitalWrite(LEFT_LED, LOW);
    digitalWrite(RIGHT_LED, LOW);
    isMovingUp = true;
    isMovingRight = false;
    isMovingLeft = false;
    isMovingDown = false;
    steppers[0]->setSpeed(4000);
    steppers[1]->setSpeed(-4000);
  }
  else if(msg == "NEUTRAL"){
    // No valid command, stop the motors
    digitalWrite(LEFT_LED, LOW);
    digitalWrite(RIGHT_LED, LOW);
    isMovingRight = false;
    isMovingLeft = false;
    isMovingUp = false;
    isMovingDown = false;
  }

  if(isMovingLeft){
    for(int i = 0; i < MOTOR_ITERATIONS; i++){
        for(int j = 0; j < 2; j++)
          steppers[j]->runSpeed();
    }

  }
  else if(isMovingRight){
    for(int i = 0; i < MOTOR_ITERATIONS; i++)
      for(int j = 0; j < 2; j++)
        steppers[j]->runSpeed();
    Serial.println(steppers[0]->currentPosition());
    Serial.println(steppers[1]->currentPosition());

  }
  else if(isMovingDown){
    for(int i = 0; i < MOTOR_ITERATIONS; i++)
      for(int j = 0; j < 2; j++)
        steppers[j]->runSpeed();
  }
  else if(isMovingUp){
    for(int i = 0; i < MOTOR_ITERATIONS; i++)
      for(int j = 0; j < 2; j++)
        steppers[j]->runSpeed();

    
  }
count++;
// Serial.println(steppers[0]->currentPosition());
getPosition(steppers[0]->currentPosition());
}
