#include <SoftwareSerial.h>
#include <AccelStepper.h>

// Whiteboard dimensions (cm)
#define W 108
#define H 70

// Bluetooth configuration
#define HC05_RX 10       // Arduino RX pin connected to HC-05 TX
#define HC05_TX 11       // Arduino TX pin connected to HC-05 RX
#define HC05_EN 12
#define HC05_ONOFF 13
#define BLUETOOTH_DELAY 10
#define MOTOR_ITERATIONS 500

// Motor 1 (Left Motor) Pins
#define enablePin 26
#define stepPin 27
#define dirPin 28
#define ms1Pin 29
#define ms2Pin 30

// Motor 2 (Right Motor) Pins
#define m2enablePin 31
#define m2stepPin 32
#define m2dirPin 33
#define m2ms1Pin 34
#define m2ms2Pin 35

// LED Pins
#define RIGHT_LED 9
#define LEFT_LED 8

// Step-to-Length Conversion
const float steps_per_cm = 6000.0 / 10.0; // 6000 steps = 10 cm => 600 steps/cm
const float step_length = 1.0 / steps_per_cm; // cm per step (~0.0016667 cm/step)

// Movement Parameters
const float delta_x_per_step = 0.1; // cm per movement step (e.g., 0.1 cm)
const unsigned long movementInterval = 0; // time between movement steps in microseconds (100 ms)

// Correction Factors (Adjust based on calibration)
const float CORRECTION_FACTOR_LEFT = 1.2;  // Left motor speed adjustment
const float CORRECTION_FACTOR_RIGHT = 0.8;

// Position Tracking
float current_x = W / 2.0; // Starting at center X (cm)
float current_y = H / 2.0; // Starting at center Y (cm)

// Direction Flags
bool isMovingLeft = false;
bool isMovingRight = false;
bool isMovingUp = false; // Unused currently
bool isMovingDown = false; // Unused currently

// Timing Variables
// currently unused, but can be used in future update
unsigned long lastMoveTime = 0;

// create a SoftwareSerial object for the HC-05
SoftwareSerial bluetooth(HC05_RX, HC05_TX);

// create AccelStepper instances for both motors
AccelStepper motor1(AccelStepper::DRIVER, stepPin, dirPin);
AccelStepper motor2(AccelStepper::DRIVER, m2stepPin, m2dirPin);

// array of steppers for easier management
AccelStepper* steppers[2] = {
  &motor1,
  &motor2
};

// prototypes
void handleBluetooth();
void moveMarker(float delta_x);
void calculateCorrection(float delta_x, float &leftSpeed, float &rightSpeed);


void setup() {
  // init motor control pins
  pinMode(ms1Pin, OUTPUT);
  pinMode(ms2Pin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  
  pinMode(m2ms1Pin, OUTPUT);
  pinMode(m2ms2Pin, OUTPUT);
  pinMode(m2enablePin, OUTPUT);
  pinMode(m2stepPin, OUTPUT);
  pinMode(m2dirPin, OUTPUT);
  
  // init LEDs
  pinMode(LEFT_LED, OUTPUT);
  pinMode(RIGHT_LED, OUTPUT);
  
  // init Bluetooth
  Serial.begin(9600);
  bluetooth.begin(9600);
  
  // power on the HC-05
  pinMode(HC05_EN, OUTPUT);
  pinMode(HC05_ONOFF, OUTPUT);
  digitalWrite(HC05_ONOFF, HIGH);
  
  // configure microstepping (set 1/16 microstepping)
  digitalWrite(ms1Pin, HIGH);
  digitalWrite(ms2Pin, HIGH);
  
  digitalWrite(m2ms1Pin, HIGH);
  digitalWrite(m2ms2Pin, HIGH);
  
  // enable motors (active LOW)
  digitalWrite(enablePin, LOW);
  digitalWrite(m2enablePin, LOW);
  
  // Initialize steppers
  for(int i = 0; i < 2; i++){
    steppers[i]->setCurrentPosition(0);
    steppers[i]->setSpeed(0);
    steppers[i]->setMaxSpeed(5000); // change if necessary
  }
  
  // initial Belt Length Calculation for logging
  float L1_initial = sqrt(pow(current_x, 2) + pow(H - current_y, 2)); // Left belt
  float L2_initial = sqrt(pow(W - current_x, 2) + pow(H - current_y, 2)); // Right belt
  
  // debug
  // Serial.println("=== Marker Movement Simulation Started ===");
  // Serial.print("Initial Position: (");
  // Serial.print(current_x);
  // Serial.print(" cm, ");
  // Serial.print(current_y);
  // Serial.println(" cm)");
}
String msg;
int count = 0;
void loop() {
  // Handle incoming Bluetooth messages
  handleBluetooth();
  
  // check if it's time to perform the next movement step
  unsigned long currentTime = micros();
  if ((isMovingLeft || isMovingRight) && (currentTime - lastMoveTime >= movementInterval)) {
    lastMoveTime = currentTime;
    
    // determine movement direction
    float delta_x = 0.0;
    if (isMovingLeft) {
      delta_x = -delta_x_per_step; // Move left
    }
    else if (isMovingRight) {
      delta_x = delta_x_per_step; // Move right
    }
    
    // execute movement with correction
    if (delta_x != 0.0) {
      moveMarker(delta_x);
    }
  }

  if(isMovingUp){
    steppers[0]->setSpeed(5000);
    steppers[1]->setSpeed(-5000);
  }
  if(isMovingDown){
    steppers[0]->setSpeed(-5000);
    steppers[1]->setSpeed(5000);
  }
  
  // continuously run motors at set speeds
  for(int j = 0; j < 200; j++)
    for(int i = 0; i < 2; i++){
      steppers[i]->runSpeed();
    }
}

// bluetooth function handling
void handleBluetooth(){

  // count to enable that the system does not repeatedly check for a new message every other millisecond
  // reduces power load
  if(count % 10 == 0){
    if (bluetooth.available()) {
      msg = bluetooth.readStringUntil('\n');
      msg.trim();
      // Serial.println("Message received from Master: " + msg);
    }

    if (msg == "LEFT") {
      // debug
      // Serial.println("Moving both motors LEFT");
      digitalWrite(LEFT_LED, HIGH);
      digitalWrite(RIGHT_LED, LOW);
      isMovingRight = false;
      isMovingLeft = true;
      // Reset message to prevent repeated actions
      msg = "";
    } 
    else if (msg == "RIGHT") {
      // debug
      // Serial.println("Moving both motors RIGHT");
      digitalWrite(LEFT_LED, LOW);
      digitalWrite(RIGHT_LED, HIGH);
      isMovingRight = true;
      isMovingLeft = false;
      // Reset message to prevent repeated actions
      msg = "";
    } 
    else if(msg == "Down"){
      // debug
      // Serial.println("Moving Both motors down");
      digitalWrite(LEFT_LED, LOW);
      digitalWrite(RIGHT_LED, LOW);
      
      isMovingLeft = false;
      isMovingRight = false;
      isMovingDown = true; // Implement movement down if needed
      isMovingUp = false;
      // Reset message
      msg = "";
    }
    else if(msg == "Up"){
      // debug
      // Serial.println("Moving motors up");
      digitalWrite(LEFT_LED, LOW);
      digitalWrite(RIGHT_LED, LOW);
      
      isMovingLeft = false;
      isMovingRight = false;
      isMovingUp = true;
      isMovingDown = false;
      // Reset message
      msg = "";
    }
    else if(msg == "NEUTRAL"){
      // No valid command, stop the motors

      // debug
      // Serial.println("Stopping all movements");
      digitalWrite(LEFT_LED, LOW);
      digitalWrite(RIGHT_LED, LOW);
      isMovingRight = false;
      isMovingLeft = false;
      isMovingUp = false;
      isMovingDown = false;
      
      // stop motors
      steppers[0]->setSpeed(0);
      steppers[1]->setSpeed(0);
      msg = "";
    }
    else {
      // reset message to ensure redundancy doesn't reduce power efficiency and latency
      msg = "";
    }
  }
  count++;
}

// function to move the marker horizontally with correction
void moveMarker(float delta_x) {
  
  // Calculate new desired position
  float x_new = current_x + delta_x;
  float y_new = current_y; // Y remains constant
  
  // Ensure x_new stays within whiteboard boundaries
  x_new = constrain(x_new, 0.0, W);
  
  // Calculate new belt lengths
  float L1_new = sqrt(pow(x_new, 2) + pow(H - y_new, 2)); // Left belt
  float L2_new = sqrt(pow(W - x_new, 2) + pow(H - y_new, 2)); // Right belt
  
  // Calculate current belt lengths
  float L1_current_calc = sqrt(pow(current_x, 2) + pow(H - y_new, 2));
  float L2_current_calc = sqrt(pow(W - current_x, 2) + pow(H - y_new, 2));
  
  // change in belt length necessary to move to position calculations
  float delta_L1 = L1_new - L1_current_calc; // ΔL1
  float delta_L2 = L2_new - L2_current_calc; // ΔL2
  
  // Debugging information on delta changes and belt length calculations
  // Serial.print("Movement: Δx = ");
  // Serial.print(delta_x);
  // Serial.print(" cm, ΔL1 = ");
  // Serial.print(delta_L1);
  // Serial.print(" cm, ΔL2 = ");
  // Serial.print(delta_L2);
  // Serial.print(" cm");
  
  // calculate motor speeds with correction
  float leftSpeed = 0.0;
  float rightSpeed = 0.0;
  calculateCorrection(delta_x, leftSpeed, rightSpeed);
  
  // Debugging purposes
  // Serial.print(" => Left Speed: ");
  // Serial.print(leftSpeed);
  // Serial.print(", Right Speed: ");
  // Serial.println(rightSpeed);
  
  // Set motor speeds using steppers array
  steppers[0]->setSpeed(leftSpeed);
  steppers[1]->setSpeed(rightSpeed);
  
  // Update current position
  current_x = x_new;
  
  // delay for smooth operation: Seems to slow movement rather than improve smoothness
  // delay(5);
}

// function to calculate corrected motor speeds
void calculateCorrection(float delta_x, float &leftSpeed, float &rightSpeed) {
  
  // define base speed (steps per second)
  const float baseSpeed = 5000.0; // for desired speed
  
  if (delta_x < 0) { // moving Left
    leftSpeed = -baseSpeed * CORRECTION_FACTOR_LEFT;   // Negative for left
    rightSpeed = -baseSpeed * CORRECTION_FACTOR_RIGHT; // Negative for left
  }
  else if (delta_x > 0) { // Moving Right
    leftSpeed = baseSpeed * CORRECTION_FACTOR_LEFT;    // Positive for right
    rightSpeed = baseSpeed * CORRECTION_FACTOR_RIGHT;  // Positive for right
  }
  else { // no Movement
    leftSpeed = 0.0;
    rightSpeed = 0.0;
  }
}
