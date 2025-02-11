#include <Arduino.h>
#include <AccelStepper.h>

// -------------------------------------------------------------------------------------------------
//                                              Pin setup
//--------------------------------------------------------------------------------------------------
// Define sensors
  #define ESTOP_sensor_pin 2
  byte ESTOP_sensor_state = LOW;
  volatile bool ESTOP_pressed = false;
  #define J0_sensor_pin 10
  byte J0_sensor_state = HIGH;
  #define J1_sensor_pin 6
  byte J1_sensor_state = LOW;
  #define J2_sensor_pin 4
  byte J2_sensor_state = LOW;
  #define J3_sensor_pin 14
  byte J3_sensor_state = LOW;
  #define J4_sensor_pin 8
  byte J4_sensor_state = LOW;
  #define J5_sensor_pin 12
  byte J5_sensor_state = LOW;
  const int gripper_air_solenoid_pin = A0;
  byte gripper_air_solenoid_pin_state = HIGH;
  
// Define number of motors
  #define NUM_MOTORS 6

// Joint 0 pulley = 15 tooth --> base = 96 - no gearbox
  #define J0ENA 26
  #define J0DIR 27
  #define J0PUL 28

// Joint 1 gearbox = 20:1 - no pulley
  #define J1ENA 29
  #define J1DIR 30
  #define J1PUL 31

// Joint 2 - 42 tooth base - 38 tooth top - 20:1 gearbox
  #define J2ENA 32
  #define J2DIR 33
  #define J2PUL 34

// Joint 3 - 12 tooth pulley - 48 tooth shaft
  #define J3ENA 35
  #define J3DIR 36
  #define J3PUL 37

// Joint 4 - 12 tooth pulley - 48 tooth shaft 
  #define J4ENA 41
  #define J4DIR 42
  #define J4PUL 43

// Joint 5 - 10:1 gearbox
  #define J5ENA 38
  #define J5DIR 39
  #define J5PUL 40

// Define homing parameters
const float HOMING_SPEED       = 500.0;   // [steps/s]   Speed
const float HOMING_ACCEL       = 500.0;   // [steps/s^2] Acceleration
const long  HOMING_SEARCH_DIST = 200000;  // steps to move while searching

// Define homing functions
void homing_J0();
void homing_J1();
void homing_J2();
void homing_J3();
void homing_J4();
void homing_J5();

void ESTOP_pressed_interupt();

// Define stepper motor objects
AccelStepper steppers[NUM_MOTORS] = {
    AccelStepper(AccelStepper::DRIVER, J0PUL, J0DIR), // Motor 0
    AccelStepper(AccelStepper::DRIVER, J1PUL, J1DIR), // Motor 1
    AccelStepper(AccelStepper::DRIVER, J2PUL, J2DIR), // Motor 2
    AccelStepper(AccelStepper::DRIVER, J3PUL, J3DIR), // Motor 3
    AccelStepper(AccelStepper::DRIVER, J4PUL, J4DIR), // Motor 4
    AccelStepper(AccelStepper::DRIVER, J5PUL, J5DIR), // Motor 5
};

// Stepper motor parameters (customized per motor)
struct MotorParams {
    float steps_per_rev;   // Steps per revolution (without microstepping)
    float gear_ratio;      // Gear ratio (e.g., 10:1)
    float microstep;       // Microstepping factor (e.g., 16, 32, etc.)
    float max_speed;       // Max speed in steps/sec
    float accel;           // Acceleration in steps/sec²
};

// Define specific parameters for each motor
MotorParams motorParams[NUM_MOTORS] = {
    {200,    6.4, 16, 3000, 4000},  // Motor 0: 200 steps/rev
    {200,   20.0,  8, 3000, 4000},   // Motor 1: 200 steps/rev
    {200, 18.095,  8, 3000, 4000},    // Motor 2: 200 steps/rev
    {200,    4.0, 16, 2000, 2000},  // Motor 3: 200 steps/rev
    {200,    4.0, 16, 2000, 2000},   // Motor 4: 200 steps/rev
    {200,   10.0,  8, 2000, 2000}     // Motor 5: 200 steps/rev
};

// Store current position in steps
long current_steps[NUM_MOTORS] = {0, 0, 0, 0, 0, 0};

// Convert angle to steps
long angleToSteps(int motorIndex, float angle) {
    return (angle / 360.0) * motorParams[motorIndex].steps_per_rev *
           motorParams[motorIndex].gear_ratio * motorParams[motorIndex].microstep;
};

void moveMotorToAngle(int motorIndex, float targetAngle) {
    long targetSteps = angleToSteps(motorIndex, targetAngle);
    
    // Set speed and acceleration
    steppers[motorIndex].setMaxSpeed(motorParams[motorIndex].max_speed);
    steppers[motorIndex].setAcceleration(motorParams[motorIndex].accel);
    
    // Move to new position
    steppers[motorIndex].moveTo(targetSteps);
    current_steps[motorIndex] = targetSteps;
};

// Just for the fondue demo
//--------------------------------------------------------------------------------
float squarePath[4][NUM_MOTORS] = {
    {12, 12, -12, 0, 0, 0},  // Position 1
    {10, -12, 10, 0, 0, 0},  // Position 2
    {-10, -12, 10, 0, 0, 0},  // Position 3
    {-12, 12, -12, 0, 0, 0} // Position 4
};

int currentSquareIndex = 0;  // Tracks which corner of the square we are at

void moveAllMotorsToSquarePosition(int positionIndex) {
    for (int i = 0; i < NUM_MOTORS; i++) {
        moveMotorToAngle(i, squarePath[positionIndex][i]);
    }

    // Ensure all motors reach their target
    bool allMotorsArrived = false;
    while (!allMotorsArrived) {
        allMotorsArrived = true;  // Assume all motors reached the position
        for (int i = 0; i < NUM_MOTORS; i++) {
            steppers[i].run();
            if (steppers[i].distanceToGo() != 0) {
                allMotorsArrived = false;  // At least one motor is still moving
            }
        }
    }
}

// Define a starting position for the robotic arm
float startPosition[NUM_MOTORS] = {235, -50.0, 65.0, -90.0, -80, 90.0};  // Adjust values as needed

void moveArmToStartPosition() {
    Serial.println("Moving arm to start position...");

    for (int i = 0; i < NUM_MOTORS; i++) {
        moveMotorToAngle(i, startPosition[i]);
    }

    // Ensure all motors reach the start position before continuing
    bool allMotorsArrived = false;
    while (!allMotorsArrived) {
        allMotorsArrived = true;  // Assume all motors reached the position
        for (int i = 0; i < NUM_MOTORS; i++) {
            steppers[i].run();
            if (steppers[i].distanceToGo() != 0) {
                allMotorsArrived = false;  // At least one motor is still moving
            }
        }
    }

    Serial.println("Arm is now at start position.");
}


//---------------------------------------------------------------------------------


void setup() {
  Serial.begin(115200);
  Serial.println(motorParams[1].gear_ratio);

  // Enable lines as outputs, set LOW (active low) to enable drivers
  pinMode(J0ENA, OUTPUT); digitalWrite(J0ENA, LOW);
  pinMode(J1ENA, OUTPUT); digitalWrite(J1ENA, LOW);
  pinMode(J2ENA, OUTPUT); digitalWrite(J2ENA, LOW);
  pinMode(J3ENA, OUTPUT); digitalWrite(J3ENA, LOW);
  pinMode(J4ENA, OUTPUT); digitalWrite(J4ENA, LOW);
  pinMode(J5ENA, OUTPUT); digitalWrite(J5ENA, LOW);

  // Enable accessories
  pinMode(gripper_air_solenoid_pin, OUTPUT); digitalWrite(gripper_air_solenoid_pin, HIGH);

  // Define sensors as inputs
  pinMode(ESTOP_sensor_pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(ESTOP_sensor_pin),
                  ESTOP_pressed_interupt,
                  CHANGE);
  pinMode(J0_sensor_pin, INPUT);
  pinMode(J1_sensor_pin, INPUT);
  pinMode(J2_sensor_pin, INPUT);
  pinMode(J3_sensor_pin, INPUT);
  pinMode(J4_sensor_pin, INPUT);
  pinMode(J5_sensor_pin, INPUT);

  // Homing

  homing_J0();
  homing_J1();
  homing_J2();
  homing_J3();
  homing_J5();
  homing_J4();

  moveArmToStartPosition();
  // To grab spatula
  steppers[4].setCurrentPosition(0);
  steppers[2].setCurrentPosition(0);
  delay(1000);
  steppers[2].moveTo(1000);
  steppers[2].runToPosition();
  steppers[4].moveTo(3200);
  steppers[4].runToPosition();
  digitalWrite(gripper_air_solenoid_pin, LOW);
  delay(4000);
  digitalWrite(gripper_air_solenoid_pin, HIGH);
  delay(2000);
  steppers[4].moveTo(0);
  steppers[4].runToPosition();
  steppers[2].moveTo(0);
  steppers[2].runToPosition();

  // Set all to relative 0
  steppers[0].setCurrentPosition(0);
  steppers[1].setCurrentPosition(0);
  steppers[2].setCurrentPosition(0);
  steppers[3].setCurrentPosition(0);
  steppers[4].setCurrentPosition(0);
  steppers[5].setCurrentPosition(0);

}

void loop() {
  if (digitalRead(ESTOP_sensor_pin) == HIGH){
    moveAllMotorsToSquarePosition(currentSquareIndex);  // Move to the next square position
    currentSquareIndex = (currentSquareIndex + 1) % 4;  // Loop through 0 → 1 → 2 → 3 → 0
  }
}

// Homing functions

void homing_J0(){
  Serial.println("Starting homing routine for Joint 1");
  steppers[0].setMaxSpeed(HOMING_SPEED);
  steppers[0].setAcceleration(HOMING_ACCEL);

  steppers[0].setCurrentPosition(0);
  steppers[0].moveTo(700);
  steppers[0].runToPosition();
  steppers[0].moveTo(-HOMING_SEARCH_DIST);


  while (steppers[0].distanceToGo() != 0)
  {
    steppers[0].run();
    if(digitalRead(J0_sensor_pin) == LOW){
      steppers[0].stop();
      steppers[0].setCurrentPosition(0);
      Serial.println("Endstop triggered !!!👌");
      break;
    }

  }
}

void homing_J1(){
  Serial.println("Starting homing routine for Joint 2");
  steppers[1].setMaxSpeed(HOMING_SPEED);
  steppers[1].setAcceleration(HOMING_ACCEL);

  steppers[1].setCurrentPosition(0);
  steppers[1].moveTo(-500);
  steppers[1].runToPosition();

  steppers[1].moveTo(HOMING_SEARCH_DIST);


  while (steppers[1].distanceToGo() != 0)
  {
    steppers[1].run();
    if(digitalRead(J1_sensor_pin) == HIGH){
      steppers[1].stop();
      steppers[1].setCurrentPosition(0);
      Serial.println("Endstop triggered !!!👌");
      break;
    }

  }

}

void homing_J2(){
  Serial.println("Starting homing routine for Joint 3");
  steppers[2].setMaxSpeed(HOMING_SPEED);
  steppers[2].setAcceleration(HOMING_ACCEL);

  steppers[2].setCurrentPosition(0);
  steppers[2].moveTo(500);
  steppers[2].runToPosition();

  steppers[2].moveTo(-HOMING_SEARCH_DIST);


  while (steppers[2].distanceToGo() != 0)
  {
    steppers[2].run();
    if(digitalRead(J2_sensor_pin) == HIGH){
      steppers[2].stop();
      steppers[2].setCurrentPosition(0);
      Serial.println("Endstop triggered !!!👌");
      break;
    }

  }

}

void homing_J3(){
  Serial.println("Starting homing routine for Joint 4");
  steppers[3].setMaxSpeed(HOMING_SPEED);
  steppers[3].setAcceleration(HOMING_ACCEL);

  steppers[3].setCurrentPosition(0);
  steppers[3].moveTo(-700);
  steppers[3].runToPosition();

  steppers[3].moveTo(HOMING_SEARCH_DIST);


  while (steppers[3].distanceToGo() != 0)
  {
    steppers[3].run();
    if(digitalRead(J3_sensor_pin) == LOW){
      steppers[3].stop();
      steppers[3].setCurrentPosition(0);
      Serial.println("Endstop triggered !!!👌");
      break;
    }

  }

}

void homing_J4(){
  Serial.println("Starting homing routine for Joint 5");
  steppers[4].setMaxSpeed(HOMING_SPEED);
  steppers[4].setAcceleration(HOMING_ACCEL);

  steppers[4].setCurrentPosition(0);
  steppers[4].moveTo(-HOMING_SEARCH_DIST);


  while (steppers[4].distanceToGo() != 0)
  {
    steppers[4].run();
    if(digitalRead(J4_sensor_pin) == HIGH){
      steppers[4].stop();
      steppers[4].setCurrentPosition(0);
      Serial.println("Endstop triggered !!!👌");
      break;
    }

  }
  steppers[4].moveTo(4350);
  steppers[4].runToPosition();
  steppers[4].setCurrentPosition(0);

  // Move stepper 3 to home
  steppers[3].setMaxSpeed(HOMING_SPEED);
  steppers[3].setAcceleration(HOMING_ACCEL);
  steppers[3].moveTo(-1860);
  steppers[3].runToPosition();
  steppers[3].setCurrentPosition(0);
}

void homing_J5(){
  Serial.println("Starting homing routine for Joint 6");
  steppers[5].setMaxSpeed(HOMING_SPEED*3);
  steppers[5].setAcceleration(HOMING_ACCEL*3);

  steppers[5].setCurrentPosition(0);
  steppers[5].moveTo(-HOMING_SEARCH_DIST);


  while (steppers[5].distanceToGo() != 0)
  {
    steppers[5].run();
    if(digitalRead(J5_sensor_pin) == LOW){
      steppers[5].stop();
      steppers[5].setCurrentPosition(0);
      Serial.println("Endstop triggered !!!👌");
      break;
    }
  }
  steppers[5].moveTo(4000);
  steppers[5].runToPosition();
  steppers[5].setCurrentPosition(0);
}


void ESTOP_pressed_interupt(){
    ESTOP_pressed = true;
}
