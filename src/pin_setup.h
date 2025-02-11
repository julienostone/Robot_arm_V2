#include <Arduino.h>

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
const float HOMING_SPEED       = 500.0;   // [steps/s]   Adjust as needed
const float HOMING_ACCEL       = 500.0;   // [steps/s^2] Acceleration
const long  HOMING_SEARCH_DIST = 200000;  // steps to move while searching (large enough)
