/*
  June 18 2024
  135 x 150 mm Robot
  3 Sensors with basic alg
  with the use of ultrasonic sensor
  ESP32 WROOM used
*/

/*** Include Files ************************************************************/
#include <ArduinoJson.h>        // Required for JSON deserialization
#include <ESP32Servo.h>              // Required for the Servo library APIs
#include <EEPROM.h>             // Required for flash EEPROM emulation to store OPTO calibration values
#include <HCSR04.h>

/*** Symbolic Constants used in this module ***********************************/

// Digital I/O Pin Function Assignments
#define USER_LED_PIN 2         // OUTPUT - connected to USER LED (D1) 
#define USER_SWITCH_PIN 21      // INPUT - connected to USER SWITCH (S2)
#define LEFT_MOTOR_PWM_PIN 18    // OUTPUT - connected to motor controller "S2" input ("Left" Motor)
#define RIGHT_MOTOR_PWM_PIN 19   // OUTPUT - connected to motor controller "S1" input ("Right" Motor)
#define HCSR04_TRIGGER_PIN 5   // DIGITAL OUTPUT - connected to HCSR04 Trigger pin
#define HCSR04_ECHO_PIN 17      // DIGITAL INPUT - connected to HCSR04 Echo pin

// Analog Input Pin Definitions
//#define POT_INPUT_PIN A2      // Potentiometer shares AIN2 with LEFT_OPTO_PIN and must be disconnected for this sketch. 
#define LEFT_OPTO_PIN 27        // Left opto connected to AIN2
#define MIDDLE_OPTO_PIN 26      // Middle opto connected to AIN1
#define RIGHT_OPTO_PIN 25       // Right opto connected to AIN0

// Optosensor sampling intervals
#define OPTO_SAMPLE_INTERVAL 1	        // sample interval (in mS) for sampling the sensors
#define OPTO_CALIBRATE_INTERVAL 10000   // averaging time for opto calibration (in mS)

// Ultrasonic collision sensor definitions
#define COLLISION_SAMPLE_INTERVAL 50   // sample interval for collision sensor
#define COLLISION_DETECT_THRESHOLD 10.0 // collision distance (cm) threshold, written as a float constant

// Servo Motor Default Speed Settings (Straight Motion)
#define LEFT_SERVO_DEFAULT_STRAIGHT 17     
#define RIGHT_SERVO_DEFAULT_STRAIGHT 17    
#define LSERVO_UPPER_SPEED_LIMIT (LEFT_SERVO_DEFAULT_STRAIGHT + 16)
#define LSERVO_LOWER_SPEED_LIMIT (LEFT_SERVO_DEFAULT_STRAIGHT - 15)
#define RSERVO_UPPER_SPEED_LIMIT (RIGHT_SERVO_DEFAULT_STRAIGHT + 16)
#define RSERVO_LOWER_SPEED_LIMIT (RIGHT_SERVO_DEFAULT_STRAIGHT - 15)

/*** Global Variable Declarations *********************************************/

// switch variables
int switchLevelCurrent;   
int switchLevelPrevious;

// potentiometer raw measurement  
int potValue;

// optosensor sampling interval variables
unsigned long currentOptoSampleTime;  // Used for controlling opto sample rate
unsigned long previousOptoSampleTime = 0;

// collision sensor sampling interval variables
unsigned long collisionDistanceCurrentSampleTime;  // Used for controlling collision sensor sample rate
unsigned long collisionDistancePreviousSampleTime = 0;
const long collisionDistanceSampleInterval = COLLISION_SAMPLE_INTERVAL;

typedef struct{
  int isCalibrated;
  int left_opto_trip;
  int middle_opto_trip;
  int right_opto_trip;
} OPTO_CAL;

// define some robot states
enum ROBOT_STATE {CALIBRATE_WHITE=0, CALIBRATE_BLACK, IDLE, SEEK_START, RUN1, RUN2, RUN3, RUN4};

// Adafruit "monitor" group variables
typedef struct{
  int left_opto;                        // PA10/A3: Left Opto Sensor Signal
  int middle_opto;                      // PA11/A2: Middle Opto Sensor Signal
  int right_opto;                       // PA02/A0: Right Opto Sensor Signal
  OPTO_CAL optoCal;                     // create a structure to store runtime optosensor trip values
  enum ROBOT_STATE robotState = IDLE;   // declare & initialize a variable that holds the current state
  float collisionDistance;              // collision distance measurement from HCSR04
} AIO_MONITOR_FEEDS;

AIO_MONITOR_FEEDS aioMonitorFeeds;

// variables that control how often "aioMonitorFeeds" are published to the broker
unsigned long aioMonitorFeedsCurrentSampleTime;
unsigned long aioMonitorFeedsPreviousSampleTime;
const long aioMonitorFeedsSampleInterval = 2000;   // "aioMonitorFeeds" update interval (in mS)

// declare status message string to be published to Node-Red dashboard
String robotMessage;

// define the robot state transition events
typedef enum {evNone=0, evMCUReset, evPushButton, evRemotePushButton, evTimer, evFoundTee, evLost, evCollisionDetect} EVENT;

// declare and initialize a variable that holds events
EVENT event = evNone;

// declare Servo objects
Servo lServo;             // LEFT motor servo object
Servo rServo;             // RIGHT motor servo object

// initial torque (speed) parameter for motor functions
int g_lServoSpeed = 0;
int g_rServoSpeed = 0; 

// user led variables
unsigned long userLedCurrentSampleTime, userLedPrevSampleTime;
const long userLedSampleInterval = 100;   // user led update interval (in mS)
int userLedOutputValue[8][10] = {         // incorporate a flashing pattern for all ROBOT STATES
  {1, 0, 0, 0, 0, 0, 0, 0, 0, 0},         // CALIBRATE_WHITE
  {1, 0, 1, 0, 0, 0, 0, 0, 0, 0},         // CALIBRATE_BLACK
  {1, 0, 1, 0, 1, 0, 0, 0, 0, 0},         // IDLE
  {1, 0, 1, 0, 1, 0, 1, 0, 0, 0},         // SEEK_START
  {1, 0, 1, 0, 1, 0, 1, 0, 1, 0},         // RUN1
  {1, 0, 1, 0, 1, 0, 1, 0, 1, 0},         // RUN2
  {1, 0, 1, 0, 1, 0, 1, 0, 1, 0},         // RUN3
  {1, 0, 1, 0, 1, 0, 1, 0, 1, 0}          // RUN4
};
int userLedStateIndex;                    // row_ptr: current ROBOT_STATE
int userLedValueIndex;                    // col_ptr: current LED value

// instantiate an UltrasonicDistanceSensor object
const byte triggerPin = HCSR04_TRIGGER_PIN;
const byte echoPin = HCSR04_ECHO_PIN;
UltraSonicDistanceSensor distanceSensor(triggerPin, echoPin);

/*** Local Function Prototypes ************************************************/

// setup() & Loop() functions
void Initialize(void);          // Initialize peripherals, inputs/outputs and variables
void RunningTheFairway(void);   // Main function which implements robot state machine

// robot state behavior functions
void RobotWait(void);           // Wait for pushbutton event to begin the challenge
void RobotFindTee(void);        // Seek the starting Tee
void RobotFollowLine(void);     // Follow the line
void RobotTurnAround(void);     // Turn around at the Tee
void RobotHalt(void);           // Halt in place

// robot motor control functions
void RobotStop(void);           // stop the motors
void RobotForward(int lServoSpeed, int rServoSpeed); // drive both motors forward at desired speed
void RobotReverse(int lServoSpeed, int rServoSpeed); // drive both motors in reverse at desired speed
void RobotRotateLeft(int lServoSpeed, int rServoSpeed);// rotate robot sharp left at desired speed
void RobotRotateRight(int lServoSpeed, int rServoSpeed);// rotate robot sharp right at desired speed

// switch functions
void SwitchTasks(void);         // Sample/update switch S1 state variables                        
int SwitchWasPressed(void);     // Detect if switch S1 was pressed (push-release)

// optosensor functions                        
void OptoSampleInputs(void);    // Sample the opto-sensors inputs
int OptoLineDetect(void);       // Determines which opto sensors are over a line
void OptoCalibrate(void);       // Determine the optiomal trip threshold

// collision detection function
int CollisionDetect(void);      // Determine whether robot is within a certain distance from a target

// User led functions
void UserLedTasks(void);        // Update the USER LED based on the current robot state

// serialization functions for publish ("monitor") variables
String aioMonitorFeedsSerialize(void);   // serialize "aioMonitorFeeds" data structure for publish

// deserialization functions for subscribe ("control") variables
void eventDeserialize(void);      // De-serialize "event" JSON message, save value into "event" variable

/*** setup() & loop() Functions ***********************************************/

void setup() {
  // perform all one-time setup initialization
  Initialize();
}

void loop() {
  
  // Super-loop; cooperating tasks, no blocking code in any function!        

  // Update collision distance measurement at a defined interval
  collisionDistanceCurrentSampleTime = millis();
  if ((collisionDistanceCurrentSampleTime - collisionDistancePreviousSampleTime) >= collisionDistanceSampleInterval) {
    collisionDistancePreviousSampleTime = collisionDistanceCurrentSampleTime;
    // make a distance measurement (~4ms blocking delay)
    aioMonitorFeeds.collisionDistance = distanceSensor.measureDistanceCm();
  }

  // Update all "aioMonitorFeeds" values at a defined interval
  aioMonitorFeedsCurrentSampleTime = millis();
  if ((aioMonitorFeedsCurrentSampleTime - aioMonitorFeedsPreviousSampleTime) >= aioMonitorFeedsSampleInterval) {
    aioMonitorFeedsPreviousSampleTime = aioMonitorFeedsCurrentSampleTime;
    // send current values to the Dashboard
    Serial.println(aioMonitorFeedsSerialize());
  }
  
  // Line-following state machine tasks
  RunningTheFairway();      
}

/*** Local Function Definitions  *********************************************/

/*******************************************************************************
 * Function:        void Initialize(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Initializes the microcontroller, the peripherals 
 *                  used in the application and any global variables 
 *                  used by multiple functions.
 *
 * Note:            None
 ******************************************************************************/

void Initialize(void){
  
  // Initialize serial communications at 115200 bits per second
  Serial.begin(9600);
  Serial.println("RPI-Pico Robotics Workshop 2 (Lab 2. \"Running the Fairway\" Demo)\r\n");
  
  // Initialize digital I/O pins
  
  // USER LED (D1)
  pinMode(USER_LED_PIN, OUTPUT);  	                  // set digital pin as output
  
  // USER SWITCH (S2)
  pinMode(USER_SWITCH_PIN, INPUT_PULLUP);                    // set digital pin as input

  // LEFT Motor Servo PWM control output
  
  lServo.attach(LEFT_MOTOR_PWM_PIN);                  // Initialize LEFT MOTOR PWM Digital Control Output
  lServo.write(90);                                   // Motor state = STOP
        
  // RIGHT Motor Servo PWM control output
  
  rServo.attach(RIGHT_MOTOR_PWM_PIN);                 // Initialize RIGHT MOTOR PWM Digital Control Output
  rServo.write(90);                                   // Motor state = STOP

  // Servo Angle parameter vs Motor Torque/Direction
  //
  //  0   = full speed in one direction,
  //  90  = no movement
  //  180 = full speed in the other direction

  // Initiate opto-sensor calibration if user switch pressed on reset, or if not already calibrated
  EEPROM.begin(512);
  EEPROM.get(0, aioMonitorFeeds.optoCal);
  if((0 == aioMonitorFeeds.optoCal.isCalibrated) || (0 == digitalRead(USER_SWITCH_PIN))){
    // Wait for user to release the button
    while(0 == digitalRead(USER_SWITCH_PIN));
    Serial.println("OptoSensor Calibration routine triggered");
    // Clear EEPROM by writing a 0 to all 512 bytes
    for (int i = 0; i < 512; i++) {
      EEPROM.write(i, 0);
    }
    aioMonitorFeeds.robotState = CALIBRATE_WHITE;
    OptoCalibrate();
    // Write calibration values to EEPROM (Flash memory)
    EEPROM.put(0, aioMonitorFeeds.optoCal);
  }
  EEPROM.commit();

  // Set robot state
  aioMonitorFeeds.robotState = IDLE;

  switchLevelCurrent = digitalRead(USER_SWITCH_PIN);  // initialize switch level

  // Initialize collision distance measurement
  aioMonitorFeeds.collisionDistance = distanceSensor.measureDistanceCm();
  
  // send initial values to the Serial Monitor
  Serial.println(aioMonitorFeedsSerialize());
}

/*******************************************************************************
 * Function:        void OptoCalibrate(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Computes n-point average of White, then Black readings
 *                  Then computes/saves the mid-point values into variables:
 *                  optoCal.left_opto_trip,
 *                  optoCal.middle_opto_trip, and
 *                  optoCal.right_opto_trip
 *
 * Note:            None
 ******************************************************************************/

void OptoCalibrate(void) {

  int left_opto_accumulator, middle_opto_accumulator, right_opto_accumulator;
  int left_opto_white, middle_opto_white, right_opto_white;
  int left_opto_black, middle_opto_black, right_opto_black;
  int sample_counter;
  int num_samples = OPTO_CALIBRATE_INTERVAL/OPTO_SAMPLE_INTERVAL;

  sample_counter = 0;
  left_opto_accumulator = 0;
  middle_opto_accumulator = 0;
  right_opto_accumulator = 0;

  while(aioMonitorFeeds.robotState != IDLE){
    UserLedTasks();
    switch(aioMonitorFeeds.robotState){
      case CALIBRATE_WHITE:
        if(sample_counter <= num_samples){
          currentOptoSampleTime = millis();
          if((currentOptoSampleTime - previousOptoSampleTime) >= OPTO_SAMPLE_INTERVAL){
		        previousOptoSampleTime = currentOptoSampleTime;
            left_opto_accumulator += analogRead(LEFT_OPTO_PIN);
            middle_opto_accumulator += analogRead(MIDDLE_OPTO_PIN);
            right_opto_accumulator += analogRead(RIGHT_OPTO_PIN);
            sample_counter++;     
          }
        }
        else{
          left_opto_white = left_opto_accumulator/num_samples;
          middle_opto_white = middle_opto_accumulator/num_samples;
          right_opto_white = right_opto_accumulator/num_samples;
          Serial.print("Left Opto White: ");
          Serial.print(left_opto_white);
          Serial.print(" Middle Opto White: ");
          Serial.print(middle_opto_white);
          Serial.print(" Right Opto White: ");
          Serial.println(right_opto_white);
          sample_counter = 0;
          left_opto_accumulator = 0;
          middle_opto_accumulator = 0;
          right_opto_accumulator = 0;
          aioMonitorFeeds.robotState = CALIBRATE_BLACK;
        }
        break;
      case CALIBRATE_BLACK:
        if(sample_counter <= num_samples){
          currentOptoSampleTime = millis();
          if((currentOptoSampleTime - previousOptoSampleTime) >= OPTO_SAMPLE_INTERVAL){
		        previousOptoSampleTime = currentOptoSampleTime;
            left_opto_accumulator += analogRead(LEFT_OPTO_PIN);
            middle_opto_accumulator += analogRead(MIDDLE_OPTO_PIN);
            right_opto_accumulator += analogRead(RIGHT_OPTO_PIN);
            sample_counter++;
          }  
        }
        else{
          left_opto_black = left_opto_accumulator/num_samples;
          middle_opto_black = middle_opto_accumulator/num_samples;
          right_opto_black = right_opto_accumulator/num_samples;
          Serial.print("Left Opto Black: ");
          Serial.print(left_opto_black);
          Serial.print(" Middle Opto Black: ");
          Serial.print(middle_opto_black);
          Serial.print(" Right Opto Black: ");
          Serial.println(right_opto_black);
          aioMonitorFeeds.optoCal.left_opto_trip = (left_opto_white + left_opto_black)/2;
          aioMonitorFeeds.optoCal.middle_opto_trip = (middle_opto_white + middle_opto_black)/2;
          aioMonitorFeeds.optoCal.right_opto_trip = (right_opto_white + right_opto_black)/2;
          Serial.print("Left Opto Trip: ");
          Serial.print(aioMonitorFeeds.optoCal.left_opto_trip);
          Serial.print(" Middle Opto Trip: ");
          Serial.print(aioMonitorFeeds.optoCal.middle_opto_trip);
          Serial.print(" Right Opto Trip: ");
          Serial.println(aioMonitorFeeds.optoCal.right_opto_trip);
          aioMonitorFeeds.optoCal.isCalibrated = 1;
          aioMonitorFeeds.robotState = IDLE;
          digitalWrite(USER_LED_PIN, 0);
        }
        break;
    default:
      aioMonitorFeeds.robotState = IDLE;
      digitalWrite(USER_LED_PIN, 0);
      break;    
    }
  }
}

/*******************************************************************************
 * Function:        void UserLedTasks(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Updates User LED state every 100mS, based on ROBOT_STATE
 *                  and the defined flashing pattern table above
 *
 * Note:            None
 ******************************************************************************/

void UserLedTasks(void) {
  userLedCurrentSampleTime = millis();         // get the current time

  // if 100ms has elapsed, update the user LED value per the flashing table and ROBOT application state
  if ((userLedCurrentSampleTime - userLedPrevSampleTime) >= userLedSampleInterval) {
    userLedPrevSampleTime = userLedCurrentSampleTime;
    if (userLedValueIndex > 9) {
      userLedValueIndex = 0;                   // reset LED value pointer to beginning of row if required
    }
    // Write the current value of the LED
    switch(aioMonitorFeeds.robotState){
      case CALIBRATE_WHITE:
        digitalWrite(USER_LED_PIN, userLedOutputValue[0][userLedValueIndex++]);
        break;
      case CALIBRATE_BLACK:
        digitalWrite(USER_LED_PIN, userLedOutputValue[1][userLedValueIndex++]);
        break;
      case IDLE:
        digitalWrite(USER_LED_PIN, userLedOutputValue[2][userLedValueIndex++]);
        break;
      case SEEK_START:
        digitalWrite(USER_LED_PIN, userLedOutputValue[3][userLedValueIndex++]);
        break;
      case RUN1:
        digitalWrite(USER_LED_PIN, userLedOutputValue[4][userLedValueIndex++]);
        break;
      case RUN2:
        digitalWrite(USER_LED_PIN, userLedOutputValue[5][userLedValueIndex++]);
        break;
      case RUN3:
        digitalWrite(USER_LED_PIN, userLedOutputValue[6][userLedValueIndex++]);
        break;
      case RUN4:
        digitalWrite(USER_LED_PIN, userLedOutputValue[7][userLedValueIndex++]);
        break;
      default:
        break; 
    }
  }
} 

/*******************************************************************************
 * Function:        void SwitchTasks(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Samples/Saves switch level
 *
 * Note:            None
 ******************************************************************************/

void SwitchTasks(void){
  switchLevelPrevious = switchLevelCurrent;         // save previous level
  switchLevelCurrent = digitalRead(USER_SWITCH_PIN);// read current level
}

/*******************************************************************************
 * Function:        int SwitchWasPressed(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          1 (switch release (0-1 transition) detected)
 *                  0 (was not pressed)
 *
 * Side Effects:    None
 *
 * Overview:        Evaluates switchLevelxxx variables
 *
 * Note:            None
 ******************************************************************************/

int SwitchWasPressed(void){
  if((switchLevelCurrent==1)&&(switchLevelPrevious==0)){
    return 1;
  }
  else{
    return 0;
  }
}

/*******************************************************************************
 * Function:        void OptoSampleInputs(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Samples/saves opto-sensor signal levels
 *
 * Notes:	Sampling interval (in ms) is set by OPTO_SAMPLE_INTERVAL
 *			Samples output interval (in ms) set by OPTO_PRINT_INTERVAL
 ******************************************************************************/

void OptoSampleInputs(void){
  	
  // LEFT (PA10/A3))
	aioMonitorFeeds.left_opto = analogRead(LEFT_OPTO_PIN);    // read/save the conversion result
	// MIDDLE (PA11/A2))
	aioMonitorFeeds.middle_opto = analogRead(MIDDLE_OPTO_PIN);// read/save the conversion result
	// RIGHT (PA02/A0))
	aioMonitorFeeds.right_opto = analogRead(RIGHT_OPTO_PIN);  // read/save the conversion result   
}

/*******************************************************************************
 * Function:        int OptoLineDetect(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          0 (Left: 0, Middle: 0, Right: 0)
 *                  1 (Left: 0, Middle: 0, Right: 1)
 *                  2 (Left: 0, Middle: 1, Right: 0)  // Robot centered over line
 *                  3 (Left: 0, Middle: 1, Right: 1)
 *                  4 (Left: 1, Middle: 0, Right: 0)
 *                  5 (Left: 1, Middle: 0, Right: 1)
 *                  6 (Left: 1, Middle: 1, Right: 0)
 *                  7 (Left: 1, Middle: 1, Right: 1)
 *
 * Side Effects:    None
 *
 * Overview:        Determines which opto sensors are over a line
 *                  0: line not detected
 *                  1: line detected
 *
 * Note:            None
 ******************************************************************************/

int OptoLineDetect(void){
    
    int temp = 0;
    // has a line been detected?
    if(aioMonitorFeeds.left_opto > aioMonitorFeeds.optoCal.left_opto_trip){
        temp = temp | 4;
    }
    if(aioMonitorFeeds.middle_opto > aioMonitorFeeds.optoCal.middle_opto_trip){
        temp = temp | 2;
    }
    if(aioMonitorFeeds.right_opto > aioMonitorFeeds.optoCal.right_opto_trip){
        temp = temp | 1;
    }
    return temp;
}

/*******************************************************************************
 * Function:        int CollisionDetect(void)
 *
 * PreCondition:    None
 *
 * Input:           Recent target range measurement in cm (variable "aioMonitorFeeds.collisionDistance")
 *                  Range threshold in cm (symbolic constant "COLLISION_DETECT_THRESHOLD")
 *
 * Output:          0 (no collision imminent
 *                  1 (collision is imminent)
 *
 * Side Effects:    None
 *
 * Overview:        Determines if a collision with target object is imminent
 *
 * Note:            None
 ******************************************************************************/

int CollisionDetect(void){
    
    // has a target been detected?
    if(aioMonitorFeeds.collisionDistance == -1.0){                                       
      return 0;       // out of range result, ignore
    }
    else if(aioMonitorFeeds.collisionDistance <= COLLISION_DETECT_THRESHOLD){
      return 1;       // collision is imminent
    }
    else{
      return 0;       // collision is not imminent
    }
}

/*******************************************************************************
 * Function:        void RunningTheFairway(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Main function implementing robot state machine for the
 *                  challenge "Running The Fairway"
 *
 * Note:            None
 ******************************************************************************/

void RunningTheFairway(void){
  UserLedTasks();                                     // update user led based on state
  switch(aioMonitorFeeds.robotState){
    case IDLE:
      RobotWait();
      if((event == evPushButton)||(event == evRemotePushButton)){
        event = evNone;                               // acknowledge event
        g_lServoSpeed = LEFT_SERVO_DEFAULT_STRAIGHT;
        g_rServoSpeed = RIGHT_SERVO_DEFAULT_STRAIGHT;
        RobotForward(g_lServoSpeed, g_rServoSpeed);   // Setup initial motor speed
        previousOptoSampleTime = 0;	                  // initialize opto sample timer
        aioMonitorFeeds.robotState = SEEK_START;
      }
      break;
    case SEEK_START:
      RobotFindTee();
      if(event == evFoundTee){
        event = evNone;                               // acknowledge event
        delay(250);                                   // keep moving past start tee
        g_lServoSpeed = LEFT_SERVO_DEFAULT_STRAIGHT;  // reinitialize motor speed setting for default straight speed
        g_rServoSpeed = RIGHT_SERVO_DEFAULT_STRAIGHT;
        previousOptoSampleTime = 0;	                  // initialize opto sample timer
        aioMonitorFeeds.robotState = RUN1;
      }
      break;
    case RUN1:\
      RobotFollowLine();
      if(event == evLost){
        event = evNone;
        RobotStop();
        aioMonitorFeeds.robotState = IDLE;
      }
      if((event == evFoundTee) || (event == evCollisionDetect)){
        event = evNone;
        RobotTurnAround();
        g_lServoSpeed = LEFT_SERVO_DEFAULT_STRAIGHT;  // reinitialize motor speed setting for default straight speed
        g_rServoSpeed = RIGHT_SERVO_DEFAULT_STRAIGHT;
        previousOptoSampleTime = 0;	                  // initialize opto sample timer
        aioMonitorFeeds.robotState = RUN2;
      }
      break;
    case RUN2:
      RobotFollowLine();
      if(event == evLost){
        event = evNone;
        RobotStop();
        aioMonitorFeeds.robotState = IDLE;
      }
      if(event == evFoundTee){
        event = evNone;
        RobotTurnAround();
        g_lServoSpeed = LEFT_SERVO_DEFAULT_STRAIGHT;  // reinitialize motor speed setting for default straight speed
        g_rServoSpeed = RIGHT_SERVO_DEFAULT_STRAIGHT;
        previousOptoSampleTime = 0;	                  // initialize opto sample timer
        aioMonitorFeeds.robotState = RUN3;
      }
      break;
    case RUN3:
      RobotFollowLine();
      if(event == evLost){
        event = evNone;
        RobotStop();
        aioMonitorFeeds.robotState = IDLE;
      }
      if((event == evFoundTee) || (event == evCollisionDetect)){
        event = evNone;
        RobotTurnAround();
        g_lServoSpeed = LEFT_SERVO_DEFAULT_STRAIGHT;  // reinitialize motor speed setting for default straight speed
        g_rServoSpeed = RIGHT_SERVO_DEFAULT_STRAIGHT;
        previousOptoSampleTime = 0;	                  // initialize opto sample timer
        aioMonitorFeeds.robotState = RUN4;
      }
      break;
    case RUN4:
      RobotFollowLine();
      if(event == evLost){
        event = evNone;
        RobotStop();
        aioMonitorFeeds.robotState = IDLE;
      }
      if(event == evFoundTee){
        event = evNone;
        RobotStop();
        aioMonitorFeeds.robotState = IDLE;
      }
      break;  
    default:
      break;
  }
}

/*** Robot State Chart Behavior Functions ************************************/

void RobotWait(void){
  
  // Sample Optosensor Inputs at regular intervals - update Adafruit IO dashboard
  currentOptoSampleTime = millis();
  if((currentOptoSampleTime - previousOptoSampleTime) >= OPTO_SAMPLE_INTERVAL){
      previousOptoSampleTime = currentOptoSampleTime;
      OptoSampleInputs();
  }
  
  // Look for Events
  SwitchTasks();                              // sample user switch
  if(SwitchWasPressed()){
    event = evPushButton;                     // event: "evPushButton" from local user switch
  }
  else{
	  event = evNone;
  }
}

void RobotFindTee(void){
  
  // Sample Optosensor Inputs at regular intervals
  currentOptoSampleTime = millis();
  if((currentOptoSampleTime - previousOptoSampleTime) >= OPTO_SAMPLE_INTERVAL){
		  previousOptoSampleTime = currentOptoSampleTime;
      OptoSampleInputs();
      // Look for Events          
      switch(OptoLineDetect()){
        case 7:
          event = evFoundTee;       // Left, Middle & Right detect a line
          break;
        default:
          event = evNone;
          break;
      }
  }
}

void RobotFollowLine(void){
  // Follow the line, look for the "evFoundTee" or "evLost" events
  
  // Inputs:  Left Middle  Right      Outputs:      lServo      rServo      event       comment
  
  //            0     0     0                       0           0           evLost      lost the track. stop.
  //            0     0     1                       +1          -1          evNone      turn hard right
  //            0     1     0                       default     default     evNone      keep straight
  //            0     1     1                       +1          no change   evNone      turn right
  //            1     0     0                       -1          +1          evNone      turn hard left
  //            1     0     1                       0           0           evLost      lost the track. stop.
  //            1     1     0                       no change   +1          evNone      turn left
  //            1     1     1                       0           0           evFoundTee  found tee. stop.

  // Sample Optosensor Inputs at regular intervals
  currentOptoSampleTime = millis();
  if((currentOptoSampleTime - previousOptoSampleTime) >= OPTO_SAMPLE_INTERVAL){
		  previousOptoSampleTime = currentOptoSampleTime;
      OptoSampleInputs();
      switch(OptoLineDetect()){
        case 0:   // (Left: 0 Middle: 0 Right: 0)
          //RobotStop();
          //event = evLost;
          // continue on previous heading
          event=evNone;
          break;
        case 1:   // (Left: 0 Middle: 0 Right: 1)
          if(g_lServoSpeed < LSERVO_UPPER_SPEED_LIMIT){
            g_lServoSpeed++;
          }
          if(g_rServoSpeed > RSERVO_LOWER_SPEED_LIMIT){
            g_rServoSpeed--;
          }
          RobotForward(g_lServoSpeed, g_rServoSpeed);
          event = evNone;
          break;
        case 2:   // (Left: 0 Middle: 1 Right: 0)
          g_lServoSpeed = LEFT_SERVO_DEFAULT_STRAIGHT;  // reinitialize motor speed setting for default straight speed
          g_rServoSpeed = RIGHT_SERVO_DEFAULT_STRAIGHT;
          RobotForward(g_lServoSpeed, g_rServoSpeed);
          event = evNone;
          break;
        case 3:   // (Left: 0 Middle: 1 Right: 1)
          if(g_lServoSpeed < LSERVO_UPPER_SPEED_LIMIT){
            g_lServoSpeed++;
          }
          //if(g_rServoSpeed > RSERVO_LOWER_SPEED_LIMIT){
          //  g_rServoSpeed--;
          //}
          RobotForward(g_lServoSpeed, g_rServoSpeed);
          event = evNone;
          break;
        case 4:   // (Left: 1 Middle: 0 Right: 0)
          if(g_rServoSpeed < RSERVO_UPPER_SPEED_LIMIT){
            g_rServoSpeed++;
          }
          if(g_lServoSpeed > LSERVO_LOWER_SPEED_LIMIT){
            g_lServoSpeed--;
          }
          RobotForward(g_lServoSpeed, g_rServoSpeed);
          event = evNone;
          break;
        case 5:   // (Left: 1 Middle: 0 Right: 1)
          RobotStop();
          event = evLost;
          break;
        case 6:   // (Left: 1 Middle: 1 Right: 0)
          if(g_rServoSpeed < RSERVO_UPPER_SPEED_LIMIT){
            g_rServoSpeed++;
          }
          //if(g_lServoSpeed > LSERVO_LOWER_SPEED_LIMIT){
          //  g_lServoSpeed--;
          //}
          RobotForward(g_lServoSpeed, g_rServoSpeed);
          event = evNone;
          break;
        case 7:   // (Left: 1 Middle: 1 Right: 1)
          g_lServoSpeed = LEFT_SERVO_DEFAULT_STRAIGHT;  // reinitialize motor speed setting for default straight speed
          g_rServoSpeed = RIGHT_SERVO_DEFAULT_STRAIGHT;
          RobotForward(g_lServoSpeed, g_rServoSpeed);
          delay(50);
          RobotStop();
          event = evFoundTee;
          break;
        default:
          break;
      }

      if(CollisionDetect()){
        event = evCollisionDetect;
        RobotStop();
      }  
  }
}

void RobotTurnAround(void){

  int lineDetected = 0;
  previousOptoSampleTime = 0;	                    // initialize opto sample timer
  // rotate the robot 180 degrees via right-hand turn
  g_lServoSpeed = LEFT_SERVO_DEFAULT_STRAIGHT; 
  g_rServoSpeed = RIGHT_SERVO_DEFAULT_STRAIGHT;
  RobotRotateRight(g_lServoSpeed, g_rServoSpeed); // begin a sharp turn
  delay(50);                                    // ensure optos are not positioned over line                                      
  while(!lineDetected){                           // maintain rotation until Left: 0, Middle: 1, Right: 0
    currentOptoSampleTime = millis();
    if((currentOptoSampleTime - previousOptoSampleTime) >= OPTO_SAMPLE_INTERVAL){
		  previousOptoSampleTime = currentOptoSampleTime;
      OptoSampleInputs();
      if(OptoLineDetect() != 0){
        lineDetected = 1;
      }
    }
  }
  RobotStop();
}

/*******************************************************************************
 * Function:        void RobotForward(int lServoSpeed, int rServoSpeed)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Robot moves forward at a given speed (torque) setting
 *
 * Note:            None
 ******************************************************************************/

void RobotForward(int lServoSpeed, int rServoSpeed){
  
  // Update LEFT Motor PWM Control Inputs to "Forward" state at the requested speed
  lServo.write(90-lServoSpeed);
  // Update RIGHT Motor PWM Control Inputs to "Forward" state at the requested speed
  rServo.write(90-rServoSpeed);
}

/*******************************************************************************
 * Function:        void RobotReverse(int speed)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Robot moves backward at a given speed (torque)
 *
 * Note:            None
 ******************************************************************************/

void RobotReverse(int lServoSpeed, int rServoSpeed){
  
  // Update LEFT Motor PWM Control Inputs to "Reverse" state at the requested speed
  lServo.write(90+lServoSpeed);
  // Update RIGHT Motor PWM Control Inputs to "Reverse" state at the requested speed
  rServo.write(90+rServoSpeed);
}

/*******************************************************************************
 * Function:        void RobotRotateLeft(int lServoSpeed, int rServoSpeed)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        rotate robot sharp left at desired speed
 *
 * Note:            None
 ******************************************************************************/

void RobotRotateLeft(int lServoSpeed, int rServoSpeed){
  
  // Update LEFT Motor PWM Control Inputs to "Reverse" state at the requested speed
  lServo.write(90+lServoSpeed);
  // Update RIGHT Motor PWM Control Inputs to "Forward" state at the requested speed
  rServo.write(90-rServoSpeed);
}

/*******************************************************************************
 * Function:        void RobotRotateRight(int lServoSpeed, int rServoSpeed)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        rotate robot sharp right at desired speed
 *
 * Note:            None
 ******************************************************************************/

void RobotRotateRight(int lServoSpeed, int rServoSpeed){
  
  // Update LEFT Motor PWM Control Inputs to "Forward" state at the requested speed
  lServo.write(90-lServoSpeed);
  // Update RIGHT Motor PWM Control Inputs to "Reverse" state at the requested speed
  rServo.write(90+rServoSpeed); 
}

/*******************************************************************************
 * Function:        void RobotStop(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Stops the robot by braking the motors
 *
 * Note:            None
 ******************************************************************************/

void RobotStop(void){
  
  // Update LEFT Motor PWM Control Inputs to "Stop" state
  lServo.write(90);
  // Update RIGHT Motor PWM Control Inputs to "Stop" state
  rServo.write(90);
}

/*** mqttc serialization functions ******************************************/

// aioMonitorFeedsSerialize()
// Serialize "aioMonitorFeeds" data structure variables into a single JSON payload
// "left_opto", "middle_opto", "right_opto"
// "left_opto_trip", "middle_opto_trip", "right_opto_trip"
// "robotState"
// JSON Payload Prototype example:
// {"feeds": {"left_opto": 656, "middle_opto": 457, "right_opto": 345, "left_opto_trip": 750, "middle_opto_trip": 755, "right_opto_trip": 765, "robot_state": "IDLE"}}


String aioMonitorFeedsSerialize(void){
  
  String jsonSerializedOutput;
  jsonSerializedOutput = "";

  jsonSerializedOutput += "{\"feeds\": ";
  jsonSerializedOutput += "{\"left_opto\": ";
  jsonSerializedOutput += aioMonitorFeeds.left_opto;
  jsonSerializedOutput += ", ";
  jsonSerializedOutput += "\"middle_opto\": ";
  jsonSerializedOutput += aioMonitorFeeds.middle_opto;
  jsonSerializedOutput += ", ";
  jsonSerializedOutput += "\"right_opto\": ";
  jsonSerializedOutput += aioMonitorFeeds.right_opto;
  jsonSerializedOutput += ", ";
  jsonSerializedOutput += "\"left_opto_trip\": ";
  jsonSerializedOutput += aioMonitorFeeds.optoCal.left_opto_trip;
  jsonSerializedOutput += ", ";
  jsonSerializedOutput += "\"middle_opto_trip\": ";
  jsonSerializedOutput += aioMonitorFeeds.optoCal.middle_opto_trip;
  jsonSerializedOutput += ", ";
  jsonSerializedOutput += "\"right_opto_trip\": ";
  jsonSerializedOutput += aioMonitorFeeds.optoCal.right_opto_trip;
  jsonSerializedOutput += ", ";
  jsonSerializedOutput += "\"collision_distance\": ";
  jsonSerializedOutput += aioMonitorFeeds.collisionDistance;
  jsonSerializedOutput += ", ";
  jsonSerializedOutput += "\"robot_state\": ";
  switch(aioMonitorFeeds.robotState){
    case IDLE:
      jsonSerializedOutput += "\"IDLE\"";
      break;
    case SEEK_START:
      jsonSerializedOutput += "\"SEEK_START\"";
      break;
    case RUN1:
      jsonSerializedOutput += "\"RUN1\"";
      break;
    case RUN2:
      jsonSerializedOutput += "\"RUN2\"";
      break;
    case RUN3:
      jsonSerializedOutput += "\"RUN3\"";
      break;
    case RUN4:
      jsonSerializedOutput += "\"RUN4\"";
      break;
    default:
      jsonSerializedOutput += "\"IDLE\"";
      break;     
  }
  jsonSerializedOutput += "}";
  jsonSerializedOutput += "}";
  
  return jsonSerializedOutput;
}