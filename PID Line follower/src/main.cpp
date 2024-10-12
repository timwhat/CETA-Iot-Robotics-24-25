/*** Include Files ************************************************************/
#include <ArduinoJson.h>        // Required for JSON deserialization
#include <ESP32Servo.h>              // Required for the Servo library APIs
#include <EEPROM.h>             // Required for flash EEPROM emulation to store OPTO calibration values
#include <HCSR04.h>
#include <QTRSensors.h>
#include "mqttc.h"


/*** Symbolic Constants used in this module ***********************************/

// Digital I/O Pin Function Assignments
#define USER_LED_PIN 2         // OUTPUT - connected to USER LED (D1) 
#define USER_SWITCH_PIN 32      // INPUT - connected to USER SWITCH (S2)
#define LEFT_MOTOR_PWM_PIN 33    // OUTPUT - connected to motor controller "S2" input ("Left" Motor)
#define RIGHT_MOTOR_PWM_PIN 25   // OUTPUT - connected to motor controller "S1" input ("Right" Motor)
#define HCSR04_TRIGGER_PIN 27   // DIGITAL OUTPUT - connected to HCSR04 Trigger pin
#define HCSR04_ECHO_PIN 26      // DIGITAL INPUT - connected to HCSR04 Echo pin

// Opto Sensor Configs
#define OPTO_CALIBRATE_INTERVAL 10000   // averaging time for opto calibration (in mS)
QTRSensors qtra;
// QTRSensors qtrrc;

// Define the pin numbers for the analog sensors
const int analogSensorCount = 4;
const uint8_t analogPins[analogSensorCount] = {36, 39, 34, 35};  // Analog pins for 4 analog sensors (2 left, 2 right)

// Define the pin numbers for the HY-S301 sensor array
const int digitalSensorCount = 8;
const uint8_t digitalPins[digitalSensorCount] = {22, 21, 19, 18, 5, 17, 16, 4};  // Analog pins for HY-S301 sensors

const int SensorCount = analogSensorCount + digitalSensorCount;

const int NUM_LEFT_ANALOG = 2;
const int NUM_DIGITAL = 8;
const int NUM_RIGHT_ANALOG = 2;

// Ultrasonic collision sensor definitions
#define COLLISION_SAMPLE_INTERVAL 50   // sample interval for collision sensor
#define COLLISION_DETECT_THRESHOLD 10.0 // collision distance (cm) threshold, written as a float constant

// Servo Motor Default Speed Settings (Straight Motion)
#define LEFT_SERVO_DEFAULT_STRAIGHT 10     
#define RIGHT_SERVO_DEFAULT_STRAIGHT 10    

/*** Global Variable Declarations *********************************************/

// switch variables
int switchLevelCurrent;   
int switchLevelPrevious;

// collision sensor sampling interval variables
unsigned long collisionDistanceCurrentSampleTime;  // Used for controlling collision sensor sample rate
unsigned long collisionDistancePreviousSampleTime = 0;
const long collisionDistanceSampleInterval = COLLISION_SAMPLE_INTERVAL;

typedef struct{
  int isCalibrated;
  uint16_t threshold[SensorCount];
} OPTO_CAL;

// define some robot states
enum ROBOT_STATE {CALIBRATE=0, CALIBRATE_BLACK, IDLE, SEEK_START, RUN1, RUN2, RUN3, RUN4};

// Adafruit "monitor" group variables
typedef struct{
  uint16_t sensorValues[SensorCount];
  float position;
  OPTO_CAL optoCal;                     // create a structure to store runtime optosensor trip values
  enum ROBOT_STATE robotState = IDLE;   // declare & initialize a variable that holds the current state
  float collisionDistance;              // collision distance measurement from HCSR04
} AIO_MONITOR_FEEDS;

AIO_MONITOR_FEEDS aioMonitorFeeds;

typedef struct {
  // PID VALS
  float Kp;
  float Ki;
  float Kd;
} AIO_CONTROL;

AIO_CONTROL aioControl;

// Weights for each sensor
const int weights[] = {-6, -5, -4, -3, -2, -1, 1, 2, 3, 4, 5, 6};

// PID STUFF
uint8_t multiP = 1;
uint8_t multiI  = 1;
uint8_t multiD = 1;
uint8_t Kpfinal;
uint8_t Kifinal;
uint8_t Kdfinal;
float Pvalue;
float Ivalue;
float Dvalue;

int val, cnt = 0, v[3];

int P, I, D, previousError, PIDvalue;//, error;
int lsp, rsp;

// variables that control how often "aioMonitorFeeds" are published to the broker
unsigned long aioMonitorFeedsCurrentSampleTime;
unsigned long aioMonitorFeedsPreviousSampleTime;
const long aioMonitorFeedsSampleInterval = 4000;   // "aioMonitorFeeds" update interval (in mS)

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
  {1, 0, 0, 0, 0, 0, 0, 0, 0, 0},         // CALIBRATE
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
// void RobotTurnAround(void);     // Turn around at the Tee
// void RobotHalt(void);           // Halt in place

// robot motor control functions
void ControlRobot(String action, float lServoSpeed = 0, float rServoSpeed = 0);

// switch functions
void SwitchTasks(void);         // Sample/update switch S1 state variables                        
int SwitchWasPressed(void);     // Detect if switch S1 was pressed (push-release)

// sensor values
void OptoCalibrate(void);
int OptoLineDetect(void);

// collision detection function
int CollisionDetect(void);      // Determine whether robot is within a certain distance from a target

void PIDLineFollow(int error);

// User led functions
void UserLedTasks(void);        // Update the USER LED based on the current robot state

// serialization functions for publish ("monitor") variables
String aioMonitorFeedsSerialize(void);   // serialize "aioMonitorFeeds" data structure for publish

// deserialization functions for subscribe ("control") variables
void eventDeserialize(void);      // De-serialize "event" JSON message, save value into "event" variable

void setup() {
  // perform all one-time setup initialization
  Initialize();
}

void loop() {
  
  // Super-loop; cooperating tasks, no blocking code in any function!        
  
  // MQTT client tasks
  mqttcTasks();

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
    mqttcTx(PUB_TOPIC_AIO_MONITOR_FEEDS, aioMonitorFeedsSerialize());
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

  // Initialize sensors
  qtra.setTypeAnalog();
  qtra.setSensorPins(analogPins, analogSensorCount);

  // Initialize digital sensor pins as inputs
  for (int i = 0; i < digitalSensorCount; i++) {
    pinMode(digitalPins[i], INPUT);
  }

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
  EEPROM.get(0, aioControl);
  if((0 == aioMonitorFeeds.optoCal.isCalibrated) || (0 == digitalRead(USER_SWITCH_PIN))){
    // Wait for user to release the button
    while(0 == digitalRead(USER_SWITCH_PIN));
    Serial.println("OptoSensor Calibration routine triggered");
    // Clear EEPROM by writing a 0 to all 512 bytes
    for (int i = 0; i < 512; i++) {
      EEPROM.write(i, 0);
    }
    aioMonitorFeeds.robotState = CALIBRATE;
    OptoCalibrate();
    // Write calibration values to EEPROM (Flash memory)
    EEPROM.put(0, aioMonitorFeeds.optoCal);
    EEPROM.put(0, aioControl);
  }
  EEPROM.commit();
  
  // mqttc Initialization: Connect to Adafruit server, subscribe for all notifications
  mqttcInitialize();

  // Set robot state
  aioMonitorFeeds.robotState = IDLE;

  switchLevelCurrent = digitalRead(USER_SWITCH_PIN);  // initialize switch level

  // Initialize collision distance measurement
  aioMonitorFeeds.collisionDistance = distanceSensor.measureDistanceCm();
  
  // send initial values to the Serial Monitor
  Serial.println(aioMonitorFeedsSerialize());

  // send initial values to the Dashboard
  mqttcTx(PUB_TOPIC_AIO_MONITOR_FEEDS, aioMonitorFeedsSerialize());
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

    // TODO: add while thing so it wont do when active

    // analogRead() takes about 0.1 ms on an AVR.
    // 0.1 ms per sensor * 4 samples per sensor read (default) * 6 sensors
    // * 10 reads per calibrate() call = ~24 ms per calibrate() call.
    // Call calibrate() 400 times to make calibration take about 10 seconds.

    for (uint16_t i = 0; i < (OPTO_CALIBRATE_INTERVAL / 25); i++)
    {
        qtra.calibrate();
    }
    
    // For digital sensors:
    // Here we set a static threshold. If your digital sensors output directly high or low, you might not need calibration.
    // Instead, ensure that you read the sensors in various conditions (e.g., over line and off line) to determine typical high/low states.

    for (int i = 0; i < NUM_DIGITAL; i++) {
        // Read each digital sensor and determine a threshold based on empirical data or expected high/low values
        // For example, you might use a pre-determined threshold if your sensors have a defined behavior.
        aioMonitorFeeds.optoCal.threshold[NUM_LEFT_ANALOG + i] = 1; //(digitalRead(digitalPins[i]) == HIGH) ? 1 : 0;
    }

    // Place QTR sensor readings into the final sensorValues array
    for (int i = 0; i < NUM_LEFT_ANALOG; i++) {
      aioMonitorFeeds.optoCal.threshold[i] = (qtra.calibrationOn.minimum[i] + qtra.calibrationOn.maximum[i])/2;  // Left analog sensors
    }

    // Read remaining analog sensors (right side) and place them into the final sensorValues array
    for (int i = 0; i < NUM_RIGHT_ANALOG; i++) {
      aioMonitorFeeds.optoCal.threshold[NUM_LEFT_ANALOG + NUM_DIGITAL + i] = (qtra.calibrationOn.minimum[NUM_LEFT_ANALOG + i] + qtra.calibrationOn.maximum[NUM_LEFT_ANALOG + i])/2;  // Right analog sensors
    }
    
    Serial.println("Calibration complete.");
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
      case CALIBRATE:
        digitalWrite(USER_LED_PIN, userLedOutputValue[0][userLedValueIndex++]);
        break;
      // case CALIBRATE_BLACK:
      //   digitalWrite(USER_LED_PIN, userLedOutputValue[1][userLedValueIndex++]);
      //   break;
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
 * Function:        int OptoLineDetect(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          0 if no sensors detect a line
 *                  -1 if all sensors detect the line
 *                  everything else around the middle
 *
 * Side Effects:    None
 * 
 * Overview:        Determines which opto sensors are over a line
 *                  0: line not detected
 *                  1: line detected
 *
 * Note:            None
 ******************************************************************************/

int OptoLineDetect(void) {
  uint16_t tempQTRAValues[analogSensorCount];  // Temporary array for QTR sensor readings
  uint8_t tempQTRRCValues[digitalSensorCount];  // Temporary array for digital sensor readings

  // Read analog sensors
  qtra.read(tempQTRAValues);

  int i;

  // Read digital sensors
  for (i = 0; i < digitalSensorCount; i++) tempQTRRCValues[i] = digitalRead(digitalPins[i]); // Read digital sensors directly

  // Place sensor readings into the final sensorValues array
  // Left analog sensors
  for (i = 0; i < NUM_LEFT_ANALOG; i++) aioMonitorFeeds.sensorValues[i] = tempQTRAValues[i];
  
  // Digital sensors
  for (i = 0; i < NUM_DIGITAL; i++) aioMonitorFeeds.sensorValues[NUM_LEFT_ANALOG + i] = tempQTRRCValues[i];
  
  // Right analog sensors
  for (i = 0; i < NUM_RIGHT_ANALOG; i++) aioMonitorFeeds.sensorValues[NUM_LEFT_ANALOG + NUM_DIGITAL + i] = tempQTRAValues[NUM_LEFT_ANALOG + i];
  
  int sumWeights = 0, sumValues = 0;

  // Calculate weighted average for line position based on absolute detection
  for (i = 0; i < SensorCount; i++) {
    if(aioMonitorFeeds.sensorValues[i] >= aioMonitorFeeds.optoCal.threshold[i]) {
      sumWeights += weights[i];
      sumValues++;
    }
  }

  // Calculate the weighted average (error function)
  if (sumValues != 0) aioMonitorFeeds.position = (float)sumWeights / sumValues;
  else aioMonitorFeeds.position = 0;

  // Check sensor detection status
  if (sumValues == SensorCount) return -1; // All sensors detect the line
  else if (sumValues == 0) return 0;     // No sensors detect the line
  else return 1;                         // Some sensors detect the line
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
        ControlRobot("forward", g_lServoSpeed, g_rServoSpeed);   // Setup initial motor speed
        // previousOptoSampleTime = 0;	                  // initialize opto sample timer
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
        // previousOptoSampleTime = 0;	                  // initialize opto sample timer
        aioMonitorFeeds.robotState = RUN1;
      }
      break;
    case RUN1:
      RobotFollowLine();
      if((event == evFoundTee) || (event == evCollisionDetect)){
        event = evNone;
        ControlRobot("turnAround", g_lServoSpeed, g_rServoSpeed);
        g_lServoSpeed = LEFT_SERVO_DEFAULT_STRAIGHT;  // reinitialize motor speed setting for default straight speed
        g_rServoSpeed = RIGHT_SERVO_DEFAULT_STRAIGHT;
        // previousOptoSampleTime = 0;	                  // initialize opto sample timer
        aioMonitorFeeds.robotState = RUN2;
      }
      break;
    case RUN2:
      RobotFollowLine();
      if(event == evFoundTee){
        event = evNone;
        ControlRobot("turnAround", g_lServoSpeed, g_rServoSpeed);
        g_lServoSpeed = LEFT_SERVO_DEFAULT_STRAIGHT;  // reinitialize motor speed setting for default straight speed
        g_rServoSpeed = RIGHT_SERVO_DEFAULT_STRAIGHT;
        // previousOptoSampleTime = 0;	                  // initialize opto sample timer
        aioMonitorFeeds.robotState = RUN3;
      }
      break;
    case RUN3:
      RobotFollowLine();
      if((event == evFoundTee) || (event == evCollisionDetect)){
        event = evNone;
        ControlRobot("turnAround", g_lServoSpeed, g_rServoSpeed);
        g_lServoSpeed = LEFT_SERVO_DEFAULT_STRAIGHT;  // reinitialize motor speed setting for default straight speed
        g_rServoSpeed = RIGHT_SERVO_DEFAULT_STRAIGHT;
        // previousOptoSampleTime = 0;	                  // initialize opto sample timer
        aioMonitorFeeds.robotState = RUN4;
      }
      break;
    case RUN4:
      RobotFollowLine();
      if(event == evFoundTee){
        event = evNone;
        ControlRobot("stop");
        aioMonitorFeeds.robotState = IDLE;
      }
      break;  
    default:
      break;
  }
}

/*** Robot State Chart Behavior Functions ************************************/

void RobotWait(void){
  
  // // Sample Optosensor Inputs at regular intervals - update Adafruit IO dashboard
  OptoLineDetect();
  
  // Look for Events
  SwitchTasks();                              // sample user switch
  if(SwitchWasPressed()){
    event = evPushButton;                     // event: "evPushButton" from local user switch
  }
  else if(mqttcRxIsAvailable(SUB_TOPIC_AIO_CONTROL)){
    Serial.println("New message available on the control topic.");
    eventDeserialize();                       // event: assigned (set) from "pushbutton" IoT dashboard widget
  }
  else if(mqttcRxIsAvailable(SUB_TOPIC_PID_K_VALUES)){
    Serial.println("New message available on the PID control topic.");
    eventDeserialize();                       // event: assigned (set) from "pushbutton" IoT dashboard widget
  }
  else{
	  event = evNone;
  }
}

void RobotFindTee(void){  
  switch(OptoLineDetect()){
    case -1:
      event = evFoundTee;       // Left, Middle & Right detect a line
      break;
    default:
      event = evNone;
      break;
  }
}

void RobotFollowLine(void){
  RobotFindTee();

  PIDLineFollow(aioMonitorFeeds.position);

  if(CollisionDetect()){
    event = evCollisionDetect;
    // ControlRobot("stop");
  }  
}

void PIDLineFollow(int error){
    P = error;
    I = I + error;
    D = error - previousError;
    
    Pvalue = (aioControl.Kp/pow(10,multiP))*P;
    Ivalue = (aioControl.Ki/pow(10,multiI))*I;
    Dvalue = (aioControl.Kd/pow(10,multiD))*D; 

    float PIDvalue = Pvalue + Ivalue + Dvalue;
    previousError = error;

    lsp =  LEFT_SERVO_DEFAULT_STRAIGHT - PIDvalue;
    rsp = RIGHT_SERVO_DEFAULT_STRAIGHT + PIDvalue;

    // changew so drive right
    if (lsp > 90) lsp = 90;
    if (lsp < 0) lsp = 0;
    if (rsp > 90) rsp = 90;
    if (rsp < 0) rsp = 0;
    ControlRobot("forward", lsp, rsp);
}

void ControlRobot(String action, float lServoSpeed, float rServoSpeed){
  if (action == "forward") {
    // Move forward
    lServo.write(90 - lServoSpeed);
    rServo.write(90 - rServoSpeed);
  } else if (action == "reverse") {
    // Move backward
    lServo.write(90 + lServoSpeed);
    rServo.write(90 + rServoSpeed);
  } else if (action == "rotateLeft") {
    // Rotate left
    lServo.write(90 + lServoSpeed);
    rServo.write(90 - rServoSpeed);
  } else if (action == "rotateRight") {
    // Rotate right
    lServo.write(90 - lServoSpeed);
    rServo.write(90 + rServoSpeed);
  } else if (action == "stop") {
    // Stop
    lServo.write(90);
    rServo.write(90);
  } else if (action == "turnAround") {
    // Turn around
    bool lineDetected = 0;
    // rotate the robot 180 degrees via right-hand turn
    g_lServoSpeed = LEFT_SERVO_DEFAULT_STRAIGHT; 
    g_rServoSpeed = RIGHT_SERVO_DEFAULT_STRAIGHT;
    ControlRobot("rotateRight", g_lServoSpeed, g_rServoSpeed); // begin a sharp turn
    delay(50);                                    // ensure optos are not positioned over line                                      
    while(!lineDetected){                           // maintain rotation until Left: 0, Middle: 1, Right: 0
      if(OptoLineDetect() != 0){
        lineDetected = 1;
      }
    }
    // ControlRobot("stop");
  } else {
    // Invalid action
    Serial.println("Invalid action");
  }
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
  // for (int i = 0; i < SensorCount; i++) {
  //   Serial.print(aioMonitorFeeds.sensorValues[i]);
  //   Serial.print("\t");
  // }
  // Serial.println();
  // for (int i = 0; i < SensorCount; i++) {
  //   Serial.print(aioMonitorFeeds.optoCal.threshold[i]);
  //   Serial.print("\t");
  // }
  // Serial.println(aioControl.Kp);
  // Serial.println(aioControl.Ki);
  // Serial.println(aioControl.Kd);
  // Serial.println();
  String jsonSerializedOutput;
  jsonSerializedOutput = "";

  jsonSerializedOutput += "{\"feeds\": {";
  // for(int i = 0; i < SensorCount; i++){
  //   jsonSerializedOutput += "\"sensor_value ";
  //   jsonSerializedOutput += i;
  //   jsonSerializedOutput += "\": ";
  //   jsonSerializedOutput += aioMonitorFeeds.sensorValues[i];
  //   jsonSerializedOutput += ", ";
  // }
  // jsonSerializedOutput += "\n";
  // for(int i = 0; i < SensorCount; i++){
  //   jsonSerializedOutput += "\"sensor_trip_value ";
  //   jsonSerializedOutput += i;
  //   jsonSerializedOutput += "\": ";
  //   jsonSerializedOutput += aioMonitorFeeds.optoCal.threshold[i];
  //   jsonSerializedOutput += ", ";
  // }
  // jsonSerializedOutput += ", ";
  jsonSerializedOutput += "\"position\": ";
  jsonSerializedOutput += aioMonitorFeeds.position;
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

/*** mqttc de-serialization functions ******************************************/

// eventDeserialize() - De-serialize "event" JSON message, save value into "event" variable
// JSON Payload Prototype: {"event": 3} (event "evRemotePushButton")
// events are enumerated in the global veriable definitions:
// typedef enum {evNone=0, evMCUReset, evPushButton, evRemotePushButton, evTimer, evFoundTee, evLost} EVENT;

void eventDeserialize(void) {
    // Declare a String variable "payload" to hold a JSON payload
    String payload;
    
    // Declare a JSON document to hold any received payload
    StaticJsonDocument<128> doc;
    
    // Declare an error variable for JSON deserialization operation
    DeserializationError error;

    // Read the received JSON packet
    payload = mqttcRx();
    
    // Deserialize the JSON packet
    error = deserializeJson(doc, payload);

    if (error) {
        Serial.print(F("deserializeJson() failed: "));
        Serial.println(error.c_str());
        return;
    }

    // Check for the "event" key
    if (doc.containsKey("event")) {
        event = doc["event"]; // Extract/save "event" value
        Serial.print("Event assigned: ");
        Serial.println(event);
    }
    else if (doc.containsKey("kp")) {
        aioControl.Kp = doc["kp"]; // Assign Kp
        Serial.print("Kp assigned: ");
        Serial.println(aioControl.Kp);

        aioControl.Ki = doc["ki"];  // Assign ki
        Serial.print("ki assigned: ");
        Serial.println(aioControl.Ki);

        aioControl.Kd = doc["kd"];  // Assign kd
        Serial.print("kd assigned: ");
        Serial.println(aioControl.Kd);

        // Write the PID values to EEPROM (Flash memory)
        EEPROM.begin(512);
        EEPROM.get(0, aioMonitorFeeds.optoCal);
        EEPROM.get(0, aioControl);
        // Clear EEPROM by writing a 0 to all 512 bytes
        for (int i = 0; i < 512; i++) {
          EEPROM.write(i, 0);
        }
        // Write calibration values to EEPROM (Flash memory)
        EEPROM.put(0, aioMonitorFeeds.optoCal);
        EEPROM.put(0, aioControl);
        EEPROM.commit();
    }
    else {
        Serial.println("Parsing failed! Invalid Key\r\n"); // No valid keys found in JSON Message
    }
}
