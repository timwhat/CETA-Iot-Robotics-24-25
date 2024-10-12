

/** Include Files *************************************************************/
#include <ArduinoMqttClient.h>  // Required for MQTT client apis
#include <WiFi.h>               // Required for WiFi library APIs
#include <WiFiClientSecure.h>   // Required for BearSSL APIs
#include <WiFiUdp.h>            // Required for NTP client
#include <NTPClient.h>          // Required for NTP client
#include "mqttc.h"              // mqttc APIs

/*** Symbolic Constants used in this module ***********************************/
#define MQTTC_STAT_LED_PIN LED_BUILTIN // DIGITAL OUTPUT - connected to Pico-WH on-board LED
                                  // Indicates netDebug connection status
                                  // ON:        connected to MQTT broker
                                  // OFF:       disconnected
                                  // Flashing:  trying to reconnect

/*** Global Variable Declarations *********************************************/

// WiFi Parameters
const char ssid[] = WIFI_SSID;              // (EDIT) SSID of desired Access Point
const char pass[] = WIFI_PASS;        // (EDIT) Required Passphrase (WPA2/Personal)

// TCP Client Connection Parameters 
const char broker[] = "io.adafruit.com";    // server IP address or hostname
int port = 1883;                            // server port number

// MQTT Client Session Parameters
const char clientID[] = CLIENT_ID;        // (EDIT) ClientID (use https://www.guidgenerator.com/online-guid-generator.aspx)
const char userName[] = AIO_USERNAME;  // (EDIT) MQTT User Name
const char userPass[] = AIO_KEY;   // (EDIT) MQTT User Password

// MQTT Client Publish Parameters
String pubTopicAIOMonitorFeeds = PUB_TOPIC_AIO_MONITOR_FEEDS; // (EDIT in mqttc.h) AIO monitor group topic ID 
int pubQoS = 0;                                 // Set Publish QoS to level 0 (fire-and-forget)
bool retained = false;                          // Disable retained message
//bool dup = false;                               // Duplicates not issued with QoS level 0

// MQTT Client Subscribe Parameters
String subTopicAIOControl = SUB_TOPIC_AIO_CONTROL; // (EDIT in mqttc.h) topic ID for AIO remote switch press feed
int subQoS = 0;                                 // Set Subscribe QoS to level 0 (fire-and-forget)
String subTopicPIDKValues = SUB_TOPIC_PID_K_VALUES; // (EDIT in mqttc.h) topic ID for AIO PID K values feed
int subQoSPIDKValues = 0;                       // Set Subscribe QoS to level 0 (fire-and-forget)

// Initialize Socket classes - MQTT Client
WiFiClient wifiClient;                          // Used for TCP Socket connection
MqttClient mqttClient(wifiClient);              // Instantiate an MQTT client having WiFiClient methods

// WiFi & TCP Connection Monitoring Variables ("connectionTasks()" function)
int linkStatus = WL_IDLE_STATUS;                // WiFi link status
bool tcpConnStatus = false;                     // TCP connection status
unsigned long connStatusCurrentSampleTime, connStatusPrevSampleTime;
const long connStatusSampleInterval = 10000;    // Test connection every 10 seconds

/*** NTP Client Setup *******************************************************/
WiFiUDP ntpUDP;
NTPClient NTP(ntpUDP, "pool.ntp.org");

/*** Private Function Prototypes *********************************************/
static void wifiConnect(void);                  // Connect to WiFi access network
static void mqttClientConnect (void);           // Connect to the MQTT broker
void mqttClientOnMessage(int messageSize);      // Call-back function, processes all subscribed messages from broker
void connectionTasks(void);                     // Monitor WiFi and TCP connection and reconnect if required
void setClock(void);                            // Set time via NTP, as required for x.509 certificate validation

MQTTC_SUB_MSG_STAT mqttcRxPacket;

/*** Public Function Definitions **********************************************/

void mqttcInitialize(void){

    int i;
    String inTopic;

    // Initialize MQTTC CONNECTION STATUS LED
    pinMode(MQTTC_STAT_LED_PIN, OUTPUT);     // set digital pin as output
    digitalWrite(MQTTC_STAT_LED_PIN, 0);     // initialize LED state

    // Attempt to connect to Wifi network (blocking code):
    wifiConnect();

    // Set the MQTT Client Identifier
    mqttClient.setId(clientID);

    // Set the MQTT UserName and Password
    mqttClient.setUsernamePassword(userName, userPass);

    // Attempt to connect to Broker (blocking code)
    mqttClientConnect();

    // Set the MQTT subscription receive callback
    mqttClient.onMessage(mqttClientOnMessage);

    // Subscribe to topicID for all IN messages
    Serial.print("Subscribing to topic: ");
    Serial.println(subTopicAIOControl);
    if (mqttClient.subscribe(subTopicAIOControl.c_str(), subQoS)) {
        Serial.println("Subscribed to AIO Control topic");
    } else {
        Serial.println("Failed to subscribe to AIO Control topic");
    }

    if (mqttClient.subscribe(subTopicPIDKValues.c_str(), subQoSPIDKValues)) {
        Serial.println("Subscribed to PID K Values topic");
    } else {
        Serial.println("Failed to subscribe to PID K Values topic");
    }
    Serial.print("Waiting for messages on topic: ");
    Serial.println(subTopicAIOControl);
    Serial.println(subTopicPIDKValues);
    Serial.println();

    // if you get here, you are connected and ready to go!
    digitalWrite(MQTTC_STAT_LED_PIN, 1);

    // Initialize timeout for connectTasks()
    connStatusPrevSampleTime = 0;

}

void mqttcTasks(void){

    connectionTasks();              // monitor WiFi & TCP connection and reconnect if required
    
    if (!mqttClient.connected()) {
        Serial.println("MQTT client disconnected, attempting to reconnect...");
        mqttClientConnect();
    }

    mqttClient.poll();              // call poll() regularly to allow the library to receive MQTT messages and
                                    // send MQTT keep alives which avoids being disconnected by the broker
}

void mqttcTx(String outTopic, String jsonOutPayload){
    // Publish message to topic
    mqttClient.beginMessage(outTopic, jsonOutPayload.length(), retained, pubQoS, dup);
    mqttClient.print(jsonOutPayload);
    mqttClient.endMessage(); 
}

bool mqttcRxIsAvailable(String subTopic){
    if(mqttcRxPacket.inTopic.equals(subTopic)){
      return 1;
    }
    else{
      return 0;
    }

}

String mqttcRx(void){

    String temp = mqttcRxPacket.inPayload;
    mqttcRxPacket.inPayload = "";         // acknowledge RX packet by deleting it
    mqttcRxPacket.inTopic = "";  
    return temp;                          // return most recent payload
}

/*** Private Function Definitions ********************************************/

void wifiConnect(void){
    // attempt to connect to Wifi network:
    digitalWrite(MQTTC_STAT_LED_PIN, 0);
    Serial.print("\nAttempting to connect to WPA SSID: ");
    Serial.println(ssid);
    WiFi.begin(ssid, pass);
    while (WiFi.status() != WL_CONNECTED) {
        // failed, retry
        Serial.print(".");
        delay(1000);
        digitalWrite(MQTTC_STAT_LED_PIN, 1);
        delay(100);
        digitalWrite(MQTTC_STAT_LED_PIN, 0);
        delay(100);
    }

    // once you are connected :
    Serial.println("You're connected to the network");
}

void mqttClientConnect(void){
    // attempt to connect to MQTT Broker
    digitalWrite(MQTTC_STAT_LED_PIN, 0);
    Serial.print("\nAttempting to connect to the MQTT broker: ");
    Serial.println(broker);
    while(!mqttClient.connect(broker, port)){
        // failed, retry
        Serial.print(".");
        digitalWrite(MQTTC_STAT_LED_PIN, 1);
        delay(100);
        digitalWrite(MQTTC_STAT_LED_PIN, 0);
        delay(100);
        digitalWrite(MQTTC_STAT_LED_PIN, 1);
        delay(100);
        digitalWrite(MQTTC_STAT_LED_PIN, 0);
        delay(100);
    }

    // once you are connected :
    Serial.println("You're connected to the MQTT broker!");
    Serial.println();

}

void connectionTasks(void){
    connStatusCurrentSampleTime = millis();               // get the current time
    // if 10s has elapsed, check the status of both WiFi and TCP connection and reconnect if required
    if ((connStatusCurrentSampleTime - connStatusPrevSampleTime) >= connStatusSampleInterval) {
        connStatusPrevSampleTime = connStatusCurrentSampleTime;
        if(WiFi.status() == WL_CONNECTED){
        Serial.println("WiFi Status: connected");
        if(!mqttClient.connected()){
            digitalWrite(MQTTC_STAT_LED_PIN, 0);       // turn off CONNECT status LED
            mqttClient.flush();
            mqttClient.stop();
            Serial.println("TCP Status: disconnected..attempting to reconnect");
            // reconnect to the broker, using the same MQTT Client initialization as in netDebugInitialize()..
            mqttClient.setId(clientID);
            mqttClientConnect();
            mqttClient.onMessage(mqttClientOnMessage);
            Serial.print("Subscribing to topic: ");
            Serial.println(subTopicAIOControl);
            mqttClient.subscribe(subTopicAIOControl, subQoS);
            mqttClient.subscribe(subTopicPIDKValues, subQoSPIDKValues);
            Serial.print("Waiting for messages on topic: ");
            Serial.println(subTopicAIOControl);
            Serial.println(subTopicPIDKValues);
            Serial.println();
            connStatusPrevSampleTime = 0;
            digitalWrite(MQTTC_STAT_LED_PIN, 1);       // turn on CONNECT status LED
        }
        else{
            Serial.println("TCP Status: connected");
        }
    }
    else{
        Serial.println("WiFi Status: disconnected..attempting to reconnect WiFi annd TCP");
        mqttClient.flush();
        mqttClient.stop();
        wifiConnect();
        mqttClientConnect();
        connStatusPrevSampleTime = 0;
        digitalWrite(MQTTC_STAT_LED_PIN, 1);
    }
  }
}

void mqttClientOnMessage(int messageSize) {
    int i;
    char inBuffer[256];

    mqttcRxPacket.inTopic = String(mqttClient.messageTopic());

    // use the Stream interface to save the contents to a char buffer
    i = 0;
    while (mqttClient.available() && i < sizeof(inBuffer) - 1) {
        inBuffer[i++] = (char)mqttClient.read();
    }
    inBuffer[i] = '\0';

    mqttcRxPacket.inPayload = String(inBuffer);

    Serial.print("Message received on: ");
    Serial.println(mqttcRxPacket.inTopic);
    Serial.print("Payload: ");
    Serial.println(mqttcRxPacket.inPayload);
}

// Set time via NTP, as required for x.509 validation
void setClock() {
  NTP.begin();
  NTP.update();

//   Serial.print("Waiting for NTP time sync: ");
//   NTP.isTimeSet([]() {
//     Serial.print(".");
//   });
//   Serial.println("");
//   time_t now = time(nullptr);
//   struct tm timeinfo;
//   gmtime_r(&now, &timeinfo);
//   Serial.print("Current time: ");
//   Serial.print(asctime(&timeinfo));

  Serial.print("Current time: ");
  Serial.println(NTP.getFormattedTime());
}
