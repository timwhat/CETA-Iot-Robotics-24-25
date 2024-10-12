#ifndef MQTTC_H_
#define MQTTC_H_

/*** Include Files ***********************************************************/

// Wifi Credentials
#define WIFI_SSID "" // (EDIT) SSID of desired Access Point
#define WIFI_PASS "" // (EDIT) Required Passphrase (WPA2/Personal)

// Adafruit IO Credentials
#define CLIENT_ID "" // (EDIT) ClientID (use https://www.guidgenerator.com/online-guid-generator.aspx)
#define AIO_USERNAME "" // (EDIT) Adafruit IO Username
#define AIO_KEY "" // (EDIT) Adafruit IO Key

// Publish & Subscribe topics
#define PUB_TOPIC_AIO_MONITOR_FEEDS   "" // (EDIT) Adafruit Group Topic ID
#define SUB_TOPIC_AIO_CONTROL   "" // (EDIT) Adafruit Feed Topic ID
#define SUB_TOPIC_PID_K_VALUES "" // (EDIT) Adafruit Feed Topic ID
/*** Macros ******************************************************************/

/*** Custom Data Types *******************************************************/
typedef struct {
    String inTopic;         // topicID for most recent received message
    String inPayload;       // message payload for most recent received message
} MQTTC_SUB_MSG_STAT;


/*** Public Function Prototypes ***********************************************/
void mqttcInitialize(void);                                 // Connect to the broker and subscribe for all notifications
void mqttcTasks(void);                                      // Run mqttc background tasks
void mqttcTx(String outTopic, String jsonOutPayload);       // Publish serialized JSON payload to a topic
bool mqttcRxIsAvailable(String subTopic);                    // Check if JSON message has been received for a specific subscription topic
String mqttcRx(void);                                       // Retrieve JSON payload for deserialization

#endif /* MQTTC_H_ */
