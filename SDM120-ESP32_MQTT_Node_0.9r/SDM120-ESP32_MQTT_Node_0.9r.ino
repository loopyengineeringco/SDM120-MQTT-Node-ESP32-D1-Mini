/*
SDM120-ESP32_MQTT_Node_0.9

Software serial ESP32 based MQTT node for reading a SDM120 meter and throwing the data over MQTT to
Node-Red or what have you.

The SDM120 baud is 2400

MQTT info:
Subscribe your MQTT client to:
SDM120_MQTT_Node_01/state

topics published are:
V
A
W

For extracting the data into Home Asisstant sensors, use Node-Red. Have a look at the github for an example Node-Red setup.

Notes/dependencies:
- Developed on Arduino IDE 1.8.5. Downgrade if you have issues
- Using arduino ESP32 Core version 1.0.6
- https://github.com/reaper7/SDM_Energy_Meter

this is the Node Red flow for extracting the data into home assistant sensors (copy & paste the below into Node Red)
[{"id":"7c481c3c20336b7b","type":"tab","label":"Flow 6","disabled":false,"info":"","env":[]},{"id":"08f5804babfcd983","type":"mqtt in","z":"7c481c3c20336b7b","name":"SDM120_MQTT_Node_01","topic":"SDM120_MQTT_Node_01/state","qos":"1","datatype":"json","broker":"98a72657b1ac3f6a","nl":false,"rap":true,"rh":0,"inputs":0,"x":190,"y":380,"wires":[["470f45cff80f7196","721a4defb6989c1a","5a4811ded6c6d97f","7d7f3b3f3fa3f72b","ace08ded8ffbf8cb"]]},{"id":"470f45cff80f7196","type":"function","z":"7c481c3c20336b7b","name":"Watts","func":"//Watts\nmsg.payload = msg.payload.W.toFixed(0);\nreturn msg;","outputs":1,"noerr":0,"initialize":"","finalize":"","libs":[],"x":450,"y":580,"wires":[["1b972c8bbe7041ef"]],"info":"This function sets the format of each parameter sent by the inverter and assigns each on to a global variable.\r\nTherefore, you can either pass the msg.payload object directly to the next function or pick up inverter \r\nparameters in any function using something like:\r\n\r\nUsing the complete object...\r\n\r\nvar sofar2mqtt = global.get(\"sofar2mqtt\");\r\nvar batterySoc = sofar2mqtt.batterySoc;\r\n\r\nor use individual variables...\r\n\r\nvar batterySoc = global.get(\"sofarBatterySOC\");"},{"id":"721a4defb6989c1a","type":"function","z":"7c481c3c20336b7b","name":"Amps","func":"//Amps\nmsg.payload = msg.payload.A.toFixed(1);\nreturn msg;","outputs":1,"noerr":0,"initialize":"","finalize":"","libs":[],"x":450,"y":500,"wires":[["ae89d750b66970f6"]],"info":"This function sets the format of each parameter sent by the inverter and assigns each on to a global variable.\r\nTherefore, you can either pass the msg.payload object directly to the next function or pick up inverter \r\nparameters in any function using something like:\r\n\r\nUsing the complete object...\r\n\r\nvar sofar2mqtt = global.get(\"sofar2mqtt\");\r\nvar batterySoc = sofar2mqtt.batterySoc;\r\n\r\nor use individual variables...\r\n\r\nvar batterySoc = global.get(\"sofarBatterySOC\");"},{"id":"5a4811ded6c6d97f","type":"function","z":"7c481c3c20336b7b","name":"Volts","func":"//Volts\nmsg.payload = msg.payload.V.toFixed(1);\nreturn msg;","outputs":1,"noerr":0,"initialize":"","finalize":"","libs":[],"x":450,"y":420,"wires":[["fe2e3a97fca36484"]],"info":"This function sets the format of each parameter sent by the inverter and assigns each on to a global variable.\r\nTherefore, you can either pass the msg.payload object directly to the next function or pick up inverter \r\nparameters in any function using something like:\r\n\r\nUsing the complete object...\r\n\r\nvar sofar2mqtt = global.get(\"sofar2mqtt\");\r\nvar batterySoc = sofar2mqtt.batterySoc;\r\n\r\nor use individual variables...\r\n\r\nvar batterySoc = global.get(\"sofarBatterySOC\");"},{"id":"7d7f3b3f3fa3f72b","type":"debug","z":"7c481c3c20336b7b","name":"","active":true,"tosidebar":true,"console":false,"tostatus":false,"complete":"false","statusVal":"","statusType":"auto","x":470,"y":300,"wires":[]},{"id":"fe2e3a97fca36484","type":"ha-entity","z":"7c481c3c20336b7b","name":"SDM120Volts","server":"6178e120.ea2bf","version":2,"debugenabled":false,"outputs":1,"entityType":"sensor","config":[{"property":"name","value":"SDM120Volts"},{"property":"device_class","value":""},{"property":"icon","value":""},{"property":"unit_of_measurement","value":"V"},{"property":"state_class","value":""},{"property":"last_reset","value":""}],"state":"payload","stateType":"msg","attributes":[],"resend":true,"outputLocation":"payload","outputLocationType":"none","inputOverride":"allow","outputOnStateChange":false,"outputPayload":"","outputPayloadType":"str","x":680,"y":420,"wires":[[]]},{"id":"ae89d750b66970f6","type":"ha-entity","z":"7c481c3c20336b7b","name":"SDM120Amps","server":"6178e120.ea2bf","version":2,"debugenabled":false,"outputs":1,"entityType":"sensor","config":[{"property":"name","value":"SDM120Amps"},{"property":"device_class","value":""},{"property":"icon","value":""},{"property":"unit_of_measurement","value":"A"},{"property":"state_class","value":""},{"property":"last_reset","value":""}],"state":"payload","stateType":"msg","attributes":[],"resend":true,"outputLocation":"payload","outputLocationType":"none","inputOverride":"allow","outputOnStateChange":false,"outputPayload":"","outputPayloadType":"str","x":680,"y":500,"wires":[[]]},{"id":"1b972c8bbe7041ef","type":"ha-entity","z":"7c481c3c20336b7b","name":"SDM120Watts","server":"6178e120.ea2bf","version":2,"debugenabled":false,"outputs":1,"entityType":"sensor","config":[{"property":"name","value":"SDM120Watts"},{"property":"device_class","value":""},{"property":"icon","value":""},{"property":"unit_of_measurement","value":"W"},{"property":"state_class","value":""},{"property":"last_reset","value":""}],"state":"payload","stateType":"msg","attributes":[],"resend":true,"outputLocation":"payload","outputLocationType":"none","inputOverride":"allow","outputOnStateChange":false,"outputPayload":"","outputPayloadType":"str","x":680,"y":580,"wires":[[]]},{"id":"37a204541c5a1cb8","type":"ha-entity","z":"7c481c3c20336b7b","name":"SDM120ExportWatts","server":"6178e120.ea2bf","version":2,"debugenabled":false,"outputs":1,"entityType":"sensor","config":[{"property":"name","value":"SDM120ExportWatts"},{"property":"device_class","value":""},{"property":"icon","value":""},{"property":"unit_of_measurement","value":"W"},{"property":"state_class","value":""},{"property":"last_reset","value":""}],"state":"payload","stateType":"msg","attributes":[],"resend":true,"outputLocation":"payload","outputLocationType":"none","inputOverride":"allow","outputOnStateChange":false,"outputPayload":"","outputPayloadType":"str","x":700,"y":660,"wires":[[]]},{"id":"ace08ded8ffbf8cb","type":"function","z":"7c481c3c20336b7b","name":"ExportWatts","func":"//Export Watts calculation\n\nvar Power = msg.payload.W;\nvar ExportPower;\n\nif (Power < 0 ) {\n    ExportPower = Math.abs(Power);\n    } else {\n    ExportPower = 0;\n}\n\nmsg.payload = ExportPower.toFixed(0);\nreturn msg;","outputs":1,"noerr":0,"initialize":"","finalize":"","libs":[],"x":470,"y":660,"wires":[["37a204541c5a1cb8"]],"info":"This function sets the format of each parameter sent by the inverter and assigns each on to a global variable.\r\nTherefore, you can either pass the msg.payload object directly to the next function or pick up inverter \r\nparameters in any function using something like:\r\n\r\nUsing the complete object...\r\n\r\nvar sofar2mqtt = global.get(\"sofar2mqtt\");\r\nvar batterySoc = sofar2mqtt.batterySoc;\r\n\r\nor use individual variables...\r\n\r\nvar batterySoc = global.get(\"sofarBatterySOC\");"},{"id":"98a72657b1ac3f6a","type":"mqtt-broker","name":"Mosquitto","broker":"http://localhost","port":"1883","clientid":"","autoConnect":true,"usetls":false,"protocolVersion":"4","keepalive":"60","cleansession":true,"birthTopic":"","birthQos":"0","birthPayload":"","birthMsg":{},"closeTopic":"","closeQos":"0","closePayload":"","closeMsg":{},"willTopic":"","willQos":"0","willPayload":"","willMsg":{},"sessionExpiry":""},{"id":"6178e120.ea2bf","type":"server","name":"Home Assistant","addon":true}]


*/

// ENTER YOUR WIFI & MQTT BROKER DETAILS HERE

#define WIFI_SSID "xxx"
#define WIFI_PASSWORD "xxx"
static const char mqttUser[] = "xxx";
static const char mqttPassword[] = "xxx";
const char* deviceName = "SDM120_MQTT_Node_01"; //Device name is used as the MQTT base topic.

#define MQTT_HOST IPAddress(192, 168, 0, 206) //replace with your MQTT server IP
#define MQTT_PORT 1883 // replace with your MQTT server port - default should bne 1883

// ##########################################

#include <SDM.h>                               //import SDM library          
#include <SoftwareSerial.h>                    //import SoftwareSerial library  
SoftwareSerial swSerSDM;                       //config SoftwareSerial

//              ________________________________________software serial reference
//             |      __________________________________baudrate(optional, default from SDM_Config_User.h)
//             |     |           _______________________dere pin for max485(optional, default from SDM_Config_User.h)
//             |     |          |              _________software uart config(optional, default from SDM_Config_User.h)
//             |     |          |             |    _____rx pin number(optional, default from SDM_Config_User.h)
//             |     |          |             |   |    _tx pin number(optional, default from SDM_Config_User.h)
//             |     |          |             |   |   | 
SDM sdm(swSerSDM, 2400, NOT_A_PIN, SWSERIAL_8N1, 13, 15);

#include <WiFi.h>
extern "C" {
  #include "freertos/FreeRTOS.h"
  #include "freertos/timers.h"
}
#include <AsyncMqttClient.h>

AsyncMqttClient mqttClient;

float volts = 0;
float amps = 0;
float watts = 0;

TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;
TimerHandle_t mqttPublishXTimer;

bool WiFiStatus = false;
bool MQTTStatus = false;

// USER SETUP VALUES:
unsigned long previousMillisUART = 0;   // will store last time a CAN Message was send
const int intervalUART = 2000;          // interval at which send CAN Messages (milliseconds)

unsigned long previousMillisWIFIMQTT = 0;
const int intervalWIFIMQTT = 2000; 


void getEverythingFromSDM120() {  
  // Grab values from SDM120

  volts = sdm.readVal(SDM_PHASE_1_VOLTAGE);
  //vTaskDelay(200);
  amps = sdm.readVal(SDM_PHASE_1_CURRENT);
  //vTaskDelay(200);
  watts = sdm.readVal(SDM_PHASE_1_POWER);
  //vTaskDelay(400);
}

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event) {
    Serial.printf("[WiFi-event] event: %d\n", event);
    switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
        Serial.println("WiFi connected");
        Serial.println("IP address: ");
        Serial.println(WiFi.localIP());
        WiFiStatus = true;
        connectToMqtt();
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        Serial.println("WiFi lost connection");
        WiFiStatus = false;
        xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
        xTimerStart(wifiReconnectTimer, 0);
        break;
    }
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  xTimerStop(mqttPublishXTimer, 0);
  MQTTStatus = false;
  //mqttPublisherTimer->Stop();
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}

void onMqttUnsubscribe(uint16_t packetId) {
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  Serial.println("Publish received.");
  Serial.print("  topic: ");
  Serial.println(topic);
  Serial.print("  qos: ");
  Serial.println(properties.qos);
  Serial.print("  dup: ");
  Serial.println(properties.dup);
  Serial.print("  retain: ");
  Serial.println(properties.retain);
  Serial.print("  len: ");
  Serial.println(len);
  Serial.print("  index: ");
  Serial.println(index);
  Serial.print("  total: ");
  Serial.println(total);
}

void onMqttPublish(uint16_t packetId) {
  Serial.println("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}


void sendMQTTData() {
  
  //getEverythingFromSDM120();
  
  // Update all parameters and send to MQTT.
  String state = "{";
  
  if (!( state == "{")) { state += ","; }
  state += "\"V\":"+String(volts);

  if (!( state == "{")) { state += ","; }
  state += "\"A\":"+String(amps);

  if (!( state == "{")) { state += ","; }
  state += "\"W\":"+String(watts);
      
  state = state+"}";
  
  //Prefixt the mqtt topic name with deviceName.
  String topic (deviceName);
  topic += "/state";

  if (volts > 0) {
  uint16_t packetIdPub2 = mqttClient.publish(const_cast<char*>(topic.c_str()), 1, true, const_cast<char*>(state.c_str()));
  Serial.println("Publishing at QoS 1");
  Serial.print("Topic: ");
  Serial.println(topic);
  Serial.print("Payload: ");
  Serial.println(state);
  } else {
    Serial.print("Error: volts reading from SDM120 is 0. MQTT publish is halted until there is valid data.");
  }

  xTimerStart(mqttPublishXTimer, 0);
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
  MQTTStatus = true;
  sendMQTTData();
}

void setup() {
  //rtc_wdt_protect_off();
  //rtc_wdt_disable();

  Serial.begin(115200);
  Serial.println("SDM120 ESP32 MQTT Node v0.9");
  sdm.begin();

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(5000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));
  mqttPublishXTimer = xTimerCreate("mqttPubTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(sendMQTTData));

  WiFi.onEvent(WiFiEvent);

  mqttClient.setCredentials(mqttUser, mqttPassword);
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);

  connectToWifi();
}

void loop() {
  unsigned long currentMillisUART = millis();
  if (currentMillisUART - previousMillisUART >= intervalUART) {
    previousMillisUART = currentMillisUART;
    
    getEverythingFromSDM120();
  
  }
}
