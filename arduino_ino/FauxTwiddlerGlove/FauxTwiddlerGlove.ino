  /*
  OBS_MQTT_BRACER.ino
  WRITTEN: Nate Damen
  CONACT: nate.damen@gmail.com
  DATE: 5/13/21
  Update: 

  PURPOSE: send out button counts through mqtt.

  Features:
    -send glove state as button total counts out over mqtt
    -single and multi button press (B1+B2)
    -sends IMU data over mqtt every 41ms
  
*/
#include <Bounce2.h>
#include "EspMQTTClient.h"
#include "cred.h"
#include <Arduino.h>
#include <ArduinoJson.h>
#include <Adafruit_LSM6DSOX.h>

Adafruit_LSM6DSOX sox;

const float gyrocal[3]={0.00904999,-0.01169999,-0.00905}; // rads/s
const float altgyrocal[3]={0.00904999,-0.01169999,-0.0035}; // rads/s
const float magcal[3]={1.35999999,-22.63,69.08};  // utesla

const int capacity = JSON_OBJECT_SIZE(15); 
StaticJsonDocument<capacity> doc;

StaticJsonDocument<capacity> doc2;

struct Button {
  const uint8_t PIN;
  uint32_t numberKeyPresses;
  bool bpressed;
};

struct Glove {
  bool gbpressed[11];
  bool gbreleased[11];
  int gbcount[11];
};
//Glove glove;
bool glove_ready=true;
bool bpressed[11];
bool breleased[11];
//int bcount[11]={0,0,0,0,0,0,0,0,0,0,0};

#define NUM_BUTTONS 11
const uint8_t BUTTON_PINS[NUM_BUTTONS] = {26, 25, 32, 14, 21, 17, 5, 18, 19, 16, 33};
Bounce * buttons = new Bounce[NUM_BUTTONS];


unsigned long accel_timer = 0;
unsigned long last_accel_timer = 0;
int accel_delay = 16; //16~=60hz 41~=24hz


 
char input;
char imu_input;

EspMQTTClient client(
  SSID1,
  PASSWORD1,
  MQTTBROKER,  // MQTT Broker server ip
  "TestESP32",     // Client name that uniquely identify your device
  PORT
);

void setup()
{
  // Button uses the built-in pull up register.
  for (int i = 0; i < NUM_BUTTONS; i++) {
    buttons[i].attach( BUTTON_PINS[i] , INPUT_PULLUP  );       //setup the bounce instance for the current button
    buttons[i].interval(25);              // interval in ms
  }
 
  Serial.begin(115200);
  delay(100);
  if (!sox.begin_I2C()) {
    while (1) {
      Serial.println("stuck");
      delay(20);
    }
  }
  Serial.println("LSM6DSOX Found!");
    
  // Optionnal functionnalities of EspMQTTClient : 
  client.enableDebuggingMessages(); // Enable debugging messages sent to serial output
  client.enableHTTPWebUpdater(); // Enable the web updater. User and password default to values of MQTTUsername and MQTTPassword. These can be overrited with enableHTTPWebUpdater("user", "password").
  client.enableLastWillMessage("TestClient/lastwill", "I am going offline");  // You can activate the retain flag by setting the third parameter to true
  client.setMaxPacketSize(256);

  
}

// This function is called once everything is connected (Wifi and MQTT)
// WARNING : YOU MUST IMPLEMENT IT IF YOU USE EspMQTTClient
void onConnectionEstablished()
{
  // Subscribe to "mytopic/test" and display received message to Serial
  client.subscribe("stream_status", [](const String & payload) {
    Serial.println(payload);
  });
  // Subscribe to "mytopic/test" and display received message to Serial
  client.subscribe("obs_webS_Connection", [](const String & payload) {
    Serial.println(payload);
  });
  // Subscribe to "mytopic/test" and display received message to Serial
  //client.subscribe("viewer_click", [](const String & payload) {
  //  Serial.println(payload);
  //});

  // Subscribe to "mytopic/wildcardtest/#" and display received message to Serial
  //client.subscribe("mytopic/wildcardtest/#", [](const String & topic, const String & payload) {
  //  Serial.println("(From wildcard) topic: " + topic + ", payload: " + payload);
  //});

  // Publish a message to "mytopic/test"
  client.publish("controller_glove", "alive"); // You can activate the retain flag by setting the third parameter to true
  // Publish a message to "mytopic/test"
  client.publish("obs_glove/IMU", "start");

  // Execute delayed instructions
  //client.executeDelayed(5 * 1000, []() {
  //  client.publish("mytopic/wildcardtest/test123", "This is a message sent 5 seconds later");
  //});

  //Set all buttons to false. The connection to mqtt seems to disrupt some of the buttons
  for (int i = 0; i < NUM_BUTTONS; i++)  {
    bpressed[i] = false;
    breleased[i] = false;
  }
}

void loop()
{
  client.loop(); 
  button_updater();
  button_sender();
  gesture_handler();
}

void button_sender(){
  if(glove_ready==true){
    //Serial.println("glove_ready");
    for (int i = 0; i < NUM_BUTTONS; i++)  {
      //Serial.println(breleased[i]);
      doc["b" + String(i+1)]=breleased[i];
      bpressed[i] = false;
      breleased[i] = false;
    }
    char buf[256];
    size_t n = serializeJson(doc, buf);  
    client.publish("button_glove", buf, n);

    glove_ready=false;
  }
}

void button_updater(){
    for (int i = 0; i < NUM_BUTTONS; i++)  {
    // Update the Bounce instance :
    buttons[i].update();
    if ( buttons[i].fell() ) {
      bpressed[i] = true;
      breleased[i] = false;
    }
    if ( buttons[i].rose() ) {
      breleased[i] = true;
      //bcount[i]++;
    }
  }
  
  for (int i = 0; i < NUM_BUTTONS; i++)  {
    if(bpressed[i] != breleased[i]){
      glove_ready=false;
      break;
    }
    else if(bpressed[i] != false){glove_ready=true;}
  }  
}

void gesture_handler(){
  accel_timer = millis();
  if(accel_timer - last_accel_timer > accel_delay){
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;
    sox.getEvent(&accel, &gyro, &temp);


    /* Display the results (acceleration is measured in m/s^2) */
    doc2["accel_x"] = accel.acceleration.x;
    doc2["accel_y"] = accel.acceleration.y;
    doc2["accel_z"] = accel.acceleration.z;

    /* Display the results (rotation is measured in rad/s) */
    doc2["gyro_x"] = gyro.gyro.x - gyrocal[0];
    doc2["gyro_y"] = gyro.gyro.y - gyrocal[1];
    doc2["gyro_z"] = gyro.gyro.z - gyrocal[2];

    // Covert to json and send over mqtt!

    char buf2[256];
    size_t n = serializeJson(doc2, buf2);

    client.publish("obs_glove/IMU", buf2);
    last_accel_timer = accel_timer;
  }
  
  
}