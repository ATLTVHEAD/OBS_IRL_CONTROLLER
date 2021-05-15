  /*
  OBS_MQTT_BRACER.ino
  WRITTEN: Nate Damen
  CONACT: nate.damen@gmail.com
  DATE: 2/1/21
  Update: 3/16/21

  PURPOSE: send out button counts through mqtt.

  Features:
    -send glove state as button total counts out over mqtt
    -single and multi button press (B1+B2)
    -sends IMU data over mqtt every 41ms

  WORK ON NEXT: allow for double click by playing with button debounce and button capture window
  
*/

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
  bool pressed;
};

Button button1 = {13, 0, false};
Button button2 = {12, 0, false};
Button button3 = {27, 0, false};
Button button4 = {33, 0, false};
Button button5 = {15, 0, false};
Button button6 = {32, 0, false};
Button button7 = {14, 0, false};
Button button8 = {26, 0, false};
Button button9 = {25, 0, false};
Button button10 = {21, 0, false};
Button button11 = {17, 0, false};
Button button12 = {16, 0, false};
Button button13 = {19, 0, false};
Button button14 = {18, 0, false};
Button button15 = {5, 0, false};

int button_debounce = 210;
int button_double_delay = 150;

unsigned long Binterrupt_time = 0;
unsigned long Blast_interrupt_time = 0;

unsigned long accel_timer = 0;
unsigned long last_accel_timer = 0;
int accel_delay = 16; //16~=60hz 41~=24hz

void IRAM_ATTR isr() {
  static unsigned long last_interrupt_time = 0;
   unsigned long interrupt_time = millis();
 // If interrupts come faster than 350ms, assume it's a bounce and ignore
 if (interrupt_time - last_interrupt_time > button_debounce)
 {
  button1.numberKeyPresses += 1;
  button1.pressed = true;
 }
 
 last_interrupt_time = interrupt_time;
}

void IRAM_ATTR isr2() {
  static unsigned long last_interrupt_time = 0;
   unsigned long interrupt_time = millis();
 // If interrupts come faster than 350ms, assume it's a bounce and ignore
 if (interrupt_time - last_interrupt_time > button_debounce)
 {
  button2.numberKeyPresses += 1;
  button2.pressed = true;
 }
 last_interrupt_time = interrupt_time;
}

void IRAM_ATTR isr3() {
  static unsigned long last_interrupt_time = 0;
   unsigned long interrupt_time = millis();
 // If interrupts come faster than 350ms, assume it's a bounce and ignore
 if (interrupt_time - last_interrupt_time > button_debounce)
 {
  button3.numberKeyPresses += 1;
  button3.pressed = true;
 }
 last_interrupt_time = interrupt_time;
}

void IRAM_ATTR isr4() {
  static unsigned long last_interrupt_time = 0;
   unsigned long interrupt_time = millis();
 // If interrupts come faster than 350ms, assume it's a bounce and ignore
 if (interrupt_time - last_interrupt_time > button_debounce)
 {
  button4.numberKeyPresses += 1;
  button4.pressed = true;
 }
 last_interrupt_time = interrupt_time;
}

void IRAM_ATTR isr5() {
  static unsigned long last_interrupt_time = 0;
   unsigned long interrupt_time = millis();
 // If interrupts come faster than 350ms, assume it's a bounce and ignore
 if (interrupt_time - last_interrupt_time > button_debounce)
 {
  button5.numberKeyPresses += 1;
  button5.pressed = true;
 }
 last_interrupt_time = interrupt_time;
}

void IRAM_ATTR isr6() {
  static unsigned long last_interrupt_time = 0;
   unsigned long interrupt_time = millis();
 // If interrupts come faster than 350ms, assume it's a bounce and ignore
 if (interrupt_time - last_interrupt_time > button_debounce)
 {
  button6.numberKeyPresses += 1;
  button6.pressed = true;
 }
 last_interrupt_time = interrupt_time;
}

void IRAM_ATTR isr7() {
  static unsigned long last_interrupt_time = 0;
   unsigned long interrupt_time = millis();
 // If interrupts come faster than 350ms, assume it's a bounce and ignore
 if (interrupt_time - last_interrupt_time > button_debounce)
 {
  button7.numberKeyPresses += 1;
  button7.pressed = true;
 }
 last_interrupt_time = interrupt_time;
}

void IRAM_ATTR isr8() {
  static unsigned long last_interrupt_time = 0;
   unsigned long interrupt_time = millis();
 // If interrupts come faster than 350ms, assume it's a bounce and ignore
 if (interrupt_time - last_interrupt_time > button_debounce)
 {
  button8.numberKeyPresses += 1;
  button8.pressed = true;
 }
 last_interrupt_time = interrupt_time;
}

void IRAM_ATTR isr9() {
  static unsigned long last_interrupt_time = 0;
   unsigned long interrupt_time = millis();
 // If interrupts come faster than 350ms, assume it's a bounce and ignore
 if (interrupt_time - last_interrupt_time > button_debounce)
 {
  button9.numberKeyPresses += 1;
  button9.pressed = true;
 }
 last_interrupt_time = interrupt_time;
}

void IRAM_ATTR isr10() {
  static unsigned long last_interrupt_time = 0;
   unsigned long interrupt_time = millis();
 // If interrupts come faster than 350ms, assume it's a bounce and ignore
 if (interrupt_time - last_interrupt_time > button_debounce)
 {
  button10.numberKeyPresses += 1;
  button10.pressed = true;
 }
 last_interrupt_time = interrupt_time;
}

void IRAM_ATTR isr11() {
  static unsigned long last_interrupt_time = 0;
   unsigned long interrupt_time = millis();
 // If interrupts come faster than 350ms, assume it's a bounce and ignore
 if (interrupt_time - last_interrupt_time > button_debounce)
 {
  button11.numberKeyPresses += 1;
  button11.pressed = true;
 }
 last_interrupt_time = interrupt_time;
}

void IRAM_ATTR isr12() {
  static unsigned long last_interrupt_time = 0;
   unsigned long interrupt_time = millis();
 // If interrupts come faster than 350ms, assume it's a bounce and ignore
 if (interrupt_time - last_interrupt_time > button_debounce)
 {
  button12.numberKeyPresses += 1;
  button12.pressed = true;
 }
 last_interrupt_time = interrupt_time;
}

void IRAM_ATTR isr13() {
  static unsigned long last_interrupt_time = 0;
   unsigned long interrupt_time = millis();
 // If interrupts come faster than 350ms, assume it's a bounce and ignore
 if (interrupt_time - last_interrupt_time > button_debounce)
 {
  button13.numberKeyPresses += 1;
  button13.pressed = true;
 }
 last_interrupt_time = interrupt_time;
}

void IRAM_ATTR isr14() {
  static unsigned long last_interrupt_time = 0;
   unsigned long interrupt_time = millis();
 // If interrupts come faster than 350ms, assume it's a bounce and ignore
 if (interrupt_time - last_interrupt_time > button_debounce)
 {
  button14.numberKeyPresses += 1;
  button14.pressed = true;
 }
 last_interrupt_time = interrupt_time;
}

void IRAM_ATTR isr15() {
  static unsigned long last_interrupt_time = 0;
   unsigned long interrupt_time = millis();
 // If interrupts come faster than 350ms, assume it's a bounce and ignore
 if (interrupt_time - last_interrupt_time > button_debounce)
 {
  button15.numberKeyPresses += 1;
  button15.pressed = true;
 }
 last_interrupt_time = interrupt_time;
}
 
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
  pinMode(button1.PIN, INPUT_PULLUP);
  attachInterrupt(button1.PIN, isr, FALLING);
  pinMode(button2.PIN, INPUT_PULLUP);
  attachInterrupt(button2.PIN, isr2, FALLING);
  pinMode(button3.PIN, INPUT_PULLUP);
  attachInterrupt(button3.PIN, isr3, FALLING);
  pinMode(button4.PIN, INPUT_PULLUP);
  attachInterrupt(button4.PIN, isr4, FALLING);
  pinMode(button5.PIN, INPUT_PULLUP);
  attachInterrupt(button5.PIN, isr5, FALLING);
  pinMode(button6.PIN, INPUT_PULLUP);
  attachInterrupt(button6.PIN, isr6, FALLING);
  pinMode(button7.PIN, INPUT_PULLUP);
  attachInterrupt(button7.PIN, isr7, FALLING);
  pinMode(button8.PIN, INPUT_PULLUP);
  attachInterrupt(button8.PIN, isr8, FALLING);
  pinMode(button9.PIN, INPUT_PULLUP);
  attachInterrupt(button9.PIN, isr9, FALLING);
  pinMode(button10.PIN, INPUT_PULLUP);
  attachInterrupt(button10.PIN, isr10, FALLING);
  pinMode(button11.PIN, INPUT_PULLUP);
  attachInterrupt(button11.PIN, isr11, FALLING);
  pinMode(button12.PIN, INPUT_PULLUP);
  attachInterrupt(button12.PIN, isr12, FALLING);
  pinMode(button13.PIN, INPUT_PULLUP);
  attachInterrupt(button13.PIN, isr13, FALLING);
  pinMode(button14.PIN, INPUT_PULLUP);
  attachInterrupt(button14.PIN, isr14, FALLING);
  pinMode(button15.PIN, INPUT_PULLUP);
  attachInterrupt(button15.PIN, isr15, FALLING);
  
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
}

void loop()
{
  client.loop(); 
  button_handler();
  gesture_handler();
}
  
void button_handler(){
  Binterrupt_time = millis();
  if(Binterrupt_time - Blast_interrupt_time > button_double_delay){
    if (button1.pressed || button2.pressed || button3.pressed || button4.pressed || button5.pressed || button6.pressed || button7.pressed || button8.pressed || button9.pressed || button10.pressed || button11.pressed || button12.pressed || button13.pressed || button14.pressed || button15.pressed) {
     

      //doc["b1B"]=button1.pressed;
      doc["b1"]=button1.numberKeyPresses;
      //doc["b2B"]=button12.pressed;
      doc["b2"]=button2.numberKeyPresses;
      //doc["b3B"]=button3.pressed;
      doc["b3"]=button3.numberKeyPresses;
      //doc["b4B"]=button4.pressed;
      doc["b4"]=button4.numberKeyPresses;
      //doc["b5B"]=button5.pressed;
      doc["b5"]=button5.numberKeyPresses;
      //doc["b6B"]=button6.pressed;
      doc["b6"]=button6.numberKeyPresses;
      //doc["b7B"]=button7.pressed;
      doc["b7"]=button7.numberKeyPresses;
      //doc["b8B"]=button8.pressed;
      doc["b8"]=button8.numberKeyPresses;
      //doc["b9B"]=button9.pressed;
      doc["b9"]=button9.numberKeyPresses;
      //doc["b10B"]=button10.pressed;
      doc["b10"]=button10.numberKeyPresses;
      //doc["b11B"]=button11.pressed;
      doc["b11"]=button11.numberKeyPresses;
      //doc["b12B"]=button12.pressed;
      doc["b12"]=button12.numberKeyPresses;
      //doc["b13B"]=button13.pressed;
      doc["b13"]=button13.numberKeyPresses;
      //doc["b14B"]=button14.pressed;
      doc["b14"]=button14.numberKeyPresses;
      //doc["b15B"]=button15.pressed;
      doc["b15"]=button15.numberKeyPresses;
      
      char buf[256];
      size_t n = serializeJson(doc, buf);
      
      client.publish("button_glove", buf, n);
      //Serial.println(n);
      button1.pressed = false;
      button2.pressed = false;
      button3.pressed = false;
      button4.pressed = false;
      button5.pressed = false;
      button6.pressed = false;
      button7.pressed = false;
      button8.pressed = false;
      button9.pressed = false;
      button10.pressed = false;
      button11.pressed = false;
      button12.pressed = false;
      button13.pressed = false;
      button14.pressed = false;
      button15.pressed = false;

      
      
    }
  Blast_interrupt_time = Binterrupt_time;
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