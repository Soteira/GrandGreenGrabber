/* 
 * Project myProject
 * Author: Your Name
 * Date: 
 * For comprehensive documentation and examples, please visit:
 * https://docs.particle.io/firmware/best-practices/firmware-template/
 */

// Include Particle Device OS APIs
#include "Particle.h"
#include "AdaFruit_VL53L0X.h"
#include "Stepper.h"
#include <Adafruit_MQTT.h>
#include "Adafruit_MQTT\Adafruit_MQTT_SPARK.h"
#include "Adafruit_MQTT\Adafruit_MQTT.h"
#include "credentials.h"

/************ Global State (you don't need to change this!) ***   ***************/ 
TCPClient TheClient; 

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details. 
Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY); 

/****************************** Feeds ***************************************/ 
// Setup Feeds to publish or subscribe 
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname> 
Adafruit_MQTT_Subscribe subFeed = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/buttonOnOff"); 
Adafruit_MQTT_Publish pubFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/randomNumber");


float pubValue;
int subValue;
unsigned int last, lastTime;
const int CLOSEBUTTON = D3;
const int SPR = 2048;
const int IN1 = D5; 
const int IN2 = D6;
const int IN3 = D7;
const int IN4 = D10;
Stepper myStepper(SPR,IN1,IN3,IN2,IN4);
int pinState;

void MQTT_connect();
bool MQTT_ping();

SYSTEM_MODE(AUTOMATIC);

// Run the application and system concurrently in separate threads
SYSTEM_THREAD(ENABLED);

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

void setup() {
  Serial.begin(115200);

  // wait until serial port opens for native USB devices
  while (! Serial) {
    delay(1);
  }
  
  Serial.println("Adafruit VL53L0X test");
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }
  // power 
  Serial.println(F("VL53L0X API Simple Ranging example\n\n")); 

  myStepper.setSpeed(10);

  pinMode(CLOSEBUTTON, OUTPUT);

    // Connect to Internet but not Particle Cloud
  WiFi.on();
  WiFi.connect();
  while(WiFi.connecting()) {
    Serial.printf(".");
  }
  Serial.printf("\n\n");

  // Setup MQTT subscription
  mqtt.subscribe(&subFeed);

}


void loop() {
  VL53L0X_RangingMeasurementData_t measure;
    
  Serial.print("Reading a measurement... ");
  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    Serial.print("Distance (mm): "); Serial.println(measure.RangeMilliMeter);
  } else {
    Serial.println(" out of range ");
  }
    
  delay(100);


 if(pinState > 0){
  myStepper.step(100);
 } else {
  myStepper.step(0);
 }

  pinState = digitalRead(CLOSEBUTTON);
  Serial.printf("The value of the button is %i \n", pinState);

  MQTT_connect();
  MQTT_ping();

  pubValue = random(10000);

   Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(10000))) {
    if (subscription == &subFeed) {
      subValue = atoi((char *)subFeed.lastread);
      Serial.printf("subValue is: %i \n", subValue);
    }
  }

    if((millis()-lastTime > 6000)) {
    if(mqtt.Update()) {
      pubFeed.publish(pubValue);
      Serial.printf("Publishing %0.2f \n",pubValue); 
       //Serial.printf("Current random number is %i \n", num);
      } 
    lastTime = millis();
  }

}

void MQTT_connect() {
  int8_t ret;
 
  // Return if already connected.
  if (mqtt.connected()) {
    return;
  }
 
  Serial.print("Connecting to MQTT... ");
 
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.printf("Error Code %s\n",mqtt.connectErrorString(ret));
       Serial.printf("Retrying MQTT connection in 5 seconds...\n");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds and try again
  }
  Serial.printf("MQTT Connected!\n");
}

bool MQTT_ping() {
  static unsigned int last;
  bool pingStatus;

  if ((millis()-last)>120000) {
      Serial.printf("Pinging MQTT \n");
      pingStatus = mqtt.ping();
      if(!pingStatus) {
        Serial.printf("Disconnecting \n");
        mqtt.disconnect();
      }
      last = millis();
  }
  return pingStatus;
}