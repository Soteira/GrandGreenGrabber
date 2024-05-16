/* 
 * Project myProject
 * Author: Your Name
 * Date: 
 * For comprehensive documentation and examples, please visit:
 * https://docs.particle.io/firmware/best-practices/firmware-template/
 */

// Include Particle Device OS APIs
#include "Particle.h"
#include <JsonParserGeneratorRK.h>
#include "Adafruit_GPS.h"
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
Adafruit_MQTT_Publish GPSFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/GPSCoordinates");
Adafruit_MQTT_Subscribe GreenGrabberControls = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/greengrabbercontrols");
Adafruit_MQTT_Publish TOFFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/GrabberTOF");
Adafruit_MQTT_Subscribe GreenGrabberopen = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/greengrabberopen");


float pubValue;
int subValue;
unsigned int last, lastTime;
const int CLOSEBUTTON = D4;
const int SPR = 2048;
const int IN1 = D5; 
const int IN2 = D6;
const int IN3 = D7;
const int IN4 = D10;
Stepper myStepper(SPR,IN1,IN3,IN2,IN4);
bool buttonPinState1;
bool virtualButtonState;
bool virtualButtonState2;
void createEventPayLoad(float latitude, float longitude);
int lastPublish;
const int RESETVALUE = 10000;

void getGPS(float *latitude, float *longitude, float *altitude, int *satellites);
Adafruit_GPS GPS(&Wire);

// Define Constants
const int TIMEZONE = -6;
const unsigned int UPDATE = 30000;
int OLED_RESET = -1;

// Declare Variables 
float lat, lon, alt;
int sat;
unsigned int lastGPS;

void MQTT_connect();
bool MQTT_ping();

SYSTEM_MODE(AUTOMATIC);

// Run the application and system concurrently in separate threads
SYSTEM_THREAD(ENABLED);

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
VL53L0X_RangingMeasurementData_t measure;

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

  myStepper.setSpeed(15);
  pinMode(CLOSEBUTTON, OUTPUT);

    // Connect to Internet but not Particle Cloud
  WiFi.on();
  WiFi.connect();
  while(WiFi.connecting()) {
    Serial.printf(".");
  }
  Serial.printf("\n\n");

  // Setup MQTT subscriptions
  mqtt.subscribe(&GreenGrabberControls);
  mqtt.subscribe(&GreenGrabberopen);

  //Iinitialization for the GPS signal
  GPS.begin(0x10);  // The I2C address to use is 0x10
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); 
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(1000);
  GPS.println(PMTK_Q_RELEASE);
}


void loop() {

     // Get data from GSP unit (best if you do this continuously)
  GPS.read();
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA())) {
      return;
    }   
  }
    
  MQTT_connect();
  MQTT_ping();

  if (millis() - lastGPS > UPDATE) {
    lastGPS = millis(); // reset the timer
    getGPS(&lat,&lon,&alt,&sat);
    //Serial.printf("\n=================================================================\n");
    Serial.printf("Lat: %0.6f, Lon: %0.6f, Alt: %0.6f, Satellites: %i\n",lat, lon, alt, sat);
    //Serial.printf("=================================================================\n\n");
  }

  Serial.print("Reading a measurement... ");
  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    Serial.print("Distance (mm): "); Serial.println(measure.RangeMilliMeter);
  } else {
    Serial.println(" out of range ");
  }

  buttonPinState1 = digitalRead(CLOSEBUTTON);
  if(buttonPinState1){
    myStepper.step(-1000);
    } else {
    myStepper.step(0);
  }

  Serial.printf("The value of the button is %s \n", buttonPinState1 ? "true" : "false");

  pubValue = random(100);

    // this is our 'wait for incoming subscription packets' busy subloop 
  Adafruit_MQTT_Subscribe *subscription1;
  while ((subscription1 = mqtt.readSubscription (100))) {
    if (subscription1 == &GreenGrabberControls) {
      virtualButtonState = atoi((char *)GreenGrabberControls.lastread);
      Serial.printf("subValue is: %i \n", virtualButtonState);
    }
  }

    if(virtualButtonState){
    myStepper.step(-100);
    } else {
    myStepper.step(0);
    }

    Adafruit_MQTT_Subscribe *subscription2;
  while ((subscription2 = mqtt.readSubscription (100))) {
    if (subscription2 == &GreenGrabberopen) {
      virtualButtonState2 = atoi((char *)GreenGrabberopen.lastread);
      Serial.printf("subValue is: %i \n", virtualButtonState2);
    }
  }

if (millis() - lastPublish > RESETVALUE) {
    lastPublish = millis();

    TOFFeed.publish(measure.RangeMilliMeter);
  }

    if(virtualButtonState2){
      myStepper.step(100);
    } else {
      myStepper.step(0);
    }
  
   // if((millis()-lastTime > 6000)) {
   // if(mqtt.Update()) {
    //  GPSFeed.publish(lat);
    //  Serial.printf("Publishing %0.2f \n",pubValue); 
       //Serial.printf("Current random number is %i \n", num);
    //  } 
   // lastTime = millis();
  //}

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


void getGPS(float *latitude, float *longitude, float *altitude, int *satellites){
  int theHour;

  theHour = GPS.hour + TIMEZONE;
  if(theHour < 0) {
    theHour = theHour + 24;
  }
    
  Serial.printf("Time: %02i:%02i:%02i:%03i\n",theHour, GPS.minute, GPS.seconds, GPS.milliseconds);
  Serial.printf("Dates: %02i-%02i-20%02i\n", GPS.month, GPS.day, GPS.year);
  Serial.printf("Fix: %i, Quality: %i",(int)GPS.fix,(int)GPS.fixquality);
    if (GPS.fix) {
      *latitude = GPS.latitudeDegrees;
      *longitude = GPS.longitudeDegrees; 
      *altitude = GPS.altitude;
      *satellites = (int)GPS.satellites;
      createEventPayLoad(*latitude, *longitude);
    }
}

void createEventPayLoad (float latitude, float longitude) {
JsonWriterStatic<256> jw;
  {
    JsonWriterAutoObject obj(&jw);

      jw.insertKeyValue ("lat", latitude );
      jw.insertKeyValue ("lon", longitude );
  }
  GPSFeed.publish(jw.getBuffer());
}
