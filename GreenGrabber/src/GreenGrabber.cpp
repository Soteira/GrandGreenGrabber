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

const int CLOSEBUTTON = D3;
const int SPR = 2048;
const int IN1 = D5; 
const int IN2 = D6;
const int IN3 = D7;
const int IN4 = D10;
Stepper myStepper(SPR,IN1,IN3,IN2,IN4);
int pinState;

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

}