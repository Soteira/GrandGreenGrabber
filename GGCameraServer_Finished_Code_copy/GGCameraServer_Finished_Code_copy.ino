#include <Arduino.h>
#include "esp_camera.h"
#include <WiFi.h>
#include "esp_timer.h"
#include "img_converters.h"
#include "Arduino.h"
#include "fb_gfx.h"
#include "soc/soc.h" //disable brownout problems
#include "soc/rtc_cntl_reg.h"  //disable brownout problems
#include "esp_http_server.h"

//Replace with your network credentials
const char* ssid = "DDCIOT";
const char* password = "ddcIOT2020";

// other camera models listed in "camera_pins.h"
// #define CAMERA_MODEL_ESP32S3_EYE
// #include "camera_pins.h"
const int ONBOARD_LED = 2;


#include "credentials.h"
const char* mqttTopicSnap = "Tigress/feeds/GrabberCam";
const char* mqttTopicStream = "Tigress/feeds/GrabberStream";
const int MQTT_BUFFERSIZE = 50 * 1024;
const int MAX_PUBLISH = 50 * 1024;    // Adafruit limit is 100 kB, not 50

// these are used for image publication to Adafruit dashboard
#include <PubSubClient.h>
#include <base64.h>

bool pin2On = false;  // used for irregular LED flashing while attempting serial connection, as serial is often not up

const int PUBLISH_DELAY = 30000;  // 5 minutes between publishing images
const int SERIAL_TIMEOUT = 0*1000;   // 0 seconds to wait for serial connection - often absent
int lastTick = 0; // for timing

WiFiClient espClient;
PubSubClient client(espClient);

void mqtt_setup();
void callback(char* topic, byte* payload, unsigned int length);

char strOut[1024];  // for Adafruit text publications (1kB limit when Dashboard control history is on)
String buffer;      // for Adafruit image publication (100kB limit when Dashboard control history is on)

#define PART_BOUNDARY "123456789000000000000987654321"

// This project was tested with the AI Thinker Model, M5STACK PSRAM Model and M5STACK WITHOUT PSRAM
#define CAMERA_MODEL_AI_THINKER
//#define CAMERA_MODEL_M5STACK_PSRAM
//#define CAMERA_MODEL_M5STACK_WITHOUT_PSRAM

// Not tested with this model
//#define CAMERA_MODEL_WROVER_KIT

#if defined(CAMERA_MODEL_WROVER_KIT)
  #define PWDN_GPIO_NUM    -1
  #define RESET_GPIO_NUM   -1
  #define XCLK_GPIO_NUM    21
  #define SIOD_GPIO_NUM    26
  #define SIOC_GPIO_NUM    27
  
  #define Y9_GPIO_NUM      35
  #define Y8_GPIO_NUM      34
  #define Y7_GPIO_NUM      39
  #define Y6_GPIO_NUM      36
  #define Y5_GPIO_NUM      19
  #define Y4_GPIO_NUM      18
  #define Y3_GPIO_NUM       5
  #define Y2_GPIO_NUM       4
  #define VSYNC_GPIO_NUM   25
  #define HREF_GPIO_NUM    23
  #define PCLK_GPIO_NUM    22

#elif defined(CAMERA_MODEL_M5STACK_PSRAM)
  #define PWDN_GPIO_NUM     -1
  #define RESET_GPIO_NUM    15
  #define XCLK_GPIO_NUM     27
  #define SIOD_GPIO_NUM     25
  #define SIOC_GPIO_NUM     23
  
  #define Y9_GPIO_NUM       19
  #define Y8_GPIO_NUM       36
  #define Y7_GPIO_NUM       18
  #define Y6_GPIO_NUM       39
  #define Y5_GPIO_NUM        5
  #define Y4_GPIO_NUM       34
  #define Y3_GPIO_NUM       35
  #define Y2_GPIO_NUM       32
  #define VSYNC_GPIO_NUM    22
  #define HREF_GPIO_NUM     26
  #define PCLK_GPIO_NUM     21

#elif defined(CAMERA_MODEL_M5STACK_WITHOUT_PSRAM)
  #define PWDN_GPIO_NUM     -1
  #define RESET_GPIO_NUM    15
  #define XCLK_GPIO_NUM     27
  #define SIOD_GPIO_NUM     25
  #define SIOC_GPIO_NUM     23
  
  #define Y9_GPIO_NUM       19
  #define Y8_GPIO_NUM       36
  #define Y7_GPIO_NUM       18
  #define Y6_GPIO_NUM       39
  #define Y5_GPIO_NUM        5
  #define Y4_GPIO_NUM       34
  #define Y3_GPIO_NUM       35
  #define Y2_GPIO_NUM       17
  #define VSYNC_GPIO_NUM    22
  #define HREF_GPIO_NUM     26
  #define PCLK_GPIO_NUM     21

#elif defined(CAMERA_MODEL_AI_THINKER)
  #define PWDN_GPIO_NUM     32
  #define RESET_GPIO_NUM    -1
  #define XCLK_GPIO_NUM      0
  #define SIOD_GPIO_NUM     26
  #define SIOC_GPIO_NUM     27
  
  #define Y9_GPIO_NUM       35
  #define Y8_GPIO_NUM       34
  #define Y7_GPIO_NUM       39
  #define Y6_GPIO_NUM       36
  #define Y5_GPIO_NUM       21
  #define Y4_GPIO_NUM       19
  #define Y3_GPIO_NUM       18
  #define Y2_GPIO_NUM        5
  #define VSYNC_GPIO_NUM    25
  #define HREF_GPIO_NUM     23
  #define PCLK_GPIO_NUM     22
#else
  #error "Camera model not selected"
#endif

void mqtt_setup();
void callback(char* topic, byte* payload, unsigned int length);

void setup() {
  Serial.begin(9600);
  pinMode(ONBOARD_LED, OUTPUT); 
  while(!Serial.available() && millis() - lastTick < SERIAL_TIMEOUT){
    digitalWrite(ONBOARD_LED, pin2On = !pin2On);
    delay(200 + 200 * (random()%2));              // random on/off delays of either 200 or 400 ms
    Serial.begin(9600);
  }
  Serial.setDebugOutput(true);
  Serial.println("Serial is up!");

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_VGA;          // limited for 100 kB Adafruit limit for publishing
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_DRAM;
  config.jpeg_quality = 10;                   // (0-63) higher numbers are lower quality
  config.fb_count = 1;

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
// adjusted for spefic use in class
  s->set_agc_gain(s, 15);   // (1 - 31)
  s->set_gain_ctrl(s, 1);   // white balance gain control (0 or 1)
  s->set_brightness(s, 2);  // (-2 to 2) set to brightest

  WiFi.begin(ssid, password);
  WiFi.setSleep(false);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
}

// publish a pic, then wait
void loop() {
  mqtt_setup();     // refresh to keep alive
  static camera_fb_t *fb = NULL;
  fb = esp_camera_fb_get();  
  if(!fb) {
    Serial.println("Camera capture failed");
    client.publish(mqttTopicStream, "Camera capture failed\0");
    return;
  }

// Adafruit insists on 64-bit encoded jpegs
  buffer = base64::encode((uint8_t *)fb->buf, fb->len);
  sprintf(strOut, "Got frame: %d x %d (%d/%d)\0", fb->width, fb->height, fb->len, buffer.length());
  client.publish(mqttTopicStream, strOut);
  Serial.printf("%s\n", strOut);
  if (buffer.length() < MAX_PUBLISH) {
    if (client.publish(mqttTopicSnap, buffer.c_str()))
    {
      Serial.print("Published Image to ");
      Serial.print(mqttTopicSnap);
      Serial.printf("\n");
      client.publish(mqttTopicStream, "Published!\0");
      Serial.printf("Published %d bytes (from %d)\n", buffer.length(), fb->len);
    }
    else
    {
      Serial.println("Error Publishing Image");
      client.publish(mqttTopicStream, "Error publishing...\0");
    }
  } else {
    client.publish(mqttTopicStream, "Over limit - We'll try to publish the next pic\0");
  }

  esp_camera_fb_return(fb);
  buffer.clear();

  delay(PUBLISH_DELAY);     // use a delay here - time since last attempt ended, not started
}

//------------------ MQTT ----------------------------------
void mqtt_setup() {
// Adafruit publication limit is 100kB with Dashboard control history off
// Buffersize is set lower based on successful publish history
  client.setBufferSize((uint16_t)MQTT_BUFFERSIZE);
  client.setServer(AIO_SERVER, AIO_SERVERPORT);
  client.setCallback(callback);
  Serial.println("Connecting to MQTT…");
  while (!client.connected()) {        
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    if (client.connect(clientId.c_str(), AIO_USERNAME, AIO_KEY)) {
        Serial.println("connected");
    } else {
        Serial.print("failed with state  ");
        Serial.println(client.state());
        delay(2000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {

    Serial.print("Message arrived in topic: ");
    Serial.println(topic);

    String byteRead = "";
    Serial.print("Message: ");
    for (int i = 0; i < length; i++) {
        byteRead += (char)payload[i];
    }    
    Serial.println(byteRead);
}
