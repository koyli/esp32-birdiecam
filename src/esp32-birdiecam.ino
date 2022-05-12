#include "credentials.h"
#include <WiFi.h>
#include <WiFiMulti.h>
#include <ESPmDNS.h>
#include <esp32-aws-s3.h>
#include <esp32-cam.h>
#include <avisimple.h>
#include <TimeLib.h>
#include <SD_MMC.h>

#define SYSTEM "birdie"
#define RED_LIGHT_PIN GPIO_NUM_33
#define WHITE_LIGHT_PIN GPIO_NUM_4
#define PIR_PIN GPIO_NUM_12                   // for active high pir or microwave etc
#define IR_LED_PIN GPIO_NUM_13

#define SDCARD_ERROR 2
#define CAMERA_ERROR 1

RTC_DATA_ATTR unsigned int bootCount = 0;
RTC_DATA_ATTR time_t epoch_time = 0;

WiFiMulti wifiMulti;

#define FRAME_BUFFER_SIZE 30

camera_fb_t frames[FRAME_BUFFER_SIZE];

void fatal_error(int code, const char* message)
{
    for (int i = 0; i < code ; ++i) {
        digitalWrite(RED_LIGHT_PIN, LOW);
        delay(1000);
        digitalWrite(RED_LIGHT_PIN, HIGH);
        delay(1000);
    }    
    ESP.restart();
}
                 
static void initSDCard()
{
    if (!SD_MMC.begin("/sdcard", true))
        fatal_error(SDCARD_ERROR, "Could not mount SD card");
}

static void initWifi()
{
    wifiMulti.addAP(WIFI_SSID_1, WIFI_ACCESSCODE_1);
    wifiMulti.addAP(WIFI_SSID_2, WIFI_ACCESSCODE_2);
    wifiMulti.addAP(MESH_SSID_1, MESH_ACCESSCODE_1);

    unsigned long startTime = millis();
    
    while (wifiMulti.run() != WL_CONNECTED && millis() - startTime < 20000)
        {
            Serial.print(".");
            delay(500);
        }
    
    if (!MDNS.begin(SYSTEM)) 
        Serial.println("Error setting up MDNS responder!");
    

    configTime(0, 0, "pool.ntp.org");

    while(now() < 60000) {
        Serial.print("t");
        delay(100);
    }

    
}



void setup() {

    Serial.begin(115200);

    pinMode(RED_LIGHT_PIN, OUTPUT);             // little red led on back of chip
    pinMode(IR_LED_PIN, OUTPUT);
    pinMode(PIR_PIN, INPUT);
    pinMode(WHITE_LIGHT_PIN, OUTPUT);               // Blinding Disk-Avtive Light

    digitalWrite(RED_LIGHT_PIN, LOW);           // turn on the red LED on the back of chip
    digitalWrite(WHITE_LIGHT_PIN, LOW);             // turn off

    AWS_S3::setup(ACCESS_KEY, SECRET_KEY, BUCKET);

    initSDCard();
    
    ++bootCount;


    if (epoch_time > 0) {
        /* will be lower bound - better than 1970 */
        setTime(epoch_time);
    }

    if (!Camera::configure(FRAME_BUFFER_SIZE, 10, false, (framesize_t) 8 )) {
        fatal_error(CAMERA_ERROR, "Camera not initialized");
    }

    digitalWrite(RED_LIGHT_PIN, HIGH);// turn off the red LED  - ready for action
    digitalWrite(IR_LED_PIN, HIGH);

}


void loop() {}
