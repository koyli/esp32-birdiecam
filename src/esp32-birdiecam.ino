#include "credentials.h"
#include <WiFi.h>
#include <WiFiMulti.h>
#include <ESPmDNS.h>
#include <esp32-aws-s3.h>
#include <esp32-cam.h>
#include <avisimple.h>
#include <TimeLib.h>
#include <SD_MMC.h>
#include <pthread.h>
#include <mutex>
#include <condition_variable>

#define SYSTEM "birdie"
#define RED_LIGHT_PIN GPIO_NUM_33
#define WHITE_LIGHT_PIN GPIO_NUM_4
#define PIR_PIN GPIO_NUM_12                   // for active high pir or microwave etc
#define IR_LED_PIN GPIO_NUM_13

#define SDCARD_ERROR 2
#define CAMERA_ERROR 1

std::mutex sd_mutex;
std::condition_variable cond;

String filename;

RTC_DATA_ATTR unsigned int bootCount = 0;
RTC_DATA_ATTR time_t epoch_time = 0;

WiFiMulti wifiMulti;

#define FRAME_BUFFER_SIZE 8
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


void printTime()
{
    struct tm timeinfo;
    if(!getLocalTime(&timeinfo)){
        return;
    }
    Serial.println(&timeinfo, "%Y-%m-%d %H:%M:%S");
}

String timeString ()
{
    struct tm timeinfo;
    getLocalTime(&timeinfo);
    char buffer[255];
    strftime(buffer, sizeof(buffer), "%Y%m%dT%H%M%S", &timeinfo);
    return String(buffer);
}



static void initSDCard()
{
    
    /* this pull up necessary in single line mode - certainly for 3.3v flashed esp32  */
    pinMode(GPIO_NUM_13, INPUT_PULLUP);

    if (!SD_MMC.begin("/sdcard", true))
        fatal_error(SDCARD_ERROR, "Could not mount SD card");
}

static void initWifi()
{

    Serial.print("Initializing Wifi..");
    wifiMulti.addAP(WIFI_SSID_1, WIFI_ACCESSCODE_1);
    wifiMulti.addAP(WIFI_SSID_2, WIFI_ACCESSCODE_2);
    wifiMulti.addAP(MESH_SSID_1, MESH_ACCESSCODE_1);
    
    unsigned long startTime = millis();
    
    while (wifiMulti.run() != WL_CONNECTED && millis() - startTime < 20000)
        {
            Serial.print(".");
            delay(500);
        }
    if (wifiMulti.run() == WL_CONNECTED)
        Serial.println(" connected\n");
    
    if (!MDNS.begin(SYSTEM)) 
        Serial.println("Error setting up MDNS responder!");
    
    printTime();
    
    configTime(0, 0, "pool.ntp.org");

    while(time(nullptr) < 60000) {
        delay(500);             /* give time to come up */
    }
    
}


//TaskHandle_t uploadTask;
pthread_t uploader;
pthread_t video;

bool video_ready = false;

#define FRAMES 300
#define FRAME_RATE 30
#define FRAME_TIME (1000/FRAME_RATE)

void* video_loop(void *param)
{
    for (;;) {
        int frames = FRAMES;
        while (frames > 0) {
            int t = millis();

            camera_fb_t* fr = Camera::getFrame();

            AviFileWriter::addFrame(fr->buf, fr->len);
            Serial.println("Wrote frame");
            Camera::returnFrame(fr);

            int elapsed = millis() - t;
            if (elapsed < FRAME_TIME)
                delay(FRAME_TIME - elapsed);

        }
            
        AviFileWriter::closeAvi();
        {
            std::lock_guard<std::mutex> lk(sd_mutex);
            video_ready = true;
        }
        cond.notify_all();
        break;
    }
    return NULL;
}

void* upload_loop(void *param)
{
    for (;;) {
        {
            std::unique_lock<std::mutex> lk(sd_mutex);
            
            cond.wait(lk, []{ return video_ready; });
            Serial.println();
            
            Serial.print("file uploaded");

            File fd = SD_MMC.open(filename, FILE_READ);

            lk.unlock();
            AWS_S3::put(filename, fd);

            Serial.print("file uploaded");
        }

    }
    return NULL;
        
}


void setup() {

    Serial.begin(115200);

    pinMode(RED_LIGHT_PIN, OUTPUT);             // little red led on back of chip
    pinMode(IR_LED_PIN, OUTPUT);
    pinMode(PIR_PIN, INPUT);
    pinMode(WHITE_LIGHT_PIN, OUTPUT);               // Blinding Disk-Avtive Light

    digitalWrite(RED_LIGHT_PIN, LOW);           // turn on the red LED on the back of chip
    digitalWrite(WHITE_LIGHT_PIN, LOW);             // turn off

    initSDCard();
    initWifi();

    ++bootCount;

    Serial.print("Boot count: ");
    Serial.println( bootCount);
    AWS_S3::setup(ACCESS_KEY, SECRET_KEY, BUCKET);

    if (epoch_time > 0) {
        /* will be lower bound - better than 1970 */
        setTime(epoch_time);
    }

    if (!Camera::configure(FRAME_BUFFER_SIZE, 10, false, (framesize_t) FRAMESIZE_VGA )) {
        fatal_error(CAMERA_ERROR, "Camera not initialized");
    }

    digitalWrite(RED_LIGHT_PIN, HIGH);// turn off the red LED  - ready for action
    digitalWrite(IR_LED_PIN, HIGH);

    filename = String("/") + String(SYSTEM) + String("-") + timeString() + String(".avi");
    
    AviFileWriter::init_avi(filename.c_str(), 640,480, 10, YUYV);
    AviFileWriter::writeHeader();

    pthread_create(&uploader, NULL, upload_loop, NULL);
    pthread_create(&video, NULL, video_loop, NULL);
}


void loop() {

}
