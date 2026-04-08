#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include "MS5837.h"
#include <Wifi.h>
#include <esp_heap_caps.h>
#include <cmath>
#include <ESP32Time.h>
#include <time.h>
#include <stdio.h>
#include <ESP32Ping.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <TMC2209.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <Adafruit_ST7789.h> 
#include <Fonts/FreeSans9pt7b.h>
#include <Adafruit_GFX.h>
#include <Fonts/FreeSans12pt7b.h>
#include <Adafruit_Sensor.h>
#include <stdio.h>
#include <string>

// Analog servos run at ~50 Hz updates
#define maximumReadings 2000
#define D0 0 // button D0 on feather board
#define D1 1 // button D1 on feather board
#define D2 2 // button D2 on feather board
bool NewReading = false;
bool Logging = false;
int LoggingEnabler = 0;
bool endLog = false;
int loggingDisabler = 0;
int readingCnt = 0;
int prevReadingCnt = 0;
const char* ntpServer = "pool.ntp.org";
const int daylightOffset_sec = 3600;
const char* ssid = "SM-N950U48f"; //"Justin's S25+";
const char* password = "bucketman";//"8e9uphtuumacfst";
unsigned long currentTime = millis(); 
unsigned long previousTime = 0; 
const long timeoutTime = 2000; // Define timeout time in milliseconds (example: 2000ms = 2s)
String header;
const long gmtOffset_sec = -18000;
float depthPascal = 0;
float depthMeter = 0;
int runNum = 0;
bool diving;
int JustInCase = 0;
double pressureValueMax;
double pressureValueMin;
double deltaP;
static int lastSecond = -1;
int sec;
bool limit_hit = false;
int step_pin = 16;
int UART_RX = 17;
int UART_TX = 18;
int dir_pin = 15;
static const long SERIAL_BAUD = 9600;
unsigned long pDiveTime = 0;
unsigned long deltaT_prev;
int step_pos;
bool bouncing;
const uint8_t RUN_CURRENT_PERCENT = 100;
float lastDiveCall = millis();


void getTime();
void flashLED(int times);
//void ledTask(void*);
void dive();
void surface();
void home();
void stop();
//void flashLED_async(uint32_t flashes);
void defineHTML(void);
void step(int steps, int step_delay);
void bounce(void);
void handleWebserver(void);



typedef struct {
    int runNumber;
    int lHour;
    int lMin;
    int lSec;
  float depthPa;
  float depthM;
  char  packet;
} sReadings;

sReadings *psram_Readings;
MS5837 sensor;

extern const char index_html[] PROGMEM;


//WiFiServer server(80);
ESP32Time rtc(0);
IPAddress local_IP(192, 168, 165, 183);
IPAddress gateway(192, 168, 165, 1);
IPAddress subnet(255, 255, 0, 0);
HardwareSerial & serial_stream = Serial1;

TMC2209 stepper_driver;
Adafruit_ST7789 display = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
GFXcanvas16 canvas(240, 135);

//TaskHandle_t ledTaskHandle = nullptr;

void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

AsyncWebServer server1(80);
WiFiServer server2(8080);