#ifndef MAIN_HPP
#define MAIN_HPP

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
#include <string>
#include <ESP_FlexyStepper.h>
#include <AutoPID.h>
#include <WebHandler.hpp>
#include <DataLogging.hpp>

// ------------ Pin Definitions ------------
#define maximumReadings 2000
#define D0 0 // button D0 on feather board
#define D1 1 // button D1 on feather board
#define D2 2 // button D2 on feather board
// int UART_RX = 18;
// int UART_TX = 17;

// bool NewReading = false;
// bool Logging = false;
// int LoggingEnabler = 0;
// bool endLog = false;
// int loggingDisabler = 0;
int readingCnt = 0;
// int prevReadingCnt = 0;

int runNum = 0;
bool diving = false;
int lastSecond = -1;

// static const long SERIAL_BAUD = 9600;
unsigned long pDiveTime = 0;

// const uint8_t RUN_CURRENT_PERCENT = 100;
// float lastDiveCall = millis();
// int readings;

// ------------ Wireless and Webserver Variables ------------
const char* ssid = "SM-N950U48f"; //"Justin's S25+";
const char* password = "bucketman";//"8e9uphtuumacfst";
const char* hostname = "float-esp32";
const char* ntpServer = "pool.ntp.org";
const int daylightOffset_sec = 3600;
const long gmtOffset_sec = -18000;
extern const char index_html[] PROGMEM;
// Static IP configuration 
IPAddress local_IP(192, 168, 165, 183);
IPAddress gateway(192, 168, 165, 1);
IPAddress subnet(255, 255, 0, 0);
AsyncWebServer server1(80);


// ------------ Stepper Variables ------------
bool limit_hit = false;     // Flag to indicate if limit has been hit
bool home_status = false;   // Flag to indicate if homing was successful

int step_pos = 0;           // Current step position
int step_pos_max = 54000;   // Maximum step position (corresponding to fully extended plunger)
int step_pos_min = 0;       // Minimum step position (corresponding to fully retracted plunger)
ESP_FlexyStepper stepper;   // Stepper object

#if defined(ESP32_S2)
int step_pin = 16;          // Pin connected to step signal of stepper driver
int dir_pin = 15;           // Pin connected to direction signal of stepper driver
int limit_switch_pin = 6;    // Pin connected to limit switch (configured with pull-up resistor, so HIGH when not triggered, LOW when triggered)
#elif defined(ESP32_S3)
int step_pin = 12;          // Pin connected to step signal of stepper driver
int dir_pin = 11;           // Pin connected to direction signal of stepper driver
int limit_switch_pin = 6;    // Pin connected to limit switch (configured with pull-up resistor, so HIGH when not triggered, LOW when triggered)
#endif


// ------------ Sensor Variables ------------
MS5837 sensor;              // Create an instance of the depth sensor
bool recordData = false;    // Flag to control when to record data (e.g., only during a dive)
double previousDepth = 0.0; // Variable to store the previous depth reading for filtering purposes
double pressureValueMax, pressureValueMin;
float depthPascal = 0.0, depthMeter = 0.0;
float allowed_depth_delta = 0.5; // maximum allowed change in depth between readings (in meters) to filter out erratic measurements
float depthOffset = 0.0; // Offset to apply to depth readings for calibration purposes (in meters)
sReadings *psram_Readings;


// ------------ Display Variables ------------
Adafruit_ST7789 display = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
GFXcanvas16 canvas(240, 135);

// ------------ Setpoint Variables ------------
double arrivalTime = NAN, packet_count = 0;   // TODO: validate packet_count
int setpoint_index = 0, loiter_time_sec = 10; // TODO: remove loiter time?
float setpoint_margin = 0.05; // meters
float setpoint[] = {0.45, 0.25, 0.15, 0.0};

// ------------ PID Variables ------------
double control_plant, control_setpoint, control_output, outputMin, outputMax = 0.0;
double Kp, Ki, Kd = 0.0;
uint32_t control_loop_rate_ms = 500; // How often to run control loop in milliseconds (e.g. 100 ms = 10 Hz)
AutoPID BangBangBoi(&control_plant, &control_setpoint, &control_output, 
  outputMin, outputMax, Kp, Ki, Kd);


// ------------ Function declarations ------------

/**
 * @brief Connects to WiFi, configures NTP, and synchronizes the RTC with local time.
 * This function first attempts to connect to the specified WiFi network, then checks for internet connectivity by pinging a known server. 
 * Once internet access is confirmed, it configures the NTP client to synchronize the RTC with the correct local time, accounting for GMT offset 
 * and daylight savings time.
 */
void getTime();

/**
 * @brief Flashes the built-in LED a specified number of times.
 * Flashes the built-in LED on and off for 1ms with a short delay of 500us before each flash, 
 * repeating for the number of times specified by the `times` parameter.
 * @param times The number of times to flash the LED.
 */
void flashLED(int times);

/**
 * @brief Initiates the diving sequence.
 * Sets the diving flag, records the dive start time, and increments the run number.
 * This function is called when the dive command is received, and it prepares the system 
 * for a new dive by updating relevant state variables.
 */
void dive();

// void surface();
// void stop();
//void flashLED_async(uint32_t flashes);
// void step(int steps, int step_delay);
void handleWebserver(void);

/**
 * @brief Filters input depth values to reject erratic measurements.
 * 
 * Compares the new depth value against the previously recorded depth.
 * If the change exceeds 1 meter, the new value is rejected and the previous
 * depth is retained. Otherwise, the new value is accepted and stored.
 * 
 * @param depthValue The new depth measurement to filter
 */
void filterInput(double depthValue);

/**
 * @brief Check and constrain a proposed relative move for the stepper motor.
 *
 * This function determines whether a requested relative movement (in steps)
 * would move the stepper beyond configured limits or trigger the endstop.
 * If the move would exceed the allowed range, the stepper target is set to
 * the corresponding limit (step_pos_max or step_pos_min). If the move is
 * within bounds, it is applied as a relative move.
 *
 * @param relativeMove The requested movement relative to the current position (in steps).
 */
void limitCheck(double absoluteTarget);

/**
 * @brief Traverse through a list of setpoints.
 *
 * This function manages the logic for moving the stepper motor to each setpoint
 * in the sequence, handling arrival detection, loitering, and transition to the next setpoint.
 */
void traverseSetpoints();

/**
 * @brief Initiates the surfacing sequence and appropriate flashing of the screen color.
 * This function is called when the final setpoint is reached, signaling the end of the dive.
 * It loops between two colors on the display to indicate surfacing and commands the stepper motor to move to the maximum position.
 */
void surface();
// ------------ ESP Variables ------------
ESP32Time rtc(0);
HardwareSerial & serial_stream = Serial1;

#endif