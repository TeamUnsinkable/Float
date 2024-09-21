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
//#include "Servo.h"
#include <Adafruit_PWMServoDriver.h>



#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
#define maximumReadings 200
bool NewReading = false;
bool Logging = false;
int LoggingEnabler = 0;
bool endLog = false;
int loggingDisabler = 0;
int readingCnt = 0;
int prevReadingCnt = 0;
const char* ntpServer = "pool.ntp.org";
const int daylightOffset_sec = 3600;
const char* ssid = "SM-N950U48f";
const char* password = "bucketman";
unsigned long currentTime = millis(); 
unsigned long previousTime = 0; 
const long timeoutTime = 2000; // Define timeout time in milliseconds (example: 2000ms = 2s)
String header;
const long gmtOffset_sec = -18000;
float depthPascal;
float depthMeter;
int runNum = 0;
bool diving;
bool atBottom = false;
int JustInCase = 0;
double pressureValueMax;
double pressureValueMin;
const uint8_t servonum = 0;


void getTime();
void flashLED(int times);
void dive();


typedef struct {
    int runNumber;
    int lHour;
    int lMin;
    int lSec;
  float depthPa;
  float depthM;
} sReadings;

sReadings *psram_Readings;
//Servo myservo = Servo();
MS5837 sensor;
WiFiServer server(80);
ESP32Time rtc(0);
IPAddress local_IP(192, 168, 165, 183);
IPAddress gateway(192, 168, 165, 1);
IPAddress subnet(255, 255, 0, 0);
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();



void setup() {
//  if (!WiFi.config(local_IP, gateway, subnet)) {\]
//  Serial.println("STA Failed to configure");
//}

pinMode(LED_BUILTIN, OUTPUT);
psram_Readings = (sReadings *)ps_malloc(maximumReadings * sizeof(sReadings)); 
        if(psramInit()){
        Serial.println("\nPSRAM is correctly initialized");
        }else{
        Serial.println("PSRAM not available");
        }

  // initialize USB serial converter so we have a port created
   Serial.begin(115200);
    //while (! Serial) delay(10);
  
    delay(100);

    Serial.println("Begin chooch");
  Wire.begin();
  //pwm.begin();
  //pwm.setOscillatorFrequency(27000000);
  //pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  delay(10);
  // Initialize pressure sensor
  // Returns true if initialization was successful
  // We can't continue with the rest of the program unless we can initialize the sensor
  //while (!sensor.init()) {
   //Serial.println("Init failed!");
   //Serial.println("Are SDA/SCL connected correctly?");
   //Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
   //Serial.println("\n\n\n");
   //elay(5000);

  //sensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)
    
//}
getTime();
}

void loop() {
  //sensor.read();
  //depthPascal = sensor.pressure();
  //depthMeter = sensor.depth();
  //Serial.print("PN06   ");
  //Serial.print(rtc.getTime("%H:%M:%S"));
  //Serial.print(" EST  ");
  //Serial.print(depthPascal/10.0f);
  //Serial.print("kPa  ");
  //Serial.print(depthMeter);
  //Serial.println(" meters");
  psram_Readings[readingCnt].runNumber = runNum;
  psram_Readings[readingCnt].depthPa = depthPascal/10.0f;        // Units °C
  psram_Readings[readingCnt].depthM = depthMeter;      // Units % RH
  psram_Readings[readingCnt].lHour = rtc.getHour();       // current hour
  psram_Readings[readingCnt].lMin = rtc.getMinute();     // current minute
  psram_Readings[readingCnt].lSec = rtc.getSecond();     // current second
  readingCnt++;
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(500);              // wait for a second
  digitalWrite(LED_BUILTIN, LOW);
 // for (int r = 0; r < readingCnt; r++){
       // Now output readings in CSV format to the serial port
   //     Serial.println("Profile#: " + String(psram_Readings[r].runNumber) + "   PN06    " + String(psram_Readings[r].lHour) + ":" + String(psram_Readings[r].lMin) + ":" + String(psram_Readings[r].lSec) + "  EST   " + String(psram_Readings[r].depthPa) + 
     //   "kPa  " + String(psram_Readings[r].depthM) + " meters");
  //}
 delay(2000);
 if (WiFi.status() == WL_CONNECTION_LOST || WiFi.status() != WL_CONNECTED) {
  WiFi.disconnect();
  WiFi.begin(ssid, password);
  flashLED(2);
 }

WiFiClient client = server.available();   // Listen for incoming clients

  if (client) {                             // If a new client connects,
    currentTime = millis();
    previousTime = currentTime;
    Serial.println("New Client.");          // Print a message out in the serial port
    String currentLine = "";                // Make a String to hold incoming data from the client
    while (client.connected() && currentTime - previousTime <= timeoutTime) {  // Loop while the client's connected
      currentTime = millis();
      if (client.available()) {             // If there's bytes to read from the client,
        char c = client.read();             // Read a byte, then
        Serial.write(c);                    // Print it out the serial monitor
        header += c;
        if (c == '\n') {                    // If the byte is a newline character
          // If the current line is blank, you got two newline characters in a row.
          // That's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // And a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();
            
            // Display the HTML web page
            client.println("<!DOCTYPE html><html>");
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<link rel=\"icon\" href=\"data:,\">");
            // CSS to style the table 
            client.println("<style>body { text-align: center; font-family: \"Trebuchet MS\", Arial;}");
            client.println("table { border-collapse: collapse; width:35%; margin-left:auto; margin-right:auto; }");
            client.println("th { padding: 12px; background-color: #0043af; color: white; }");
            client.println("tr { border: 1px solid #ddd; padding: 12px; }");
            client.println("tr:hover { background-color: #bcbcbc; }");
            client.println("td { border: none; padding: 12px; }");
            client.println(".sensor { color:white; font-weight: bold; background-color: #bcbcbc; padding: 1px; }");
            client.println("</style></head><body><h1>Da Floaty Boi</h1>");
            // Add button
            client.println("<button onclick=\"triggerFunction()\">DIVE!</button>");
            client.println("<script>");
            client.println("function triggerFunction() {");
            client.println("fetch('/trigger-function', { method: 'POST' })");
            client.println(".then(response => response.text())");
            client.println(".then(data => console.log(data))");
            client.println(".catch(error => console.error('Error:', error));");
            client.println("}");
            client.println("</script>");
            client.println("</body></html>");
            for (int r = 0; r < readingCnt; r++){
            client.println("<p>penis</p>");

            }
            
            // The HTTP response ends with another blank line
            client.println();
            // Break out of the while loop
            break;
          } else { // If you got a newline, then clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') {  // If you got anything else but a carriage return character,
          currentLine += c;      // Add it to the end of the currentLine
        }
      }
    }

    // Check if the request was a POST to /trigger-function
    if (header.indexOf("POST /trigger-function") >= 0) {
      Serial.println("Button was clicked");
      // Respond to the POST request
      dive();
      client.println("HTTP/1.1 200 OK");
      client.println("Content-type:text/plain");
      client.println("Connection: close");
      client.println();
      client.println("Function executed successfully!");
    }

    // Clear the header variable
    header = "";
    // Close the connection
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
  }



if (diving == true) {
        pressureValueMax = depthPascal + 3; //Sets pressure range
        pressureValueMin = depthPascal - 3; //Sets pressure range 
        if (depthPascal >= pressureValueMin || depthPascal <= pressureValueMax){
            atBottom = 1;
        }
        JustInCase = JustInCase + 1;
    }
    Serial1.print("0");
    delay(5500); //For one second
    Serial1.print("90");
    diving = false;
    
}


void getTime(void){
  setCpuFrequencyMhz(240);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(1000);              // wait for a second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  }
  while (Ping.ping("www.google.com") == false) {
    flashLED(3);
    delay(2000);
  }

  server.begin();
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
        struct tm timeinfo = rtc.getTimeStruct();
        if (getLocalTime(&timeinfo)){
        rtc.setTimeStruct(timeinfo); 
        Serial.println(rtc.getTime("%A, %B %d %Y %H:%M:%S"));
 } 
} 

void flashLED(int flashes) {
  for (int i = 0; i < flashes; ++i) {
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(100);              // wait for a second
  digitalWrite(LED_BUILTIN, LOW);
  delay(50);    // turn the LED off by making the voltage LOW
  }
}

void dive(void) {
   Serial1.println("180");
   Serial.println("I'ma diving bitch!");
    delay (4000); //For one second.
   Serial1.println("90");
    atBottom = false;
    JustInCase = 0;
    diving = true; 
    runNum++;   
}