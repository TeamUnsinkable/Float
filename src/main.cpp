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


// Analog servos run at ~50 Hz updates
#define maximumReadings 2000
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
bool atBottom = false;
int JustInCase = 0;
double pressureValueMax;
double pressureValueMin;
double deltaP;
static int lastSecond = -1;
int sec;
bool limit_hit = false;
int step_pin = 16;
int UART_RX = 18;
int UART_TX = 17;
int dir_pin = 15;
static const long SERIAL_BAUD = 9600;
unsigned long pDiveTime = 0;
unsigned long deltaT_prev;
int step_pos;
bool bouncing;

void getTime();
void flashLED(int times);
//void ledTask(void*);
void dive();
void surface();
void home();
void stop();
//void flashLED_async(uint32_t flashes);
void defineHTML(void);
void step(int steps, int step_rate);
void bounce(void);



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


//TaskHandle_t ledTaskHandle = nullptr;

void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

AsyncWebServer server1(80);
WiFiServer server2(8080);

/// pin 16 and 17 are for endstops
void setup() {

getTime();
   // Send web page to client
  server1.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html);
  });

  // Receive an HTTP GET request
  server1.on("/on", HTTP_GET, [] (AsyncWebServerRequest *request) {
    dive();
    request->send(200, "text/plain", "ok");
  });

  // Receive an HTTP GET request
  server1.on("/off", HTTP_GET, [] (AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "ok");
  });
  
  server1.onNotFound(notFound);
//  if (!WiFi.config(local_IP, gateway, subnet)) {\]
//  Serial.println("STA Failed to configure");
//}

pinMode(6, INPUT_PULLDOWN);
attachInterrupt(digitalPinToInterrupt(6), stop, FALLING);
pinMode(9, OUTPUT);
digitalWrite(9, HIGH);
pinMode(LED_BUILTIN, OUTPUT);

psram_Readings = (sReadings *)ps_malloc(maximumReadings * sizeof(sReadings)); 
        if(psramInit()){
        Serial.println("\nPSRAM is correctly initialized");
        }else{
        Serial.println("PSRAM not available");
        }

  // initialize USB serial converter so we have a port created
   Serial.begin(115200);
  while (! Serial) delay(10);
  delay(100);
  stepper_driver.setup(serial_stream,
                       SERIAL_BAUD,
                       TMC2209::SERIAL_ADDRESS_0,
                       UART_RX,
                       UART_TX);
  delay(100);  

  Serial.println("Begin chooch");
  Wire.begin();

  //xTaskCreate(ledTask,"LED",1024,nullptr,1,&ledTaskHandle);
  sensor.setModel(MS5837::MS5837_02BA);

  // Initialize pressure sensor
  // We can't continue with the rest of the program unless we can initialize the sensor
 // while (!sensor.init()) {
//    Serial.println("Init failed!");
 //   Serial.println("Are SDA/SCL connected correctly?");
  //  Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
  //  Serial.println("\n\n\n");
  //  delay(100);
  //  flashLED(4);
//}
//sensor.setFluidDensity(1000); // kg/m^3 (freshwater, 1029 for seawater)


Serial.println("Chooch has begun");
//flashLED_async(8);
//home(); 

pinMode(dir_pin, OUTPUT);
pinMode(step_pin, OUTPUT);

step(1000000, 50);
}


 

void loop() {



  //Serial.println("Looped");
  //flashLED_async(1);

    
  
  sensor.read();
  depthMeter = sensor.depth();
  depthPascal = sensor.pressure();
  sec = rtc.getSecond();
  if (sec != lastSecond && sec % 5 == 0)
  {
  psram_Readings[readingCnt].runNumber = runNum;
  psram_Readings[readingCnt].depthPa = depthPascal/10.0f;        
  psram_Readings[readingCnt].depthM = depthMeter;      
  psram_Readings[readingCnt].lHour = rtc.getHour();       // current hour
  psram_Readings[readingCnt].lMin = rtc.getMinute();     // current minute
  psram_Readings[readingCnt].lSec = rtc.getSecond();     // current second
  readingCnt++;
  Serial.println("Grabbed a data");

  }
  lastSecond = sec;
/*
  for (int r = 0; r < readingCnt; r++){
       // Now output readings in CSV format to the serial port
       Serial.println("Profile#: " + String(psram_Readings[r].runNumber) + "   EX01    " + String(psram_Readings[r].lHour) + ":" + String(psram_Readings[r].lMin) + ":" + String(psram_Readings[r].lSec) + "  EST   " + String(psram_Readings[r].depthPa) + 
    "kPa  " + String(psram_Readings[r].depthM) + " meters");
  }
*/
 if (WiFi.status() == WL_CONNECTION_LOST || WiFi.status() != WL_CONNECTED) {
  WiFi.disconnect();
  WiFi.begin(ssid, password);
  flashLED(2);
  Serial.println("Lost wifi");
 }

WiFiClient client = server2.available();   // Listen for incoming clients

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
            for (int r = 0; r < readingCnt; r++){
            client.println("<p> Profile#:" + String(psram_Readings[r].runNumber) +  "  EX01  "  + String(psram_Readings[r].lHour) + ":" + String(psram_Readings[r].lMin) + ":" + String(psram_Readings[r].lSec) + "  EST   " + String(psram_Readings[r].depthPa) + 
        "kPa  " + String(psram_Readings[r].depthM) + " meters</p>");
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



    // Clear the header variable
    header = "";
    // Close the connection
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
  }
  
if (diving == true) {
        deltaP = psram_Readings[readingCnt].depthPa - psram_Readings[readingCnt-1].depthPa;
        deltaT_prev = pDiveTime - millis();
        if ((deltaT_prev >= 60000) || (deltaP < 1000 && deltaT_prev > 11000)){
            diving = false;
            surface();
        }
    }

}


void getTime(void){
  setCpuFrequencyMhz(240);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    flashLED(1);
    Serial.println("No wifi");
  }
  while (Ping.ping("www.google.com") == false) {
    flashLED(3);
    delay(2000);
    Serial.print("No internet");
  }

  server1.begin();
  server2.begin();
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

void step(int steps, int step_rate)
{
  Serial.println("Ented for loop");
    for (int i = 0; i < steps; i++) {
      if (limit_hit == true && bouncing == false) 
      {bounce(); break;}
      digitalWrite(step_pin, HIGH);
      delayMicroseconds(step_rate);
      digitalWrite(step_pin, LOW);
      delayMicroseconds(step_rate);
      step_pos++;
    }
  Serial.println("Exited For Loop");
}

void dive(void) {
  Serial.println("I'ma divin', bitch!");
  pDiveTime = millis(); 
  limit_hit = false;
  stepper_driver.disableInverseMotorDirection();  // Flip this line and the other inverse line of homing is in wrong direction.
  stepper_driver.moveAtVelocity(360000);
  delay(3000);
  stepper_driver.moveAtVelocity(0);
  atBottom = false;
  JustInCase = 0;
  diving = true; 
  runNum++;   
}

void surface(void)
{
    Serial.println("I'ma surfacin', bitch!");
  limit_hit = false;
  stepper_driver.enableInverseMotorDirection();  // Flip this line and the other inverse line of homing is in wrong direction.
  stepper_driver.moveAtVelocity(360000);
  delay(2900);
  stepper_driver.moveAtVelocity(0);
  diving = false;
}

void home(void)
{Serial.println("HOMING"); //print action
  limit_hit = false;
  step(-20000, 200);
}

void bounce(void){

  bouncing = true;
  limit_hit == false;
  step(1000, 200);
  while (limit_hit == false)
  {
  step(-1, 400);
  }
  limit_hit = false;
  step(15000, 200);
  bouncing = false;
  
}

void stop()
{
 limit_hit = true;
}
/*
void ledTask(void*){
  pinMode(LED_BUILTIN, OUTPUT);

  for (;;) {
    // Wait for a flash request (number of flashes passed as value)
    uint32_t flashes = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    for (uint32_t i = 0; i < flashes; ++i) {
      digitalWrite(LED_BUILTIN, HIGH);
      vTaskDelay(pdMS_TO_TICKS(10));

      digitalWrite(LED_BUILTIN, LOW);
      vTaskDelay(pdMS_TO_TICKS(5));
    }
  }
}
*/
//void flashLED_async(uint32_t flashes) {
 // if (!ledTaskHandle) return;
 // xTaskNotifyGive(ledTaskHandle);               // wake task
 // xTaskNotify(ledTaskHandle, flashes, eSetValueWithOverwrite);
//}


const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
  <head>
    <title>ESP Pushbutton Web Server</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
      body { font-family: Arial; text-align: center; margin:0px auto; padding-top: 30px;}
      .button {
        padding: 10px 20px;
        font-size: 24px;
        text-align: center;
        outline: none;
        color: #fff;
        background-color: #2f4468;
        border: none;
        border-radius: 5px;
        box-shadow: 0 6px #999;
        cursor: pointer;
        -webkit-touch-callout: none;
        -webkit-user-select: none;
        -khtml-user-select: none;
        -moz-user-select: none;
        -ms-user-select: none;
        user-select: none;
        -webkit-tap-highlight-color: rgba(0,0,0,0);
      }  
      .button:hover {background-color: #1f2e45}
      .button:active {
        background-color: #1f2e45;
        box-shadow: 0 4px #666;
        transform: translateY(2px);
      }
    </style>
  </head>
  <body>
    <h1>ESP32-Driven Float</h1>
    <button class="button" onmousedown="toggleCheckbox('on');" ontouchstart="toggleCheckbox('on');" onmouseup="toggleCheckbox('off');" ontouchend="toggleCheckbox('off');">DIVE!</button>
   <script>
   function toggleCheckbox(x) {
     var xhr = new XMLHttpRequest();
     xhr.open("GET", "/" + x, true);
     xhr.send();
   }
  </script>
  </body>
</html>)rawliteral";


