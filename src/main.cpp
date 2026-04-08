#include <main.hpp>


/// pin 16 and 17 are for endstops
void setup() {
Serial.println("Begin chooch");
getTime();

 pinMode(TFT_I2C_POWER, OUTPUT);
 digitalWrite(TFT_I2C_POWER, HIGH);

  
 display.init(135, 240);           // Init ST7789 240x135
 display.setRotation(3);
 canvas.setFont(&FreeSans9pt7b);
 canvas.setTextColor(ST77XX_WHITE);


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
  //while (! Serial) delay(10);
  delay(100);
  stepper_driver.setup(serial_stream);
  delay(100);  

  
  Wire.begin();

  //xTaskCreate(ledTask,"LED",1024,nullptr,1,&ledTaskHandle);
  sensor.setModel(MS5837::MS5837_02BA);


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

canvas.fillScreen(ST77XX_BLACK);
canvas.setCursor(0, 25);
canvas.setFont(&FreeSans9pt7b);
canvas.print("SHE'S ALIVEEEEEEEEE");
display.drawRGBBitmap(0, 0, canvas.getBuffer(), 240, 135);

Serial.println("Chooch has begun");
//flashLED_async(8);
stepper_driver.setRunCurrent(RUN_CURRENT_PERCENT);
stepper_driver.enableCoolStep();
stepper_driver.enable();
stepper_driver.disable();
pinMode(dir_pin, OUTPUT);
pinMode(step_pin, OUTPUT);
digitalWrite(dir_pin, LOW);

home();
//stepper_driver.enableInverseMotorDirection();
//stepper_driver.moveAtVelocity(2000);
//delay(5000);
//stepper_driver.moveAtVelocity(0);


}


 

void loop() {
  //Serial.println("Looped");
  //flashLED_async(1);
  

  //stepper_driver.moveAtVelocity(2000);
  //delay(5000);
  //stepper_driver.moveAtVelocity(0);


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


handleWebserver();

  
if (diving == true) {
        deltaP = psram_Readings[readingCnt].depthPa - psram_Readings[readingCnt-1].depthPa;
        deltaT_prev = millis() - pDiveTime;
       // if ((deltaT_prev >= 90000) || (deltaP < 500 && deltaT_prev > 11000)){
       //     diving = false;
       //     surface();
        //}
         if (deltaT_prev >= 30000) {
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

void step(int steps, int step_delay)
{
  Serial.println("step function called");
  if (steps < 0) {
    for (int i = 0; i < abs(steps); i++) {
        digitalWrite(dir_pin, LOW);
        if (limit_hit == true && bouncing == false) 
        {bounce(); break;}
        digitalWrite(step_pin, HIGH);
        delayMicroseconds(step_delay);
        digitalWrite(step_pin, LOW);
        delayMicroseconds(step_delay);
        step_pos--;
      }}
  else if (steps > 0) {
    for (int i = 0; i < steps; i++) {
       digitalWrite(dir_pin, HIGH);
       if (limit_hit == true && bouncing == false) 
       {bounce(); break;}
       digitalWrite(step_pin, HIGH);
       delayMicroseconds(step_delay);
       digitalWrite(step_pin, LOW);
       delayMicroseconds(step_delay);
       step_pos++;
      }
    }
    else {
      Serial.println("Fuck, called steps count no chooch");
    }
}

void dive(void) {
  if (diving == false)
  {
  diving = true;
  Serial.println("I'ma divin', bitch!");
  pDiveTime = millis(); 
  limit_hit = false;
  step(-22000,100);  // Flip this line and the other inverse line of homing is in wrong direction.
  JustInCase = millis();
  runNum++;  
  } 
}

void surface(void)
{
    Serial.println("I'ma surfacin', bitch!");
  limit_hit = false;
  step(51000,100);
  diving = false;
  
}

void home(void){
  Serial.println("HOMING"); //print action
  limit_hit = false;
  step(-800000, 200);
}

void bounce(void){
  Serial.println("Bouncing");
  bouncing = true;
  limit_hit = false;
  step(1000, 200);
  while (limit_hit == false)
  {
  step(-1, 400);
  }
  limit_hit = false;
  step(25000, 100); // 1000 steps is ~2ml
  bouncing = false;
}

void stop()
{
 limit_hit = true;
 Serial.println("Stop received");
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


void handleWebserver() {

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
}

