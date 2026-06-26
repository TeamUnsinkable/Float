#include <main.hpp>
#include <ESPmDNS.h>

/// pin 16 and 17 are for endstops
void setup() {
  setCpuFrequencyMhz(240);
  Serial.begin(115200);
  Serial.println("Begin chooch");

  // Configure I2C for depth sensor and display
  Wire.setPins(3, 4);
  Wire.begin();
   
  // Display Initialization
  pinMode(TFT_PWR_I2C, OUTPUT);
  digitalWrite(TFT_PWR_I2C, HIGH);
  delay(100); // Adafruit says it needs a brief delay for power to stabilize
  pinMode(TFT_BACKLITE, OUTPUT);
  digitalWrite(TFT_BACKLITE, HIGH);
  delay(100); // Adafruit says it needs a brief delay for power to stabilize
  // Init ST7789 240x135
  display.init(135, 240);    
  // Rotate to landscape       
  display.setRotation(3);
  canvas.setFont(&FreeSans9pt7b);
  canvas.setTextColor(ST77XX_WHITE);

  // Display startup message
  // canvas.fillScreen(ST77XX_RED);
  // canvas.setCursor(0, 25);
  // canvas.setFont(&FreeSans9pt7b);
  // canvas.print("SHE'S ALIVEEEEEEEEE");
  // display.drawRGBBitmap(0, 0, canvas.getBuffer(), 240, 135);
  writeDisplay("Awaiting WiFi Network", ST77XX_WHITE, ST77XX_BLACK);

  // Blocks until WiFi connected and gets NTP time
  getTime();

  // Limit Switch Configuration
  pinMode(limit_switch_pin, INPUT_PULLUP);

  // Only for old ESP32 - remove when upgraded
  #if defined(ESP32_S2)
  pinMode(9, OUTPUT);
  digitalWrite(9, LOW);
  pinMode(LED_BUILTIN, OUTPUT);
  #endif

  // PSRAM Initialization
  psram_Readings = (sReadings *)ps_malloc(maximumReadings * sizeof(sReadings)); 
  if(psramInit()){
    Serial.println("\nPSRAM is correctly initialized");
  } else {
    Serial.println("PSRAM not available");
  }

  //while (! Serial) delay(10);
  delay(100); // do not remove 
  
  // Configure Webserver
  setupWebServer();

  writeDisplay("Homing...", ST77XX_WHITE, ST77XX_BLACK);

  // Stepper Configuration
  stepper.connectToPins(step_pin, dir_pin);
  stepper.setSpeedInStepsPerSecond(4000);
  stepper.setAccelerationInStepsPerSecondPerSecond(10000);
  stepper.setDecelerationInStepsPerSecondPerSecond(10000);
  Serial.println("Attempting to home...");
  home_status = stepper.moveToHomeInSteps(-1, 1000, step_pos_max*1.2, limit_switch_pin);
  if (home_status == true) {
    Serial.println("Homing successful");
    writeDisplay("Homing...\n\nSuccessful", ST77XX_GREEN, ST77XX_BLACK);
    // stepper.setCurrentPositionInSteps(step_pos_max - 251);
    stepper.setTargetPositionRelativeInSteps(-step_pos_max);
    delay(500);
  } else {
    Serial.println("Homing failed");
    writeDisplay("Homing...\n\nFailed", ST77XX_WHITE, ST77XX_RED);
    delay(5000);
  }
  // When used with moveRelativeInStep will cause race condition
  stepper.startAsService(0);

  // Initialize pressure sensor
  // We can't continue with the rest of the program unless we can initialize the sensor
  // Configure pressure sensor
  sensor.setModel(MS5837::MS5837_02BA); 
  sensor.setFluidDensity(1000); // kg/m^3 (freshwater, 1029 for seawater)
  while (!sensor.init()) {
    writeDisplay("Depth Sensor Init Failed", ST77XX_WHITE, ST77XX_RED);
    Serial.println("Init failed!");
    Serial.println("Are SDA/SCL connected correctly?");
    Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
    Serial.println("\n\n\n");
    delay(250);
    flashLED(4);
  }

  Serial.println("Chooch has begun");
  //flashLED_async(8);
  writeDisplay("Awaiting for dive command", ST77XX_WHITE, ST77XX_BLACK);
  // Configure Depth Controller
  outputMin = step_pos_min;
  outputMax = step_pos_max;
  BangBangBoi.setTimeStep(control_loop_rate_ms);
  BangBangBoi.setOutputRange(outputMin, outputMax);
  control_setpoint = 0.45;
  // BangBangBoi.setBangBang(0.01);
} // end of setup()
 
void loop() {
  flashLED(1);
  
  // Get sensor readings
  sensor.read();
  depthMeter = sensor.depth();
  depthPascal = sensor.pressure()/10.0f; // Convert mBar to kPa
  int sec = rtc.getSecond();
  filterInput(depthMeter);

  // Update web UI with telemetry
  drainLogQueue();
  broadcastTelemetry();
  Serial.println("Depth: " + String(depthMeter));

  // Record Sensor Readings
  if (recordData && sec != lastSecond && sec % 5 == 0 ){
    psram_Readings[readingCnt].runNumber = runNum;
    psram_Readings[readingCnt].depthPa = depthPascal;         // in mBar    
    psram_Readings[readingCnt].depthM = depthMeter;           // current depth
    psram_Readings[readingCnt].lHour = rtc.getHour();         // current hour
    psram_Readings[readingCnt].lMin = rtc.getMinute();        // current minute
    psram_Readings[readingCnt].lSec = rtc.getSecond();        // current second
    psram_Readings[readingCnt].packet = readingCnt;           // example packet identifier
    readingCnt++;

    // Count to ensure 7 packets are recorded
    if (arrivalTime != NAN){
      packet_count++;
    }

    // Serial.println("Grabbed a data");
  }
  lastSecond = sec;

  // Wifi Reconnection Logic
  // TODO: Validate reconnection states
  // if (( WiFi.status() == WL_CONNECTION_LOST || WiFi.status() != WL_CONNECTED ) && 
  //     ((millis() - lastWifiReconnect > 2000) || true)) {
  //   lastWifiReconnect = millis();
  //   WiFi.disconnect();
  //   WiFi.begin(ssid, password);
  //   // flashLED(2);
  //   Serial.println("Lost wifi");
  // }


  Serial.println("Current Position: " + String(stepper.getCurrentPositionInSteps()));
  // Validate at least 1 packet has been recorded
  if (diving == true && millis() - pDiveTime > 10000) {
    char* msg = (char*)malloc(128 * sizeof(char));
    sprintf(msg, "diving...\nCurrent Depth: %.2f m\n Current Setpoint: %.2f m\nCurrent Target: %ld steps", depthMeter, control_setpoint, stepper.getTargetPositionInSteps());
    writeDisplay(msg, ST77XX_WHITE, status_color);
    free(msg);

    // Setpoint Traversal Logic
    traverseSetpoints();

    // Serial.println("Controller Weights: Kp: " + String(Kp) + ", Ki: " + String(Ki) + ", Kd:" + String(Kd));
    Serial.println("Control output: " + String(control_output));
    Serial.println("Plant state: " + String(depthMeter));
    Serial.println("Current Setpoint: " + String(control_setpoint));
    Serial.println("Current Target: " + String(stepper.getTargetPositionInSteps()));
    Serial.println("\n\n");

    BangBangBoi.run();
    limitCheck(control_output);
  }


  // Surfacing Logic
  if (control_setpoint == -10.0) {
    if (rtc.getSecond() % 2 == 0) {
      color = ST77XX_RED;
      writeDisplay("COMPLETED MISSION!", ST77XX_WHITE, color);
    } else if (color == ST77XX_RED) {
      color = ST77XX_BLACK;
      writeDisplay("COMPLETED MISSION!", ST77XX_WHITE, color);
    } 
    delay(250);
  }
}

void traverseSetpoints() {
  if (BangBangBoi.atSetPoint(setpoint_margin) && isnan(arrivalTime)) {
      // Just arrived
      arrivalTime = millis();
      Serial.println("Setpoint reached: " + String(control_plant) + " meters");
      status_color = ST77XX_GREEN;
  } else if (!isnan(arrivalTime) && millis() - arrivalTime >= loiter_time_sec * 1e3 && packet_count >= packet_count_req) {
      // Loiter complete — check bounds before incrementing
      status_color = ST77XX_BLACK;
      arrivalTime = NAN;
      packet_count = 0;
      if (setpoint_index + 1 < sizeof(setpoint) / sizeof(setpoint[0]) ) {
          setpoint_index++;
          if (setpoint[setpoint_index] > control_setpoint)
          {
            runNum++;
          }
          control_setpoint = setpoint[setpoint_index];
          Serial.println("Setting next waypoint: " + String(control_setpoint) + "...");
      } else {
          Serial.println("Final setpoint reached!\nSurfacing...");
          surface();
          control_setpoint = 0.4;
      }
  } else if (!isnan(arrivalTime) && !BangBangBoi.atSetPoint(setpoint_margin)) {
      // Departed early
      status_color = ST77XX_ORANGE;
      arrivalTime = NAN;
      packet_count = 0;
      Serial.println("Departed from setpoint, resetting timer");
  } else if (!isnan(arrivalTime)) {
      // Still loitering
      status_color = ST77XX_YELLOW;
      Serial.println("Loitering at setpoint: " + String(control_plant) + " meters");
  }
}

void limitCheck(double absoluteTarget){
  Serial.println("End Stop: " + String(digitalRead(limit_switch_pin)));
  if (digitalRead(limit_switch_pin) == HIGH) {
    Serial.println("Limit hit!");
  }  

  Serial.println("Absolute Target: " + String(absoluteTarget));

  if (diving == true){
    long position = -step_pos_max + absoluteTarget; 
    Serial.println("Position: " + String(position));
    stepper.setTargetPositionInSteps(position);
  }
}

void filterInput(double depthValue){
  double delta = depthValue - previousDepth;
  if (abs(delta) > allowed_depth_delta) {
    // If the change is greater than the allowed delta, ignore it
    control_plant = previousDepth;
  } else {
    // Otherwise, use the new value
    // depthValue += depthOffset; // Apply calibration offset
    control_plant = depthValue;
    previousDepth = depthValue;
  }
}

void getTime(void){
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(hostname);
  MDNS.begin(hostname);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    flashLED(2);
    Serial.println("No wifi");
    delay(400);
  }

  // Wait for internet connectivity before configuring NTP
  while (Ping.ping("www.google.com") == false) {
    flashLED(3);
    delay(2000);
    Serial.println("No internet");
  }

  writeDisplay("Connected to WiFi!", ST77XX_WHITE, ST77XX_BLACK);
  
  // Configure local time via NTP
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
        struct tm timeinfo = rtc.getTimeStruct();
        if (getLocalTime(&timeinfo)){
        rtc.setTimeStruct(timeinfo); 
        Serial.println(rtc.getTime("%A, %B %d %Y %H:%M:%S"));
 } 
} 

void flashLED(int flashes) {
  for (int i = 0; i < flashes; ++i) {
    digitalWrite(LED_BUILTIN, HIGH);    // turn the LED on (HIGH is the voltage level)
    delayMicroseconds(100);             // 1ms
    digitalWrite(LED_BUILTIN, LOW);     // turn the LED off by making the voltage LOW
    delayMicroseconds(50);              // 0.5 ms
  }
}

void dive(void) {
  if (diving == false) {
    diving = true;
    recordData = true;
    Serial.println("I'ma divin', bitch!");
    pDiveTime = millis(); 
    // TODO: Check what we want this to be
    runNum++;  

    // Display startup message
    control_setpoint = setpoint[setpoint_index];
  } 
}

void surface(void){
  recordData = false;
  diving = false;
  control_setpoint = -10.0;
}

void writeDisplay(const char *message, uint16_t color = ST77XX_WHITE,
  uint16_t background = ST77XX_BLACK) {

    canvas.setFont(&FreeSans9pt7b);
    canvas.setTextColor(color);
    canvas.fillScreen(background);

    // Split message into lines on \n
    String full = String(message);
    int lineCount = 1;
    for (int i = 0; i < (int)full.length(); i++) {
        if (full[i] == '\n') lineCount++;
    }

    // Measure line height using a reference character
    int16_t x1, y1;
    uint16_t lineW, lineH;
    canvas.getTextBounds("A", 0, 0, &x1, &y1, &lineW, &lineH);
    int lineSpacing = lineH + 4;

    // Total block height, start Y centered
    int blockH = lineCount * lineSpacing;
    int startY = (135 / 2) - (blockH / 2) + lineH;

    // Draw each line centered horizontally
    int lineIndex = 0;
    int start = 0;
    for (int i = 0; i <= (int)full.length(); i++) {
        if (full[i] == '\n' || full[i] == '\0') {
            String line = full.substring(start, i);
            int16_t lx, ly;
            uint16_t lw, lh;
            canvas.getTextBounds(line.c_str(), 0, 0, &lx, &ly, &lw, &lh);
            int cursorX = (240 / 2) - (lw / 2) - lx;
            int cursorY = startY + lineIndex * lineSpacing;
            canvas.setCursor(cursorX, cursorY);
            canvas.print(line);
            lineIndex++;
            start = i + 1;
        }
    }

    display.drawRGBBitmap(0, 0, canvas.getBuffer(), 240, 135);
}