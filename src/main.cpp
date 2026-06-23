#include <main.hpp>
#include <ESPmDNS.h>

/// pin 16 and 17 are for endstops
void setup() {
  setCpuFrequencyMhz(240);
  Serial.begin(115200);
  serialLog("Begin chooch");

  // Configure I2C for depth sensor and display
  Wire.setPins(3, 4);
  Wire.begin();
  
  // Display Initialization
  pinMode(TFT_I2C_POWER, OUTPUT);
  digitalWrite(TFT_I2C_POWER, HIGH);
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
  canvas.fillScreen(ST77XX_RED);
  canvas.setCursor(0, 25);
  canvas.setFont(&FreeSans9pt7b);
  canvas.print("SHE'S ALIVEEEEEEEEE");
  display.drawRGBBitmap(0, 0, canvas.getBuffer(), 240, 135);

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
    serialLog("\nPSRAM is correctly initialized");
  } else {
    serialLog("PSRAM not available");
  }

  //while (! Serial) delay(10);
  delay(100); // do not remove 
  
  // Configure Webserver
  setupWebServer();

  // Display startup message
  canvas.fillScreen(ST77XX_BLACK);
  canvas.setCursor(0, 25);
  canvas.setFont(&FreeSans9pt7b);
  canvas.print("SHE'S ALIVEEEEEEEEE");
  display.drawRGBBitmap(0, 0, canvas.getBuffer(), 240, 135);

  // Stepper Configuration
  stepper.connectToPins(step_pin, dir_pin);
  stepper.setSpeedInStepsPerSecond(4000);
  stepper.setAccelerationInStepsPerSecondPerSecond(10000);
  stepper.setDecelerationInStepsPerSecondPerSecond(10000);
  serialLog("Attempting to home...");
  home_status = stepper.moveToHomeInSteps(-1, 1000, step_pos_max*1.2, limit_switch_pin);
  if (home_status == true) {
    serialLog("Homing successful");
    // stepper.setCurrentPositionInSteps(step_pos_max - 251);
    stepper.setTargetPositionRelativeInSteps(-step_pos_max);
  } else {
    serialLog("Homing failed");
  }
  // When used with moveRelativeInStep will cause race condition
  stepper.startAsService(0);

  // Initialize pressure sensor
  // We can't continue with the rest of the program unless we can initialize the sensor
  // Configure pressure sensor
  sensor.setModel(MS5837::MS5837_02BA); 
  sensor.setFluidDensity(1000); // kg/m^3 (freshwater, 1029 for seawater)
  while (!sensor.init()) {
    serialLog("Init failed!");
    serialLog("Are SDA/SCL connected correctly?");
    serialLog("Blue Robotics Bar30: White=SDA, Green=SCL");
    serialLog("\n\n\n");
    delay(250);
    flashLED(4);

  }

  serialLog("Chooch has begun");
  //flashLED_async(8);

  // Configure Depth Controller
  // TODO: Fix this 
  Kp = -1.0;
  Ki = 0.0;
  Kd = 0.0;
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
  depthPascal = sensor.pressure();
  int sec = rtc.getSecond();
  filterInput(depthMeter);

  // Update web UI with telemetry
  drainLogQueue();
  broadcastTelemetry();
  Serial.println("Depth: " + String(depthMeter));

  // Record Sensor Readings
  if (recordData && sec != lastSecond && sec % 5 == 0 ){
    psram_Readings[readingCnt].runNumber = runNum;
    psram_Readings[readingCnt].depthPa = depthPascal/10.0f;   // TODO: why divide by 10?    
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

    // serialLog("Grabbed a data");
  }
  lastSecond = sec;

  // Wifi Reconnection Logic
  // TODO: Validate reconnection states
  if (WiFi.status() == WL_CONNECTION_LOST || WiFi.status() != WL_CONNECTED) {
    WiFi.disconnect();
    WiFi.begin(ssid, password);
    // flashLED(2);
    serialLog("Lost wifi");
  }

  // stepper.moveRelativeInSteps(-250);
  // delay(100);

  // serialLog("Reached diving statement");
  serialLog("Current Position: " + String(stepper.getCurrentPositionInSteps()));
  if (diving == true) {

    // serialLog("Entered diving if statement");  

    // Setpoint Traversal Logic
    // traverseSetpoints();

    // Control Implementation Logic
    // filterInput(depthMeter);
    BangBangBoi.run();
    // control_output *= -1.0; // Invert control output because of motor orientation
    limitCheck(control_output);
    // serialLog("Controller Weights: Kp: " + String(Kp) + ", Ki: " + String(Ki) + ", Kd:" + String(Kd));
    serialLog("Control output: " + String(control_output));
    serialLog("Plant state: " + String(depthMeter));
    serialLog("Current Setpoint: " + String(control_setpoint));
    serialLog("Current Target: " + String(stepper.getTargetPositionInSteps()));
    // Run 2 twice as fast to ensure we don't miss the window for setpoint arrival
  }
}

void traverseSetpoints() {
  if (BangBangBoi.atSetPoint(setpoint_margin) && isnan(arrivalTime)) {
      // Just arrived
      arrivalTime = millis();
      serialLog("Setpoint reached: " + String(control_plant) + " meters");
  } else if (!isnan(arrivalTime) && millis() - arrivalTime >= loiter_time_sec * 1e3 && packet_count>= 7 ) {
      // Loiter complete — check bounds before incrementing
      arrivalTime = NAN;
      packet_count = 0;
      if (setpoint_index + 1 < sizeof(setpoint) / sizeof(setpoint[0]) ) {
          setpoint_index++;
          control_setpoint = setpoint[setpoint_index];
          serialLog("Setting next waypoint: " + String(control_setpoint) + "...");
      } else {
          serialLog("Final setpoint reached!\nSurfacing...");
          surface();
      }
  } else if (!isnan(arrivalTime) && !BangBangBoi.atSetPoint(setpoint_margin)) {
      // Departed early
      arrivalTime = NAN;
      serialLog("Departed from setpoint, resetting timer");
  } else if (!isnan(arrivalTime)) {
      // Still loitering
      serialLog("Loitering at setpoint: " + String(control_plant) + " meters");
  }
}

void limitCheck(double absoluteTarget){
  // long currentPosition = -stepper.getCurrentPositionInSteps();
  // serialLog("Current position: " + String(currentPosition));
  serialLog("End Stop: " + String(digitalRead(limit_switch_pin)));
  if (digitalRead(limit_switch_pin) == LOW) {
    serialLog("Limit hit!");
  }  
  // absoluteTarget *= step_pos_max; // Convert from meters to steps
  // absoluteTarget = constrain(absoluteTarget, step_pos_min, step_pos_max);
  long position = step_pos_max - absoluteTarget; // Convert from meters to steps
  stepper.setTargetPositionInSteps(absoluteTarget);
}

void filterInput(double depthValue){
  double delta = depthValue - previousDepth;
  if (abs(delta) > allowed_depth_delta) {
    // If the change is greater than the allowed delta, ignore it
    control_plant = previousDepth;
  } else {
    // Otherwise, use the new value
    depthValue += depthOffset; // Apply calibration offset
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
    serialLog("No wifi");
    delay(400);
  }

  // Wait for internet connectivity before configuring NTP
  while (Ping.ping("www.google.com") == false) {
    flashLED(3);
    delay(2000);
    serialLog("No internet");
  }
  
  // Configure local time via NTP
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
        struct tm timeinfo = rtc.getTimeStruct();
        if (getLocalTime(&timeinfo)){
        rtc.setTimeStruct(timeinfo); 
        serialLog(rtc.getTime("%A, %B %d %Y %H:%M:%S"));
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
    serialLog("I'ma divin', bitch!");
    pDiveTime = millis(); 
    // TODO: Check what we want this to be
    runNum++;  

    // Display startup message
    canvas.fillScreen(ST77XX_GREEN);
    canvas.setCursor(0, 25);
    canvas.setFont(&FreeSans9pt7b);
    canvas.print("We diving bitch!");
    display.drawRGBBitmap(0, 0, canvas.getBuffer(), 240, 135);
  } 
}

void surface(void){
  recordData = false;
  // diving = false;
  control_setpoint = -2.0;

  uint8_t status = 0;
  // while(true){
  //   // Display surfaceing message
  //   // if(status == 0){
  //   //   canvas.fillScreen(ST77XX_BLACK);
  //   //   status = 1;
  //   // } else {
  //   //   canvas.fillScreen(ST77XX_RED);
  //   //   status = 0;
  //   // }
  //   // canvas.setCursor(0, 25);
  //   // canvas.setFont(&FreeSans9pt7b);
  //   // canvas.print("MISISON COMPLETE!");
  //   // display.drawRGBBitmap(0, 0, canvas.getBuffer(), 240, 135);
  //   delay(500);
  // }
}