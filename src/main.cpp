#include <main.hpp>
#include <ESPmDNS.h>

/// pin 16 and 17 are for endstops
void setup() {
  Serial.println("Begin chooch");
  getTime();

  // Display Initialization
  pinMode(TFT_I2C_POWER, OUTPUT);
  digitalWrite(TFT_I2C_POWER, HIGH);
  delay(10); // Adafruit says it needs a brief delay for power to stabilize
  // display repair attempt
  pinMode(TFT_BACKLITE, OUTPUT);
  digitalWrite(TFT_BACKLITE, HIGH);
  display.init(135, 240);           // Init ST7789 240x135
  display.setRotation(3);
  canvas.setFont(&FreeSans9pt7b);
  canvas.setTextColor(ST77XX_WHITE);

  // Limit Switch Configuration
  pinMode(6, INPUT_PULLUP);
  pinMode(9, OUTPUT);
  digitalWrite(9, LOW);
  pinMode(LED_BUILTIN, OUTPUT);

  // PSRAM Initialization
  psram_Readings = (sReadings *)ps_malloc(maximumReadings * sizeof(sReadings)); 
  if(psramInit()){
    Serial.println("\nPSRAM is correctly initialized");
  } else {
    Serial.println("PSRAM not available");
  }

  Serial.begin(115200);
  //while (! Serial) delay(10);
  delay(100); // do not remove 
  Wire.begin();
  
  // Configure Webserver
  setupWebServer();

  // Stepper Configuration
  stepper.connectToPins(step_pin, dir_pin);
  stepper.setSpeedInStepsPerSecond(4000);
  stepper.setAccelerationInStepsPerSecondPerSecond(10000);
  stepper.setDecelerationInStepsPerSecondPerSecond(10000);
  Serial.println("Attempting to home...");
  home_status = stepper.moveToHomeInSteps(-1, 1000, step_pos_max*1.2, endstop_pin);
  if (home_status == true) {
    Serial.println("Homing successful");
    // stepper.setCurrentPositionInSteps(step_pos_max - 251);
    stepper.setTargetPositionRelativeInSteps(-step_pos_max );
  } else {
    Serial.println("Homing failed");
  }
  // When used with moveRelativeInStep will cause race condition
  stepper.startAsService(0);

  // Initialize pressure sensor
  // We can't continue with the rest of the program unless we can initialize the sensor
  // Configure pressure sensor
  sensor.setModel(MS5837::MS5837_02BA); 
  sensor.setFluidDensity(1000); // kg/m^3 (freshwater, 1029 for seawater)
  while (!sensor.init()) {
    Serial.println("Init failed!");
    Serial.println("Are SDA/SCL connected correctly?");
    Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
    Serial.println("\n\n\n");
    delay(100);
    flashLED(4);
  }

  // Display startup message
  canvas.fillScreen(ST77XX_BLACK);
  canvas.setCursor(0, 25);
  canvas.setFont(&FreeSans9pt7b);
  canvas.print("SHE'S ALIVEEEEEEEEE");
  display.drawRGBBitmap(0, 0, canvas.getBuffer(), 240, 135);


  Serial.println("Chooch has begun");
  //flashLED_async(8);

  // Configure Depth Controller
  // TODO: Fix this 
  Kp = 1.0;
  Ki = 0.0;
  Kd = 0.0;
  outputMin = -250.0;
  outputMax = 250.0;
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
  sec = rtc.getSecond();

  // Record Sensor Readings
  if (sec != lastSecond && sec % 5 == 0){
    psram_Readings[readingCnt].runNumber = runNum;
    psram_Readings[readingCnt].depthPa = depthPascal/10.0f;        
    psram_Readings[readingCnt].depthM = depthMeter;      
    psram_Readings[readingCnt].lHour = rtc.getHour();       // current hour
    psram_Readings[readingCnt].lMin = rtc.getMinute();     // current minute
    psram_Readings[readingCnt].lSec = rtc.getSecond();     // current second
    psram_Readings[readingCnt].packet = readingCnt; // example packet identifier
    readingCnt++;
    Serial.println("Grabbed a data");
  }
  lastSecond = sec;

  // Wifi Reconnection Logic
  if (WiFi.status() == WL_CONNECTION_LOST || WiFi.status() != WL_CONNECTED) {
    WiFi.disconnect();
    WiFi.begin(ssid, password);
    flashLED(2);
    Serial.println("Lost wifi");
  }

  // handleWebserver();
  // stepper.moveRelativeInSteps(-250);
  // delay(100);

  Serial.println("Reached diving statement");
  if (diving == true) {

    Serial.println("Entered diving if statement");  
        
    // Benchtop validation
    // int loiter_min = 4;
    // if (millis() - pDiveTime >= loiter_min * 60 * 1000){
    //   control_setpoint = 0.0;
    // } 

    // Setpoint Traversal Logic
    // traverseSetpoints();

    // Control Implementation Logic
    filterInput(depthMeter);
    BangBangBoi.run();
    // control_output *= -1.0; // Invert control output because of motor orientation
    limitCheck(control_output);
    Serial.println("Controller Weights: Kp: " + String(Kp) + ", Ki: " + String(Ki) + ", Kd:" + String(Kd));
    Serial.println("Control output: " + String(control_output));
    Serial.println("Plant state: " + String(depthMeter));
    Serial.println("Current Setpoint: " + String(control_setpoint));
    Serial.println("Arrived at Setpoint: " + String(BangBangBoi.atSetPoint(setpoint_margin)));

    // Run 2 twice as fast to ensure we don't miss the window for setpoint arrival
  }
}

void traverseSetpoints() {
  if (BangBangBoi.atSetPoint(setpoint_margin) && isnan(arrivalTime)) {
      // Just arrived
      arrivalTime = millis();
      Serial.println("Setpoint reached: " + String(control_plant) + " meters");
  } else if (!isnan(arrivalTime) && millis() - arrivalTime >= loiter_time_sec * 1e3) {
      // Loiter complete — check bounds before incrementing
      arrivalTime = NAN;
      if (setpoint_index + 1 < sizeof(setpoint) / sizeof(setpoint[0])) {
          setpoint_index++;
          control_setpoint = setpoint[setpoint_index];
          Serial.println("Setting next waypoint: " + String(control_setpoint) + "...");
      } else {
          Serial.println("Final setpoint reached!\nSurfacing...");
          BangBangBoi.stop();
          stepper.moveToPositionInSteps(step_pos_max);
      }
  } else if (!isnan(arrivalTime) && !BangBangBoi.atSetPoint(setpoint_margin)) {
      // Departed early
      arrivalTime = NAN;
      Serial.println("Departed from setpoint, resetting timer");
  } else if (!isnan(arrivalTime)) {
      // Still loitering
      Serial.println("Loitering at setpoint: " + String(control_plant) + " meters");
  }
}

void limitCheck(double relativeMove){
  long currentPosition = -stepper.getCurrentPositionInSteps();
  Serial.println("Current position: " + String(currentPosition));
  Serial.println("End Stop: " + String(digitalRead(endstop_pin)));
  if (currentPosition >= step_pos_max || digitalRead(endstop_pin) == HIGH) {
    Serial.println("Limit hit!");
  }  
  
  if (currentPosition + relativeMove > step_pos_max) {
    stepper.setTargetPositionInSteps(step_pos_max);
    // stepper.setTargetPositionRelativeInSteps(step_pos_max - currentPosition - currentPosition);
    Serial.println("Running to upper limit!");
    return;
  } else if (currentPosition + relativeMove < step_pos_min) {
    stepper.setTargetPositionInSteps(step_pos_min);
    // stepper.setTargetPositionRelativeInSteps(step_pos_min - currentPosition - currentPosition);
    Serial.println("Running to lower limit!");
    return;
  } else {
    stepper.setTargetPositionRelativeInSteps(relativeMove);
  }
}

void filterInput(double depthValue){
  double delta = depthValue - previousDepth;
  if (abs(delta) > 1) {
    // If the change is greater than 0.1 meters, ignore it
    control_plant = previousDepth;
  } else {
    // Otherwise, use the new value
    control_plant = depthValue;
    previousDepth = depthValue;
  }
}

void getTime(void){
  setCpuFrequencyMhz(240);
  WiFi.mode(WIFI_STA);
  WiFi.setHostname("float-esp32");
  MDNS.begin("float-esp32");
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
  if (diving == false)
  {
    diving = true;
    Serial.println("I'ma divin', bitch!");
    pDiveTime = millis(); 
    runNum++;  
  } 
}
