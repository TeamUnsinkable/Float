#include <WebHandler.hpp>

// ------------ Webserver Setup ------------
AsyncEventSource events("/events");
QueueHandle_t logQueue = xQueueCreate(LOG_QUEUE_SIZE, sizeof(char[128]));


// ------------ Serial Handing ------------
void serialLog(const String &msg) {
    Serial.println(msg);
    // Only bother queuing if someone is actually connected
    if (events.count() == 0) return;
    char buf[128];
    msg.substring(0, 127).toCharArray(buf, sizeof(buf));
    xQueueSendToBack(logQueue, buf, 0);  // non-blocking, drop if full
}


void drainLogQueue() {
    // Bail immediately if no browser — no work done at all
    if (events.count() == 0) {
        // Flush stale queue entries so they don't burst on reconnect
        char buf[128];
        while (xQueueReceive(logQueue, buf, 0) == pdTRUE) {}
        return;
    }
    // Drain one message per loop tick max
    char buf[128];
    if (xQueueReceive(logQueue, buf, 0) == pdTRUE) {
        events.send(buf, "log", millis());
    }
}

void broadcastTelemetry() {
    if (events.count() == 0) return;  // zero cost when no client
    if (millis() - lastTx < 200) return;  // max 5 Hz
    lastTx = millis();

    String json = "{";
    json += "\"depth\":"    + String(depthMeter, 4)           + ",";
    json += "\"pressure\":" + String(depthPascal / 10.0f, 2)  + ",";
    json += "\"setpoint\":" + String(control_setpoint, 4)     + ",";
    json += "\"output\":"   + String(control_output, 2);
    json += "}";
    events.send(json.c_str(), "telemetry", millis());
}

// ------------ 404 Handler ------------
void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

// ------------ Webserver Endpoints ------------
void setupWebServer() {

    // Configure 404 handler
    server1.onNotFound(notFound);
    // Add event handler
    server1.addHandler(&events);

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
      surface();
      request->send(200, "text/plain", "ok");
    });


  server1.on("/getTuning", HTTP_GET, [] (AsyncWebServerRequest *request) {
      String json = "{";
      json += "\"ki\":" + String(Ki, 6) + ",";
      json += "\"kp\":" + String(Kp, 6) + ",";
      json += "\"kd\":" + String(Kd, 6) + ",";
      json += "\"setpoint\":" + String(control_setpoint, 6) + ",";
      json += "\"loop_ms\":" + String(control_loop_rate_ms);
      json += "}";

      request->send(200, "application/json", json);
  });

  server1.on("/updateTuning", HTTP_GET, [] (AsyncWebServerRequest *request) {
      if (
          !request->hasParam("ki") ||
          !request->hasParam("kp") ||
          !request->hasParam("kd") ||
          !request->hasParam("setpoint") ||
          !request->hasParam("loop_ms")
      ) {
          request->send(400, "text/plain", "Missing tuning parameter.");
          return;
      }

      double newKi = request->getParam("ki")->value().toDouble();
      double newKp = request->getParam("kp")->value().toDouble();
      double newKd = request->getParam("kd")->value().toDouble();
      double newSetpoint = request->getParam("setpoint")->value().toDouble();
      uint32_t newLoopMs = request->getParam("loop_ms")->value().toInt();

      if (newLoopMs < 1) {
          request->send(400, "text/plain", "Loop time must be at least 1 ms.");
          return;
      }

      BangBangBoi.stop();

      Ki = newKi;
      Kp = newKp;
      Kd = newKd;
      control_setpoint = newSetpoint;
      control_loop_rate_ms = newLoopMs;

      BangBangBoi.setGains(Kp, Ki, Kd);
      BangBangBoi.setTimeStep(control_loop_rate_ms);

      BangBangBoi.reset();

      Serial.println("Updated tuning:");
      Serial.println("Kp: " + String(Kp, 6));
      Serial.println("Ki: " + String(Ki, 6));
      Serial.println("Kd: " + String(Kd, 6));
      Serial.println("Setpoint: " + String(control_setpoint, 6));
      Serial.println("Loop time ms: " + String(control_loop_rate_ms));

      request->send(200, "text/plain", "Tuning values pushed.");
  });


    server1.on("/getReadingsCSV", HTTP_GET,
    [](AsyncWebServerRequest *request)
    {
        size_t currentRow = 0;

        AsyncWebServerResponse *response =
            request->beginChunkedResponse(
                "text/csv",
                [currentRow](uint8_t *buffer, size_t maxLen, size_t index) mutable -> size_t
                {
                  size_t len = 0;
                  if (index == 0)
                  {
                    len += snprintf(
                        (char*)buffer,
                        maxLen,
                        "runNumber,lHour,lMin,lSec,depthPa,depthM,packet\n");
                  }

                  while (currentRow < readingCnt)
                  {
                    char line[128];
                    int written = snprintf(
                        line,
                        sizeof(line),
                        "%d,%d,%d,%d,%.3f,%.3f,%c\n",
                        psram_Readings[currentRow].runNumber,
                        psram_Readings[currentRow].lHour,
                        psram_Readings[currentRow].lMin,
                        psram_Readings[currentRow].lSec,
                        psram_Readings[currentRow].depthPa,
                        psram_Readings[currentRow].depthM,
                        psram_Readings[currentRow].packet
                    );

                    if (len + written > maxLen)
                    {
                      break;
                    }
                    memcpy(buffer + len, line, written);
                    len += written;
                    currentRow++;
                  }
                  return len;
                });
        response->addHeader(
            "Content-Disposition",
            "attachment; filename=readings.csv");
        request->send(response);
    });

    server1.begin();
}   