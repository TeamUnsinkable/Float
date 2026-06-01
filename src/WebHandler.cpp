#include <WebHandler.hpp>

void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

void setupWebServer() {

    // Configure 404 handler
    server1.onNotFound(notFound);

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

      float newKi = request->getParam("ki")->value().toFloat();
      float newKp = request->getParam("kp")->value().toFloat();
      float newKd = request->getParam("kd")->value().toFloat();
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