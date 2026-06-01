#ifndef WEBHANDLER_HPP
#define WEBHANDLER_HPP
#include <ESPAsyncWebServer.h>
#include <AutoPID.h>
#include <Arduino.h>
#include <DataLogging.hpp>
#include <html_rendered.h>

// ------------ Webserver variables ------------
extern AsyncWebServer server1;
extern const char index_html[] PROGMEM;

// ----------- Data Logging variables ------------
extern int readingCnt;
extern sReadings *psram_Readings;

// ------------ PID Control variables ------------
extern double Ki, Kp, Kd;
extern AutoPID BangBangBoi;
extern double control_setpoint;
extern uint32_t control_loop_rate_ms;

// ----------- Function declarations ------------
/**
 * @brief Handler for 404 Not Found errors.
 *
 * This function is called when a client requests a resource that does not exist on the server.
 * It sends a 404 response back to the client with a simple message indicating that the resource was not found.
 *
 * @param request Pointer to the AsyncWebServerRequest object representing the client's request.
 */
void notFound(AsyncWebServerRequest *request);

/**
 * @brief Set up the web server.
 */
void setupWebServer();

extern void dive();

#endif