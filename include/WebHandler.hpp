#ifndef WEBHANDLER_HPP
#define WEBHANDLER_HPP
#include <ESPAsyncWebServer.h>
#include <AutoPID.h>
#include <Arduino.h>
#include <DataLogging.hpp>
#include <html_rendered.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>


// ------------ Webserver variables ------------
extern AsyncWebServer server1;
extern const char index_html[] PROGMEM;

// ----------- Data Logging variables ------------
extern int readingCnt;
extern sReadings *psram_Readings;
static uint32_t lastTx = 0;

// ------------ PID Control variables ------------
extern double Ki, Kp, Kd;
extern AutoPID BangBangBoi;
extern double control_setpoint, control_output;
extern uint32_t control_loop_rate_ms;
// For live telemetry updates
extern float depthPascal, depthMeter;

// ------------ Serial Logger Variables ------------
#define LOG_QUEUE_SIZE 20
extern AsyncEventSource events;
// QueueHandle_t logQueue;

// ------------ Serial Log Helper -----------------
/**
 * @brief Sends a message to the hardware Serial port and, if an SSE client
 *        is connected, enqueues it for delivery to the web serial-log panel.
 *
 * This function is a drop-in replacement for Serial.println() throughout the
 * codebase. It guarantees that serial output is never suppressed -- the
 * hardware Serial.println() call is unconditional. The SSE path is gated
 * behind an events.count() check so that no heap allocation, string copy, or
 * queue interaction occurs during extended periods with no browser connected.
 *
 * Enqueueing is non-blocking. If the queue is full (LOG_QUEUE_SIZE messages
 * are already pending), the incoming message is silently dropped rather than
 * blocking the caller. This is intentional -- dropping a log line is always
 * preferable to introducing jitter into the control loop.
 *
 * @param msg The log message to emit. Truncated to 127 characters before
 *            being placed on the queue to fit the fixed-size queue buffer.
 *            The full string is always forwarded to Serial regardless of
 *            length.
 *
 * @note This function is safe to call from loop() (Core 1) and from any
 *       context where Serial.println() would ordinarily be used. Do NOT
 *       call it from within an AsyncWebServer request handler or the
 *       async_tcp task (Core 0) -- use Serial.println() directly there
 *       to avoid cross-core queue contention.
 *
 * @note Messages enqueued while a client is connected but not yet drained
 *       will be delivered in order by drainLogQueue(). Messages enqueued
 *       with no client present are never queued and will not appear in the
 *       browser on reconnect.
 *
 * @see drainLogQueue() for the consumer side of the queue.
 * @see LOG_QUEUE_SIZE for maximum queue depth before drops occur.
 */
void serialLog(const String &msg);

/**
 * @brief Broadcast a telemetry JSON object to the web UI via SSE.
 *        Call once per loop() iteration to update live metric cards.
 */
void broadcastTelemetry();


/**
 * @brief Drains pending log messages from the queue and forwards them to
 *        connected SSE clients. Must be called once at the end of loop().
 *
 * This function is the consumer side of the non-blocking log pipeline
 * established by serialLog(). It is designed to have a bounded, predictable
 * cost per call regardless of how many messages are queued, ensuring it
 * cannot starve the stepper controller or PID loop of CPU time.
 *
 * Behavior varies based on client connection state:
 *
 *   No client connected:
 *     The queue is flushed in its entirety without performing any TCP I/O or
 *     heap allocation. This prevents stale messages accumulated during a long
 *     unmanned dive from bursting to the browser the moment a client
 *     reconnects, which would be misleading and could overwhelm the SSE
 *     buffer. The flush itself is a tight receive loop with zero blocking.
 *
 *   Client connected:
 *     At most one message is dequeued and forwarded to the SSE event source
 *     per call. Limiting throughput to one message per loop() tick bounds
 *     the worst-case time spent in TCP I/O to a single send operation,
 *     keeping web delivery latency proportional to the loop rate rather than
 *     to the depth of the queue.
 *
 * @note Call this function exactly once, at the very end of loop(), after
 *       all sensor reads, PID computation, and stepper commands have
 *       completed. Placing it last ensures web I/O never delays time-
 *       critical control logic.
 *
 * @note This function is non-blocking. xQueueReceive() is called with a
 *       timeout of zero ticks in all cases. It will never yield or suspend
 *       the calling task.
 *
 * @note Thread safety: this function must only be called from loop()
 *       (Core 1). events.send() is not safe to call concurrently from
 *       multiple cores. serialLog() enqueues from Core 1 and this function
 *       dequeues from Core 1, so no cross-core contention exists on the
 *       queue consumer side.
 *
 * @see serialLog() for the producer side of the queue.
 * @see broadcastTelemetry() which follows the same no-client guard pattern.
 * @see LOG_QUEUE_SIZE for the maximum number of messages that can be queued
 *      before producers begin dropping.
 */
void drainLogQueue();
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
extern void surface();

#endif