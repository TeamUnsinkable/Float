#ifndef DATALOGGING_HPP
#define DATALOGGING_HPP

typedef struct {
    int runNumber;    // Identifier for the dive run
    int lHour;        // Local time hour when the reading was taken
    int lMin;         // Local time minute when the reading was taken
    int lSec;         // Local time second when the reading was taken
  float depthPa;      // Depth in Pascals
  float depthM;       // Depth in Meters
  uint32_t  packet;   // Packet identifier for the reading (e.g., sequential number)
} sReadings;

#endif
