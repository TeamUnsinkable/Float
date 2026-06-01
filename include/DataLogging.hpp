#ifndef DATALOGGING_HPP
#define DATALOGGING_HPP

typedef struct {
    int runNumber;
    int lHour;
    int lMin;
    int lSec;
  float depthPa;
  float depthM;
  uint32_t  packet;
} sReadings;

#endif
