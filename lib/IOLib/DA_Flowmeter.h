/*
   FlowMeter based on
   Flow meter from http://www.seeedstudio.com/wiki/G1/2_Water_Flow_sensor +/- 3%
      acuracy
   Author: Peter
   Date: 2k13
 */

#ifndef FlowMeter_h
#define FlowMeter_h

// #include "WProgram.h"
// #include <Arduino.h>  // was WProgram.h, changed to Arduino.h in 1.0
#include <HardwareSerial.h>


class FlowMeter {
public:

  FlowMeter(int aPin,
            int aDeltaT); // deltaT in ms

  void         begin();
  void         end();
  void         setUnits(bool perSec);  // perSec = true, per Minute = false;
  unsigned int getCurrentPulses();     // pulses/sec
  float        getCurrentFlowRate();   // in L/sec or L/Min
  float        getPreviousFlowRate();  // in L/sec or L/Min
  float        getCummulativeVolume(); // in L

  void         resetStatistics();
  void         dayRollOver();

  long         getMinFlowDuration();     // in seconds
  long         getMaxFlowDuration();     // in seconds
  long         getAverageFlowDuration(); // in seconds
  long         getTotalFlowDuration();
  void         handleFlowDetection();
  void         serialize(HardwareSerial *tracePort,
                         bool            includeCR);

protected:

  float computeFlowRate();

private:

  //   init( int aPin );
  void updateCummulativeVolume();
  void updateFlowRunTime();
  float mCurrentFlowRate;
  float mPreviousFlowRate;
  float mCummulativeVolume;
  float mYDAYCumulativeVolume;
  long mMaxFlowDuration;
  long mMinFlowDuration;
  long mAverageFlowDuration;
  int mPin;
  volatile unsigned int mPulseCount;
  unsigned int mPrevPulseCount;
  int mDeltaT;
  long mCurrentFlowDuration; // ms
  unsigned long mTotalFlowDuration;
  long mFlowCounts;
  bool perSecond = false;
};

#endif // ifndef FlowMeter_h
