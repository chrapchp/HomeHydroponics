/*
   Atlas Semiconductor base sensor class
   https://www.atlas-scientific.com/product_pages/circuits/ezo_ph.html
   Author: Peter
   Date:  2018Feb6
 */

#ifndef DA_ATALAS__H
#define DA_ATALAS__H
#include "DA_Input.h"
#define ATLAS_POLLING_INTERVAL 5000 // ms - no check no less than 900ms
#define ATLAS_READ_COMMAND 'r'
#define ATLAS_READ_WAIT_INTERVAL 900
#define ATLAS_WAIT_INTERVAL 300
#define ATLAS_RESPONSE_BUFFER_SIZE 48
class DA_Atlas : public DA_Input {
public:

  enum PROBESTATE
  {
    reading, calibrating, writingCompensatedTemperature,
    readingCompesatedTemperature
  };
  DA_Atlas(uint8_t address,
           IO_TYPE io_type);
  virtual void      serialize(HardwareSerial *tracePort,
                              bool            includeCR);
  void              doReadCommandResponse();
  void              setOnPollCallBack(void (   *callBack)(
                                        IO_TYPE type,
                                        float   currentValue));
  inline PROBESTATE getState() __attribute__((always_inline))
  {
    return probeState;
  }

  inline uint8_t getProbeCommandStatus() __attribute__((always_inline))
  {
    return probeCommandStatus;
  }

  inline uint8_t getI2CAddress() __attribute__((always_inline))
  {
    return I2CAddress;
  }

  // return "cached" compensated temperature
  inline float getCompensatedTemperature() __attribute__((always_inline))
  {
    return compensatedTemperature;
  }

  // blocking functions. It running in a strict timing evirionment,
  // errors may occur
  // calibrate High
  // bool calibrateHigh();
  // calibrate low
  // bool calibrateLow();
  // clear the current calibarion setting in sensor
  bool calibrateClear();

  // return the calibration point count -1 if err
  int  calibrateQuery();

  // change the temperature compensation in the sensor
  // non-blocking
  // sensor does not persist value between power cycles
  void setCompensatedTemperature(float aTemperature);

  // retrieve the temperature compensation value from sensor
  // blocking. Should be invoked during start-up only
  void    retrieveCompensatedTemperature();

  // send a multi-character command to sensor
  void    sendCommand(char *aCommand);

  // send a single character command
  void    sendCommand(char aCommand);
  uint8_t receiveResponse(bool  aDataExpected,
                          char *aResponseBuffer);

  // void changeI2CAddress( uint8_t address );

protected:

  // parse messages specific to the sensor
  virtual float parseReadResponse(char *probeData);

  // process message from a read sensor request
  float         processReadResponse(bool invokeCallBack);


  // perform  read sensor value
  void doRead();

private:

  void (*onPoll)(IO_TYPE type, float value) = NULL;
  void onRefresh();
  unsigned int responsWaitTime       = 900; // 900 ms for reads, 600 ms for
                                            // temperature and calibrate
  int I2CAddress                     = 0;
  bool readResponsePending           = false;
  bool tempCompensateResponsePending = false;
  uint8_t probeCommandStatus;
  PROBESTATE probeState = reading;
  unsigned int previousPollingInterval;
  float pendingCompensatedTemperature = 0.0;
  float compensatedTemperature        = 0.0;
};

#endif // ifndef DA_ATALAS__H
