/*
   Atlas Semiconductor pH Reader
   https://www.atlas-scientific.com/product_pages/circuits/ezo_ph.html
   Author: Peter
   Date:  2018Jan13
 */

#ifndef DA_ATALAS_H
#define DA_ATALAS_H
#include "DA_Atlas.h"
#define PH_DEFAULT_I2C_ADDRESS 100

class DA_AtlasPH : public DA_Atlas {
public:

  DA_AtlasPH(uint8_t address);

  // unsigned int getRawSample();
  inline float getSample() __attribute__((always_inline))
  {
    return currentValue;
  }

  void serialize(HardwareSerial *tracePort,
                 bool            includeCR);

  // blocking calibrations
  bool calibrateHigh();
  bool calibrateMid();
  bool calibrateLow();

protected:

  float parseReadResponse(char *probeData);

private:

  // bool isOutsideDeadband( float aNewValue,  float aCurValue );  //
  float currentValue = 0;
};

#endif // ifndef DA_ATALAS_H
