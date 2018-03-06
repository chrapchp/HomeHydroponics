/*
   Atlas Semiconductor pH Reader
   https://www.atlas-scientific.com/product_pages/circuits/ezo_ph.html
   Author: Peter
   Date:  2018Jan13
 */

#ifndef DA_ATALAS_EC__H
#define DA_ATALAS_EC_H
#include "DA_Atlas.h"

// #define EC_POLLING_INTERVAL 3000 // ms - no check no less than 900ms
// #define EC_READ_COMMAND 'r'
#define EC_DEFAULT_I2C_ADDRESS 101

class DA_AtlasEC : public DA_Atlas {
public:

  DA_AtlasEC(uint8_t address);

  // unsigned int getRawSample();
  inline float getSample() __attribute__((always_inline))
  {
    return currentValue;
  }

  inline float getTDS() __attribute__((always_inline))
  {
    return TDS;
  }

  inline float getSalinity() __attribute__((always_inline))
  {
    return salinity;
  }

  inline float getSpecificGravity() __attribute__((always_inline))
  {
    return specificGravity;
  }

  bool calibrateHigh();
  bool calibrateDry();
  bool calibrateLow();
  void serialize(HardwareSerial *tracePort,
                 bool            includeCR);

protected:

  float parseReadResponse(char *probeData);

private:

  // bool isOutsideDeadband( float aNewValue,  float aCurValue );  //
  float currentValue    = 0;
  float TDS             = 0;
  float salinity        = 0;
  float specificGravity = 0;
};

#endif // ifndef DA_ATALAS_EC__H
