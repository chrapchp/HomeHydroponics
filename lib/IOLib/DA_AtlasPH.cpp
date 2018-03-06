#include <Wire.h>
#include <Streaming.h>
#include "DA_AtlasPH.h"
DA_AtlasPH::DA_AtlasPH(uint8_t address) : DA_Atlas(address, i2c_ec)
{
  // setPollingInterval(EC_POLLING_INTERVAL);
}

float DA_AtlasPH::parseReadResponse(char *probeData)
{
  currentValue = atof(probeData);
  return currentValue;
}

void DA_AtlasPH::serialize(HardwareSerial *tracePort, bool includeCR)
{
  *tracePort << "{ I2CAddress:" << getI2CAddress() << " pH:" << getSample() <<
    " compensatedTemperature:" << getCompensatedTemperature() << " status:" <<
    getProbeCommandStatus() << "}";

  if (includeCR) *tracePort << endl;
}

bool DA_AtlasPH::calibrateHigh()
{
  sendCommand((char *)"Cal,high,10.00");
  delay(ATLAS_WAIT_INTERVAL);
  return (receiveResponse(false, NULL) == 1) ? true : false;
}

bool DA_AtlasPH::calibrateMid()
{
  sendCommand((char *)"Cal,mid,7.00");
  delay(ATLAS_WAIT_INTERVAL);
  return (receiveResponse(false, NULL) == 1) ? true : false;
}

bool DA_AtlasPH::calibrateLow()
{
  sendCommand((char *)"Cal,low,4.00");
  delay(ATLAS_WAIT_INTERVAL);
  return (receiveResponse(false, NULL) == 1) ? true : false;
}
