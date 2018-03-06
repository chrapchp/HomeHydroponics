#include <Wire.h>
#include <Streaming.h>
#include "DA_AtlasEC.h"
DA_AtlasEC::DA_AtlasEC(uint8_t address) : DA_Atlas(address, i2c_ec)
{
  // setPollingInterval(EC_POLLING_INTERVAL);
}

float DA_AtlasEC::parseReadResponse(char *probeData)
{
  currentValue    = atof(strtok(probeData, ","));
  TDS             = atof(strtok(NULL, ","));
  salinity        = atof(strtok(NULL, ","));
  specificGravity = atof(strtok(NULL, ","));

  return currentValue;
}

bool DA_AtlasEC::calibrateHigh()
{
  sendCommand("Cal,high,12880");
  delay(ATLAS_WAIT_INTERVAL);
  return (receiveResponse(false, NULL) == 1) ? true : false;
}

bool DA_AtlasEC::calibrateDry()
{
  sendCommand("Cal,dry");
  delay(ATLAS_WAIT_INTERVAL);
  return (receiveResponse(false, NULL) == 1) ? true : false;
}

bool DA_AtlasEC::calibrateLow()
{
  sendCommand("Cal,low,1413");
  delay(ATLAS_WAIT_INTERVAL);
  return (receiveResponse(false, NULL) == 1) ? true : false;
}

void DA_AtlasEC::serialize(HardwareSerial *tracePort, bool includeCR)
{
  // DA_Input::serialize( tracePort, false);
  *tracePort << "{I2CAddress:" << getI2CAddress() << " EC:" << getSample() <<
  " TDS:" << getTDS() << " Salinity:" << getSalinity() << " Specific Gravity:" <<
  getSpecificGravity() << " compensatedTemperature:" <<
  getCompensatedTemperature() << " status:" << getProbeCommandStatus() << "}";

  if (includeCR) *tracePort << endl;
}
