#include <Wire.h>
#include <Streaming.h>
#include "DA_Atlas.h"
DA_Atlas::DA_Atlas(uint8_t address, IO_TYPE io_type) : DA_Input(io_type)
{
  I2CAddress = address;
  setPollingInterval(ATLAS_POLLING_INTERVAL);
  probeState = reading;
}

void DA_Atlas::setCompensatedTemperature(float aTemperature)
{
  probeState = writingCompensatedTemperature;
  char tempCommand[7]; // "T,12.5"

  // there was a read request just before a temperature compensate
  // finish the read
  if (readResponsePending)
  {
    processReadResponse(false);
    readResponsePending = false;
  }
  char floatStr[7];
  dtostrf(aTemperature, 4, 1, floatStr);
  sprintf(tempCommand, "T,%s", floatStr);

  // Serial << " setCompensatedTemperature(" << aTemperature << ") tempCommand:"
  // << tempCommand << endl;
  sendCommand(tempCommand);
  tempCompensateResponsePending = true;
  pendingCompensatedTemperature = aTemperature;
}

void DA_Atlas::sendCommand( char *aCommand)
{
  //// Serial << "sendCommand(" << aCommand << ")" << endl;
  Wire.beginTransmission(I2CAddress); // call the circuit by its ID number.
  Wire.write(aCommand);               // transmit the command
  Wire.endTransmission();             // end the I2C data transmission.
}

void DA_Atlas::sendCommand(char aCommand)
{
  //// Serial << "sendCommand(" << aCommand << ")" << endl;
  Wire.beginTransmission(I2CAddress); // call the circuit by its ID number.
  Wire.write(aCommand);               // transmit the command
  Wire.endTransmission();             // end the I2C data transmission.
}

// response format
// status: 1 (succss) | 2 (failed) } 254 (pending) | 255 (no data)
// comma seperated response depending on request. refer to sensor datasheet
uint8_t DA_Atlas::receiveResponse(bool aDataExpected, char *aResponseBuffer)
{
  Wire.requestFrom(I2CAddress, ATLAS_RESPONSE_BUFFER_SIZE, 1);
  probeCommandStatus = Wire.read();

  //// Serial << "receiveResponse:probeCommandStatus:" << probeCommandStatus <<
  // endl;
  if (aDataExpected)
  {
    int  i = 0;
    char rxByte;

    while (Wire.available())
    {
      rxByte                 = Wire.read();
      *(aResponseBuffer + i) = rxByte;
      i                     += 1;

      if (rxByte == 0)
      {
        Wire.endTransmission();
        break; // exit the while loop.
      }
    }
  }
  return probeCommandStatus;
}

float DA_Atlas::processReadResponse(bool invokeCallBack)
{
  char  probeData[ATLAS_RESPONSE_BUFFER_SIZE];
  float currentValue = -1;

  if (receiveResponse(true, probeData) == 1)
  {
    currentValue = parseReadResponse(probeData);

    if ((onPoll != NULL) && invokeCallBack)
    {
      onPoll(getInputType(), currentValue);
    }
  }
  return currentValue;
}

void DA_Atlas::retrieveCompensatedTemperature()
{
  char probeData[ATLAS_RESPONSE_BUFFER_SIZE];

  sendCommand((char *) "T,?");
  delay(ATLAS_WAIT_INTERVAL);

  if (receiveResponse(true, probeData) == 1)
  {
    atof(strtok(probeData, ","));
    compensatedTemperature = atof(strtok(NULL, ","));
  }
}

// read probe data e.g. pH, EC
void DA_Atlas::doRead()
{
  if (!readResponsePending)
  {
    sendCommand(ATLAS_READ_COMMAND);
  }
  else
  {
    processReadResponse(true);
  }

  // setPollingInterval( previousPollingInterval );
  readResponsePending = !readResponsePending;
}

// each poll request alternates between send command and read the results
// no call to delay()
void DA_Atlas::onRefresh()
{
  switch (probeState)
  {
  case reading:
    doRead();
    break;

  case calibrating:

    // statement(s);
    break;

  case writingCompensatedTemperature:

    if (tempCompensateResponsePending == true)
    {
      if (receiveResponse(false, NULL) == 1)
      {
        compensatedTemperature = pendingCompensatedTemperature;

        // Serial << "compensatedTemperature =" << compensatedTemperature <<
        // endl;
      }
      tempCompensateResponsePending = false;
      probeState                    = reading;
    }

    // statement(s);
    break;

  default:
    doRead();

    // statement(s);
  }
}

void DA_Atlas::setOnPollCallBack(void (*callBack)(IO_TYPE type,
                                                  float scaledValue))
{
  onPoll = callBack;
}

bool DA_Atlas::calibrateClear()
{
  sendCommand((char *) "Cal,clear");
  delay(ATLAS_WAIT_INTERVAL);
  return (receiveResponse(false, NULL) == 1) ? true : false;
}

int DA_Atlas::calibrateQuery()
{
  char probeData[ATLAS_RESPONSE_BUFFER_SIZE];
  int  retVal = -1;

  sendCommand((char *) "Cal,?");
  delay(ATLAS_WAIT_INTERVAL);

  if (receiveResponse(true, probeData) == 1)
  {
    atof(strtok(probeData, ","));
    retVal = atof(strtok(NULL, ","));
  }
  return retVal;
}

/*
   void DA_Atlas::changeI2CAddress(uint8_t address)
   {
   char tempCommand[7];
   I2CAddress = address;
   sprintf(tempCommand, "I2C,%u", address);
   sendCommand(tempCommand);
   }
 */
