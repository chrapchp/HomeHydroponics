#include <Streaming.h>
#include "DA_Input.h"
DA_Input::DA_Input(IO_TYPE aInputType, uint8_t aPin)
{
  inputType = aInputType;
  pin       = aPin;
  pinMode(pin, INPUT);
}

DA_Input::DA_Input(IO_TYPE aInputType)
{
  inputType = aInputType;
  pin       = -1; // input without a pin
}

void DA_Input::refresh()
{
  if (pollingEnabled)
  {
    unsigned long currentTime = millis();

    if (currentTime - lastUpdateTime > pollingInterval)
    {
      lastUpdateTime = currentTime;
      onRefresh();
    }
  }
}

void DA_Input::setPollingInterval(unsigned int aPollingInterval)
{
  pollingInterval = aPollingInterval;
}

void DA_Input::serialize(HardwareSerial *tracePort, bool includeCR)
{
  *tracePort << "{pin:" << pin << " inputType:" <<
  (inputType ==
   discrete ? 'D' : 'A') << " pollingInterval:" << pollingInterval <<
    "pollingEnabled:" << pollingEnabled << " }";

  if (includeCR) *tracePort << endl;
}

void DA_Input::suspendPoll()
{
  pollingEnabled = false;
}

void DA_Input::resumePoll()
{
  pollingEnabled = true;
}
