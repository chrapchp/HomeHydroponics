/**
 *  @file    DA_NonBlockingDelay.h
 *  @author  peter c
 *  @date    4/14/2017
 *  @version 0.1
 *
 *
 *  @section DESCRIPTION
 *  Simple class to deal with non-blocking delay
 */
#include <Streaming.h>
#include "DA_NonBlockingDelay.h"
DA_NonBlockingDelay::DA_NonBlockingDelay(unsigned long aDelayinMillSeconds,
                                         void (*callBack)())
{
  onDelayComplete = callBack;
  desiredDelay    = aDelayinMillSeconds;
}

void DA_NonBlockingDelay::setDelay(unsigned long aDelayinMillSeconds)
{
  desiredDelay = aDelayinMillSeconds;
}

void DA_NonBlockingDelay::setCallback(void (*callBack)())
{
  onDelayComplete = callBack;
}

void DA_NonBlockingDelay::serialize(HardwareSerial *tracePort, bool includeCR)
{
  *tracePort << "{delay:" << desiredDelay << " previousTime:" << previousTime;
  *tracePort << endl;
}

void DA_NonBlockingDelay::refresh()
{
  unsigned long currentTime;

  currentTime = millis();


  if ((abs(currentTime - previousTime)) >= desiredDelay)
  {
    previousTime = currentTime;

    if ((onDelayComplete != NULL) && not paused)
    {
      onDelayComplete();
    }
  }
}

void DA_NonBlockingDelay::pause()
{
  paused = true;
}

void DA_NonBlockingDelay::resume()
{
  paused = false;
}
