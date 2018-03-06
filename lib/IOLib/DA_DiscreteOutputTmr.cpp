/**
 *  @file    DA_DiscreteOutputTmr.h
 *  @author  peter c
 *  @date    4/10/2017
 *  @version 0.1
 *
 *
 *  @section DESCRIPTION
 *  Simple class to represent an arduinio discrete output with on/off duration
 *
 *
 */
#include <Streaming.h>
#include "DA_DiscreteOutputTmr.h"
DA_DiscreteOutputTmr::DA_DiscreteOutputTmr(uint8_t      aPin,
                                           bool         aActiveState,
                                           unsigned int aOnDurationInSec,
                                           unsigned int aOffDurationInSec) :
  DA_DiscreteOutput(aPin, aActiveState)
{
  onDurationInMilliSec  = (unsigned long)aOnDurationInSec * 1000;
  offDurationInMilliSec = (unsigned long)aOffDurationInSec * 1000;
}

bool DA_DiscreteOutputTmr::setOnDuration(unsigned int aOnDurationInSec)
{
  bool retVal = false;

  if (aOnDurationInSec > 0)
  {
    onDurationInMilliSec = (unsigned long)aOnDurationInSec * 1000;

    if (isOnTimerActive()) {
      timerPreset       = onDurationInMilliSec;
      timerCurrentValue = timerPreset;
    }
    retVal = true;
  }
  return retVal;
}

bool DA_DiscreteOutputTmr::setOffDuration(unsigned int aOffDurationInSec)
{
  bool retVal = false;

  if (aOffDurationInSec > 0)
  {
    offDurationInMilliSec = (unsigned long)aOffDurationInSec * 1000;

    if (!isOnTimerActive()) {
      timerPreset       = offDurationInMilliSec;
      timerCurrentValue = timerPreset;
    }
    retVal = true;
  }
  return retVal;
}

void DA_DiscreteOutputTmr::restart()
{
  DA_DiscreteOutput::reset();
  firstRun = true;
}

void DA_DiscreteOutputTmr::serialize(HardwareSerial *tracePort, bool includeCR)
{
  DA_DiscreteOutput::serialize(tracePort, false);
  *tracePort << "{onDurationInMilliSec:" <<  onDurationInMilliSec <<
    " offDurationInMilliSec:" << offDurationInMilliSec << " startMode:" <<
    startMode << " timerCurrentValue:" << timerCurrentValue << "}";

  if (includeCR) *tracePort << endl;
}

void DA_DiscreteOutputTmr::setStartDefault(bool aStartMode)
{
  startMode = aStartMode;
}

bool DA_DiscreteOutputTmr::refresh()
{
  if (firstRun)
  {
    // force a start with active state
    if (startMode)
    {
      timerPreset = onDurationInMilliSec;
      activateOnTimer();
      DA_DiscreteOutput::activate();
      timerCurrentValue = timerPreset;
    }
    else
    {
      timerPreset = offDurationInMilliSec;
      activateOffTimer();
      DA_DiscreteOutput::reset();
      timerCurrentValue = timerPreset;
    }
    firstRun = false;
  }


  if (isTimerPaused())
  {
    return true;
  }
  unsigned long currentEpoch = millis();
  unsigned long deltaT       = abs(currentEpoch - previousEpoch);

  if ((deltaT -  pausedDuration) >= timerPreset)
  {
    if (isOnTimerActive())
    {
      timerPreset = offDurationInMilliSec;
      activateOffTimer();
    }
    else
    {
      timerPreset = onDurationInMilliSec;
      activateOnTimer();
    }
    DA_DiscreteOutput::toggle();
    previousEpoch  = currentEpoch;
    currentEpoch   = 0;
    pausedDuration = 0;
  }
  timerCurrentValue =  timerPreset - deltaT + pausedDuration;
  return true;
}

unsigned long DA_DiscreteOutputTmr::getCurrentOnDuration()
{
  if (isOnTimerActive()) return timerCurrentValue;

  return onDurationInMilliSec;
}

unsigned long DA_DiscreteOutputTmr::getCurrentOffDuration()
{
  if (!isOnTimerActive()) return timerCurrentValue;

  return offDurationInMilliSec;
}

void DA_DiscreteOutputTmr::pauseTimer()
{
  timerPaused = true;
  pausedStart = millis();
}

void DA_DiscreteOutputTmr::resetTimer()
{}

void DA_DiscreteOutputTmr::resumeTimer()
{
  if (isTimerPaused())
  {
    timerPaused    = false;
    pausedDuration = abs(millis() - pausedStart);

    if (isOnTimerActive()) DA_DiscreteOutput::activate();
    else DA_DiscreteOutput::reset();
  }
}

bool DA_DiscreteOutputTmr::isTimerPaused()
{
  return timerPaused;
}
