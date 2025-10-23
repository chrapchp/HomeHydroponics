/**
 *  @file    DA_NutrientController.cpp
 *  @author  peter c
 *  @date    5/25/2018
 *  @version 0.1
 *
 *
 *  @section DESCRIPTION
 *  Implements nutrient pump control (peristalitic)
 *
 *
 */
#include <Streaming.h>

#include "DA_NutrientController.h"

DA_NutrientController::DA_NutrientController(uint8_t            aPin,
                                             bool               aActiveState,
                                             float              aSP,
                                             CONTROL_TREND_TYPE aControlTrend) :
  DA_PeristalticPump(aPin, aActiveState)
{
  sp                  = aSP;
  autoVolume          = DA_NUTRIENTCONTROL_VOLUME_DEFAULT;
  autoInterval        = DA_NUTRIENTCONTROL_TIME_DEFAULT;
  manualVolume        = DA_NUTRIENTCONTROL_VOLUME_DEFAULT;
  controlTrend        = aControlTrend;
  processOutofBand    = false;
  controlMode         = CONTROL_MODE_TYPE::Off;
  prevControlMode     = CONTROL_MODE_TYPE::Off;
  stopDeadband        = DA_NUTRIENTCONTROLLER_DEADBAND;
  correctingOutofBand = false;
  stop();
}

void DA_NutrientController::serialize(Stream *aOutputStream,  bool includeCR)
{
  *aOutputStream << "{sp:" << sp << " controlTrend:" << controlTrend <<
    " autoVolume:" <<
    autoVolume;
  *aOutputStream << " autoIntervalsp:" << autoInterval << " manualVolume:" <<
    manualVolume;
  *aOutputStream << " controlMode:" << controlMode << " processOutofBand:" <<
    processOutofBand;
  *aOutputStream << " pv:" << pv << " stopCondition:" << getStopCondition();
  *aOutputStream << " stopSP:" << stopSetpoint << " stopDeadband:" <<
  stopDeadband << " correctingOutofBand:" << correctingOutofBand << " isActive():" << isActive();
  *aOutputStream << endl;
}

bool DA_NutrientController::isValidControlParameters()
{
  return autoVolume != DA_NUTRIENTCONTROL_VOLUME_DEFAULT &&
         manualVolume != DA_NUTRIENTCONTROL_VOLUME_DEFAULT &&
         autoInterval != DA_NUTRIENTCONTROL_TIME_DEFAULT;
}

void DA_NutrientController::setControlMode(uint16_t aMode)
{
  if (aMode == 3) controlMode = Auto;
  else if (aMode ==
           1) controlMode = DA_NutrientController::Hand;
  else controlMode = DA_NutrientController::Off;
}

bool DA_NutrientController::refresh()
{
  switch (controlMode)
  {
  case CONTROL_MODE_TYPE::Hand:


    if (prevControlMode != CONTROL_MODE_TYPE::Hand)
    {
      dispenseVolume(
        manualVolume);

      prevControlMode     = CONTROL_MODE_TYPE::Hand;
      processOutofBand    = false;
      correctingOutofBand = false;
    }

    break;

  case CONTROL_MODE_TYPE::Off:

    if (prevControlMode != CONTROL_MODE_TYPE::Off)
    {

      stop();
      prevControlMode     = CONTROL_MODE_TYPE::Off;
      processOutofBand    = false;
      correctingOutofBand = false;
    }
    break;

  case CONTROL_MODE_TYPE::Auto:
    float tSP;

    tSP             = sp;
    prevControlMode = CONTROL_MODE_TYPE::Auto;

    if (processOutofBand)
    {
      tSP = getStopCondition();
    }

    float delta;
    delta = tSP - pv;

    processOutofBand = false;

    if (isValidControlParameters() &&
        (((controlTrend == RISINGTREND) && (delta < 0)) ||
         ((controlTrend == FALLINGTREND) && (delta > 0))))
    {
      processOutofBand = true;

      if (!correctingOutofBand)
      {
       dispenseVolumeEvery(
          autoVolume,
          autoInterval) ;

        correctingOutofBand = true;

        // refresh();
      }
    }
    else
    {
      processOutofBand    = false;
      correctingOutofBand = false;
      stop();

    }
    break;

  case CONTROL_MODE_TYPE::Unknown:
    processOutofBand    = false;
    correctingOutofBand = false;
    break;
  }
  return DA_DiscreteOutputTmr::refresh();
}

void DA_NutrientController::setAutoVolume(uint16_t aVolume)
{
  if (aVolume > 0) autoVolume = aVolume;
}

void DA_NutrientController::setAutoInterval(uint16_t aEveryInterval)
{
  if (aEveryInterval > 0) autoInterval = aEveryInterval;
}

void DA_NutrientController::setAutoControlParameters(uint16_t aVolume,
                                                     uint16_t aEveryInterval)
{
  if ((aVolume > 0) && (aEveryInterval > 0))
  {
    autoVolume   = aVolume;
    autoInterval = aEveryInterval;
  }
}

void DA_NutrientController::setManualVolume(uint16_t aVolume)
{
  if (aVolume > 0) manualVolume = aVolume;
}

void DA_NutrientController::setStopSetpoint(float aStopSetpoint)
{
  stopDeadband = 0.0f;
  stopSetpoint = aStopSetpoint;
}

void DA_NutrientController::setStopDeadband(float aStopDeadband)
{
  stopSetpoint = 0.0f;
  stopDeadband = aStopDeadband;
}

float DA_NutrientController::getStopCondition()
{
  float calcStopCondition;

  // if an actual stop condition was provided, then it overides the deadband
  // setting
  if (stopSetpoint != 0) calcStopCondition = stopSetpoint;

  // if no deadband or stopsetpoint use default deadband
  else {
    // check if the deadband is 0, then default to the defaut deadband.
    // then compute the stop condition.
    float tDeadBand = stopDeadband;

    if (tDeadBand == 0.0f) tDeadBand = DA_NUTRIENTCONTROLLER_DEADBAND;

    if (controlTrend ==
        RISINGTREND) calcStopCondition = sp * (1 - tDeadBand);
    else calcStopCondition = sp * (1 + tDeadBand);
  }


  return calcStopCondition;
}
