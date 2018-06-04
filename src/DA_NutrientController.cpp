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
  sp               = aSP;
  autoVolume       = DA_NUTRIENTCONTROL_VOLUMNE_DEFAULT;
  autoInterval     = DA_NUTRIENTCONTROL_TIME_DEFAULT;
  manualVolume     = DA_NUTRIENTCONTROL_VOLUMNE_DEFAULT;
  controlTrend     = aControlTrend;
  processOutofBand = false;
  controlMode      = CONTROL_MODE_TYPE::COFF;
  prevControlMode  = CONTROL_MODE_TYPE::COFF;
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
  *aOutputStream << " pv:" << pv;
  *aOutputStream << endl;
}

bool DA_NutrientController::isValidControlParameters()
{
  return autoVolume != DA_NUTRIENTCONTROL_VOLUMNE_DEFAULT &&
         manualVolume != DA_NUTRIENTCONTROL_VOLUMNE_DEFAULT &&
         autoInterval != DA_NUTRIENTCONTROL_TIME_DEFAULT;
}

void DA_NutrientController::setControlMode(uint16_t aMode)
{
  if (aMode == 1) controlMode = COFF;
  else if (aMode ==
           2) controlMode = DA_NutrientController::CMANUAL;
  else controlMode = DA_NutrientController::CAUTO;
}

bool DA_NutrientController::refresh()
{
  switch (controlMode)
  {
  case CONTROL_MODE_TYPE::CMANUAL:


    if (prevControlMode != CONTROL_MODE_TYPE::CMANUAL)
    {
      dispenseVolume(
        manualVolume);
    //  Serial << "Manual" << endl;
      prevControlMode = CONTROL_MODE_TYPE::CMANUAL;
    }

    break;

  case CONTROL_MODE_TYPE::COFF:

    //  Serial << "Off" << endl;
    stop();
    prevControlMode = CONTROL_MODE_TYPE::COFF;
    break;

  case CONTROL_MODE_TYPE::CAUTO:
    float tSP;

    tSP = sp;

    // if controlling PV then stop when were are +/-5% depending on control
    // direction
    if (processOutofBand)
    {
      if (controlTrend ==
          RISINGTREND) tSP = sp * (1 - DA_NUTRIENTCONTROLLER_DEADBAND);
      else tSP = sp * (1 + DA_NUTRIENTCONTROLLER_DEADBAND);
    }

    float delta;
    delta = tSP - pv;

    processOutofBand = false;

    if (isValidControlParameters() &&
        (((controlTrend == RISINGTREND) && (delta < 0)) ||
         ((controlTrend == FALLINGTREND) && (delta > 0))))
    {
      processOutofBand = true;

      if (prevControlMode != CONTROL_MODE_TYPE::CAUTO)
      {
        //Serial << "DispensEvery()" << endl;
        dispenseVolumeEvery(
          autoVolume,
          autoInterval);
        prevControlMode = CONTROL_MODE_TYPE::CAUTO;

        // refresh();
      }
    }
    else
    {
      processOutofBand = false;
      prevControlMode  = CONTROL_MODE_TYPE::CSTANDBY;
      stop();
    }
    break;

  case CONTROL_MODE_TYPE::CSTANDBY:
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
