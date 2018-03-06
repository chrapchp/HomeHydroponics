#include <Streaming.h>
#include "DA_AnalogInput.h"

// #define RAW_IN 0
// #define RAW_OUT 1023

DA_AnalogInput::DA_AnalogInput(uint8_t aPin, float aEUMin,
                               float aEUMax) : DA_Input(analog, aPin)
{
  euMin  = aEUMin;
  euMax  = aEUMax;
  euSpan = euMax - euMin;
}

void DA_AnalogInput::setEUMin(float aEUMin)
{
  euMin  = aEUMin;
  euSpan = euMax - euMin;
}

/**
 * [DA_AnalogInput description]
 * @type {[type]}
 */
void DA_AnalogInput::setEUMax(float aEUMax)
{
  euMax  = aEUMax;
  euSpan = euMax - euMin;
}

/*
   unsigned int DA_AnalogInput::getRawSample() {
   return ( currentRawSample );
   }

   float DA_AnalogInput::getScaledSample() {
   return ( currentScaledSample );

   }

 */
float DA_AnalogInput::fmap(unsigned int raw,
                           unsigned int raw_min,
                           unsigned int raw_max,
                           float        EU_min,
                           float        EU_max)
{
  float retVal;

  // Serial2 << "num=" << (raw - raw_min) * (EU_max - EU_min) << " dem=" <<
  // (raw_max - raw_min) << " offset=" << EU_min << endl;
  retVal = (raw - raw_min) * (EU_max - EU_min) / (raw_max - raw_min) + EU_min;

  // Serial << retVal << endl;
  return retVal;
}

bool DA_AnalogInput::isOutsideDeadband(int aNewValue,  int aCurValue)
{
  // Serial << "delta=" << abs(aNewValue - aCurValue) << " deadband=" <<
  // deadBand * 1023 << endl;
  return abs(aNewValue - aCurValue) >=  deadBand * euSpan;
}

/*
   bool DA_AnalogInput::isOutsideDeadband(float aNewValue, float aCurValue ) {
   float result = constrain( aNewValue, aCurValue * (1 - deadBand), aCurValue *
      (1 + deadBand));
   Serial << "current value=" << aCurValue << " result=" << result << "neval="
      << aNewValue << " LDB=" <<  aCurValue * (1 - deadBand) << " hDB=" <<
      aCurValue * (1 + deadBand) << endl;
   return ( !(result == aNewValue));
   }
 */
void DA_AnalogInput::onRefresh()
{
  // pinMode(pin, OUTPUT);
  //  analogWrite(pin, 0 );
  //  pinMode( pin, INPUT);
  // delay(10);
  unsigned int pendingRawValue = analogRead(pin);
  float pendingScaledValue     = fmap(pendingRawValue,
                                      DA_RAW_MIN,
                                      DA_RAW_MAX,
                                      euMin,
                                      euMax);

  if (isFirstSample || isOutsideDeadband(pendingRawValue, previousRawSample))
  {
    currentScaledSample = pendingScaledValue;
    previousRawSample   = pendingRawValue;

    if (onOutsideDeadbandDetected != NULL)
    {
      // Serial << getScaledSample() << endl;
      onOutsideDeadbandDetected(getScaledSample());
    }
  }
  currentRawSample =  pendingRawValue;

  if (onPoll != NULL)
  {
    onPoll(pendingScaledValue);
  }
  isFirstSample = false;
}

void DA_AnalogInput::setOnPollCallBack(void (*callBack)(float scaledValue))
{
  onPoll = callBack;
}

void DA_AnalogInput::setOutsideDeadbandDetectedEvent(void (*callBack)(
                                                       float scaledValue))
{
  onOutsideDeadbandDetected = callBack;
}

void DA_AnalogInput::setDeadband(float aPercentage) // +/- .11 -> 11% deadband
{
  deadBand = aPercentage;
}

void DA_AnalogInput::serialize(HardwareSerial *tracePort, bool includeCR)
{
  DA_Input::serialize(tracePort, false);

  *tracePort << "{raw in:" << getRawSample()  << " scaled Sample:" <<
  getScaledSample() << " deadband:" << deadBand << "euMin:" << euMin <<
  "euMax:" << euMax << "}";

  if (includeCR) *tracePort << endl;
}

float euMin  = 0.0;
float euMax  = 1023.0;
float euSpan = euMax - euMin;

float fmap(unsigned int raw,
           unsigned int raw_min,
           unsigned int raw_max,
           float        EU_min,
           float        EU_max);

bool isOutsideDeadband(int aNewValue,
                       int aCurValue); //
// bool isOutsideDeadband(float aCurValue ); // deadband from full EU range
unsigned int currentRawSample  = 0;
unsigned int previousRawSample = 0;
float currentScaledSample      = 0;

float deadBand = DEAD_BAND_DEFAULT;
void  (*onPoll)(float value) = NULL;
void  (*onOutsideDeadbandDetected)(float aValue) = NULL;
void onRefresh();
bool isFirstSample = true; // to avoid false edge dectection
