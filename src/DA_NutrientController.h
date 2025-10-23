/**
 *  @file    DA_NutrientController.h
 *  @author  peter c
 *  @date    5/25/2018
 *  @version 0.1
 *
 *
 *  @section DESCRIPTION
 *  Simple control for injecting nutrients into nutrient tank
 *  If in auto dispense X ml every T s
 *  If in manual dispense X ml
 *
 */

#ifndef DA_NUTRIENTCONTROLLER_H
#define DA_NUTRIENTCONTROLLER_H
#include <DA_PeristalticPump.h>

#define DA_NUTRIENTCONTROL_DEFAULT -127
#define DA_NUTRIENTCONTROL_TIME_DEFAULT 0
#define DA_NUTRIENTCONTROL_VOLUME_DEFAULT 0
#define DA_NUTRIENTCONTROLLER_DEADBAND 0.05 // 5%


class DA_NutrientController : public DA_PeristalticPump   {
public:
enum CONTROL_TREND_TYPE { RISINGTREND, FALLINGTREND };


enum CONTROL_MODE_TYPE { Unknown, Hand, Off, Auto  };

DA_NutrientController(uint8_t aPin, bool aActiveState, float aSP,CONTROL_TREND_TYPE aControlTrend );

void          serialize(Stream *aOutputStream,
                        bool includeCR);

// start off in Off with default values. if vals are not valid Controller
// remains Off
bool isValidControlParameters();

inline CONTROL_MODE_TYPE getControlMode() {
        return controlMode;
}

// volumne to dispense V mL in auto
void setAutoVolume( uint16_t aVolume);
// interval between dispensing V mL during auto
void setAutoInterval( uint16_t aEveryInterval);
// combined two above functions
void setAutoControlParameters( uint16_t aVolume, uint16_t aEveryInterval );
// dispense V ml in auto. One shot.
void setManualVolume( uint16_t aVolume );

inline uint16_t getAutoVolume()
{
  return autoVolume;
}

inline uint16_t getAutoInterval()
{
  return autoInterval;
}
inline uint16_t getManualVolume()
{
  return manualVolume;
}


inline void setControlMode( CONTROL_MODE_TYPE aControlMode )
{
        controlMode = aControlMode;
}


void setStopSetpoint( float aStopSetpoint );
void setStopDeadband( float aStopDeadband );
void setControlMode( uint16_t aMode );

// trend to control in auto. rising above SP or falling below SP
inline void setTrendingMode( CONTROL_TREND_TYPE aControlTrend )
{
        controlTrend = aControlTrend;
}

inline void setSP( float aSP )
{
        sp = aSP;
}

inline float getSP(  )
{
        return sp;
}

inline void setPV( float aPV )
{
        pv = aPV;
}



bool refresh();


private:

  float getStopCondition();
float pv;
float sp;
/**
 *   spStop: when SP is met it will apply control until SP +/- spStop
 *   spStop = 0 implies deadband control.
 *   spStop = 0 and deadband = 0 is invalid for auto. will default to deadband at
 *   DA_NUTRIENTCONTROLLER_DEADBAND
 */
float stopSetpoint;
/**
 * deadband : when SP is met it will apply control until SP +/ deadband * sp
 *            deadband = 0 => deadband control not used. Assume spStop
 */
float stopDeadband;

CONTROL_MODE_TYPE controlMode;
CONTROL_MODE_TYPE prevControlMode;
CONTROL_TREND_TYPE controlTrend;
uint16_t autoVolume;
uint16_t autoInterval;

uint16_t manualVolume;
bool processOutofBand;     // true when process is out of band
bool correctingOutofBand;  // true when in auto and correcting SP-PV error

};

#endif // ifndef DA_PERISTALTICPUMP_H
