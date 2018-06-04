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
 *  If in manual dispense X ml over T s
 *
 */

#ifndef DA_NUTRIENTCONTROLLER_H
#define DA_NUTRIENTCONTROLLER_H
#include <DA_PeristalticPump.h>

#define DA_NUTRIENTCONTROL_DEFAULT -127
#define DA_NUTRIENTCONTROL_TIME_DEFAULT 0
#define DA_NUTRIENTCONTROL_VOLUMNE_DEFAULT 0
#define DA_NUTRIENTCONTROLLER_DEADBAND 0.05 // 5%


class DA_NutrientController : public DA_PeristalticPump   {
public:
enum CONTROL_TREND_TYPE { RISINGTREND, FALLINGTREND };
enum CONTROL_MODE_TYPE { COFF=0,  CMANUAL, CAUTO, CSTANDBY };

DA_NutrientController(uint8_t aPin, bool aActiveState, float aSP,CONTROL_TREND_TYPE aControlTrend );

void          serialize(Stream *aOutputStream,
                        bool includeCR);


bool isValidControlParameters();

inline CONTROL_MODE_TYPE getControlMode() {
        return controlMode;
}


void setAutoVolume( uint16_t aVolume);
void setAutoInterval( uint16_t aEveryInterval);
void setAutoControlParameters( uint16_t aVolume, uint16_t aEveryInterval );
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


void setControlMode( uint16_t aMode );


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

float pv;
float sp;
CONTROL_MODE_TYPE controlMode;
CONTROL_MODE_TYPE prevControlMode;
CONTROL_TREND_TYPE controlTrend;
uint16_t autoVolume;
uint16_t autoInterval;
uint16_t manualVolume;
bool processOutofBand;

};

#endif // ifndef DA_PERISTALTICPUMP_H
