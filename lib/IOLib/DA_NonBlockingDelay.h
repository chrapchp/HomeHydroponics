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

#ifndef DA_NONBLOCKINIGDELAY_H
#define DA_NONBLOCKINIGDELAY_H


#include <HardwareSerial.h>


class DA_NonBlockingDelay {
public:

  DA_NonBlockingDelay(unsigned long aDelayinMillSeconds,
                      void(*callBack)());
  void setDelay(unsigned long aDelayinMillSeconds);
  void setCallback(void (*callBack)());
  void serialize(HardwareSerial *tracePort,
                 bool            includeCR);
  void refresh();
  void pause();
  void resume();

private:

  unsigned long desiredDelay = 0;
  void (*onDelayComplete)() = NULL;
  unsigned long previousTime = 0;
  bool paused                = false;
};

#endif // ifndef DA_NONBLOCKINIGDELAY_H
