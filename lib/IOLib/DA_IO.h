/**
 *  @file    DA_IO.h
 *  @author  peter c
 *  @date    25/09/2017
 *  @version 0.1
 *
 *
 *  @section DESCRIPTION
 *  Common across input and outputs
 */

#ifndef DA_IO_H
#define DA_IO_H
#define DA_RAW_MIN 0
#define DA_RAW_MAX 1023


enum IO_TYPE
{
  discrete, analog, i2c_ec, i2c_ph
};

#endif // ifndef DA_IO_H
