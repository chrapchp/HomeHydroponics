/**
 *  @file    DA_HASelect.h
 *  @author  peter c
 *  @date    2025Sep30
 *  @version 0.1
 *
 *
 *  @section DESCRIPTION
 *  Adds persistence to HASelect using EEPROM
 */

#ifndef DA_HASELECT_H
#define DA_HASELECT_H

#include <device-types/HASelect.h>

class DA_HASelect : public HASelect
{
public:
    DA_HASelect(const char *uniqueId, int eepromAddr, const char *name = nullptr);
    void setEEPROMAddr(int aEEPROMAddr);
    bool persist(int8_t index);
    int8_t load();
    // Set select options and count them
    void setOptions(const char *options);
    // Get the maximum number of options
    uint8_t getMaxOptions();

private:
    int eepromAddr = -1;
    uint8_t maxOptions = 0;                    // Track number of options
    String optionsString;                      // Store options string for parsing (optional)
    uint8_t countOptions(const char *options); // Helper to count options
};
#endif