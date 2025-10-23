#include "DA_HASelect.h"
#include <avr/eeprom.h>
#include <Streaming.h>
DA_HASelect::DA_HASelect(const char *uniqueId, int eepromAddr, const char *name)
    : HASelect(uniqueId)
{
    if (name != nullptr)
    {
        setName(name);
    }

    setEEPROMAddr(eepromAddr);
}

void DA_HASelect::setEEPROMAddr(int aEEPROMAddr)
{
    eepromAddr = aEEPROMAddr;
}

bool DA_HASelect::persist(int8_t index)
{
    if (eepromAddr != -1)
    {
        if (index >= 0 && index < maxOptions)
        {
            int16_t writeVal = index;

            //   setCurrentState(index);

            eeprom_write_block((const void *)&writeVal, (void *)eepromAddr,
                               sizeof(writeVal));

            return true;
        }
    }
}

int8_t DA_HASelect::load()
{
    if (eepromAddr != -1)
    {
        int16_t index = -1;
        eeprom_read_block((void *)&index, (void *)eepromAddr, sizeof(index));

        if (index >= 0 && index < maxOptions)
        {

            setCurrentState(int8_t(index));
            return index;
        }
    }
    return -1;
}

void DA_HASelect::setOptions(const char *options)
{
    optionsString = options;            // Store for reference (optional)
    maxOptions = countOptions(options); // Count options
    HASelect::setOptions(options);      // Set options in base class
}

uint8_t DA_HASelect::countOptions(const char *options)
{

    if (!options || !*options)
        return 0; // Empty string check

    uint8_t count = 1; // At least one option if non-empty
    for (const char *p = options; *p; ++p)
    {
        if (*p == ';')
        {
            ++count; // Count commas to determine number of options
        }
    }
    return count;
}

uint8_t DA_HASelect::getMaxOptions()
{
    return maxOptions; // Return the number of options
}