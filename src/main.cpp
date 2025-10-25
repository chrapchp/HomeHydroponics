#include <Arduino.h>

/**
 *  @file   main.cpp
 *  @author  peter c
 *  @date    2018Mar6
 *  @version 1.0.0
 *
 *
 *  @section DESCRIPTION
 *   2025Sep15 = Converted to home assistant integration
 *
 */

#include <time.h>
#include <TimeLib.h>
#include <Timezone.h> // https://github.com/JChristensen/Timezone
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>      // for RTC
#include <DS3232RTC.h> // http://github.com/JChristensen/DS3232RTC
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <Streaming.h>
#include <Adafruit_Sensor.h>
#include <DHT.h> // DHT-22 humidity sensor

#include <DebugLog.h>
// #include <MemoryFree.h>

#include <ArduinoHA.h>

#include <DA_DiscreteInput.h>
#include <DA_DiscreteOutput.h>
#include <DA_DiscreteOutputTmr.h>
#include <DA_HOASwitch.h>
#include <DA_AtlasPH.h>
#include <DA_AtlasEC.h>
// #include <DA_PeristalticPump.h>
#include <DA_NonBlockingDelay.h>

#include "hydroponics.h"
#include "DA_HASelect.h"
// #include "DA_NutrientController.h"

#define DEFAULT_LIGHTS_ON_ALARM_TIME 1519992000       // AlarmHMS (5, 0, 0)
#define DEFAULT_LIGHTS_OFF_ALARM_TIME 1520056800      // AlarmHMS (23, 0, 0)
#define DEFAULT_HEATING_PAD_ON_TIME 1542034800        // AlarmHMS (8, 0, 0)
#define DEFAULT_HEATING_PAD_OFF_TIME 1542067200       // AlarmHMS (17, 0, 0)
#define DEFAULT_RESET_TIME 1519974000                 // AlarmHMS(0, 0, 0) //
                                                      // midnight
#define DEFAULT_CIRCULATION_PUMP_ON_DURATION 15 * 60  // 15 min * 60 s
#define DEFAULT_CIRCULATION_PUMP_OFF_DURATION 45 * 60 // 45 min * 60 s
#define DEFAULT_FAN_ON_DURATION 30 * 60               // 30 min * 60 s
#define DEFAULT_FAN_OFF_DURATION 30 * 60              // 30 min * 60 s

// Flow Max DS-01230-D2 12V 3.3 GPM Water Pump Duty cycle 5 minutes on 10 minutes off
#define DEFAULT_DRAIN_PUMP_ON_DURATION 5 * 60
#define DEFAULT_DRAIN_PUMP_OFF_DURATION 10 * 60

#define DEFAULT_HEARTBEAT_DURATION 2 // seconds

#define DEFAULT_PH_AUTO_VOLUME 5    // ml
#define DEFAULT_PH_AUTO_INTERVAL 60 // s
#define DEFAULT_PH_MANUAL_VOLUME 5  // ml

#define DEFAULT_N1N2_AUTO_VOLUME 10   // ml
#define DEFAULT_N1N2_AUTO_INTERVAL 60 // s
#define DEFAULT_N1N2_MANUAL_VOLUME 25 // ml
#define DEFAULT_PH_SETPOINT 60        // * 10
#define DEFAULT_EC_SETPOINT 1100      // uS/cm
#define DEFAULT_XIC_MODE 2            // off

#define STRLEN(s) (sizeof(s) / sizeof(s[0]))

#define EventHMS(_hr_, _min_, \
                 _sec_) (_hr_ * 60 * 60 * 60 + _min_ * 60 * 60 + _sec_)

// #define HOST_COMMAND_CHECK_INTERVAL  1000
#define ALARM_REFRESH_INTERVAL 10

const uint32_t DEFAULT_TIME = 1000188000;

#define EEPROM_CONFIGURED 2       // this value stored at address CONFIG_FLAG_ADDR
#define EEPROM_CONFIG_FLAG_ADDR 0 // is 0 if nothing was written
#define EEPROM_GROWING_CHAMBER_ON_TIME_ADDR EEPROM_CONFIG_FLAG_ADDR + sizeof(uint8_t)
#define EEPROM_GROWING_CHAMBER_OFF_TIME_ADDR EEPROM_GROWING_CHAMBER_ON_TIME_ADDR + sizeof(time_t)
#define EEPROM_SEEDING_AREA_ON_TIME_ADDR EEPROM_GROWING_CHAMBER_OFF_TIME_ADDR + sizeof(time_t)
#define EEPROM_SEEDING_AREA_OFF_TIME_ADDR EEPROM_SEEDING_AREA_ON_TIME_ADDR + sizeof(time_t)
#define EEPROM_FAN_ON_DURATION_ADDR EEPROM_SEEDING_AREA_OFF_TIME_ADDR + sizeof(time_t)
#define EEPROM_FAN_OFF_DURATION_ADDR EEPROM_FAN_ON_DURATION_ADDR + sizeof(time_t)
#define EEPROM_CIRCULATION_PUMP_ON_DURATION_ADDR EEPROM_FAN_OFF_DURATION_ADDR + sizeof(uint16_t)
#define EEPROM_CIRCULATION_PUMP_OFF_DURATION_ADDR \
  EEPROM_CIRCULATION_PUMP_ON_DURATION_ADDR + sizeof(uint16_t)
#define EEPROM_PH_SETPOINT_ADDR \
  EEPROM_CIRCULATION_PUMP_OFF_DURATION_ADDR + sizeof(uint16_t)
#define EEPROM_EC_SETPOINT_ADDR \
  EEPROM_PH_SETPOINT_ADDR + sizeof(uint16_t)
#define EEPROM_N1N2_MANUAL_VOL_ADDR \
  EEPROM_EC_SETPOINT_ADDR + sizeof(uint16_t)
#define EEPROM_N1N2_AUTO_VOL_ADDR \
  EEPROM_N1N2_MANUAL_VOL_ADDR + sizeof(uint16_t)
#define EEPROM_PH_AUTO_VOL_ADDR \
  EEPROM_N1N2_AUTO_VOL_ADDR + sizeof(uint16_t)
#define EEPROM_PH_AUTO_INTERVAL_ADDR \
  EEPROM_PH_AUTO_VOL_ADDR + sizeof(uint16_t)
#define EEPROM_PH_MANUAL_VOL_ADDR \
  EEPROM_PH_AUTO_INTERVAL_ADDR + sizeof(uint16_t)
#define EEPROM_N1N2_AUTO_INTERVAL_ADDR \
  EEPROM_PH_MANUAL_VOL_ADDR + sizeof(uint16_t)
#define EEPROM_N1N2_MODE_ADDR \
  EEPROM_N1N2_AUTO_INTERVAL_ADDR + sizeof(uint16_t)
#define EEPROM_PH_DOWN_HOA_ADDR \
  EEPROM_N1N2_MODE_ADDR + sizeof(uint16_t)

// Circ pump - EEPROM is baked offset skip bad address +12
#define EEPROM_HS_105HOA_ADDR \
  EEPROM_PH_DOWN_HOA_ADDR + 12 * sizeof(uint16_t)

// fan pump
#define EEPROM_HS_101HOA_ADDR \
  EEPROM_HS_105HOA_ADDR + sizeof(uint16_t)

// seeding
#define EEPROM_HS_102HOA_ADDR \
  EEPROM_HS_101HOA_ADDR + sizeof(uint16_t)

// GC
#define EEPROM_HS_103HOA_ADDR \
  EEPROM_HS_102HOA_ADDR + sizeof(uint16_t)

// heading pad
#define EEPROM_HS_104HOA_ADDR \
  EEPROM_HS_103HOA_ADDR + sizeof(uint16_t)
#define EEPROM_HEATING_PAD_ON_TIME_ADDR EEPROM_HS_104HOA_ADDR + \
                                            sizeof(uint16_t)
#define EEPROM_HEATING_PAD_OFF_TIME_ADDR EEPROM_HEATING_PAD_ON_TIME_ADDR + \
                                             sizeof(uint16_t)

#define EEPROM_DRAIN_PUMP_ON_DURATION_ADDR EEPROM_HEATING_PAD_OFF_TIME_ADDR + \
                                               sizeof(uint16_t)
#define EEPROM_DRAIN_PUMP_OFF_DURATION_ADDR \
  EEPROM_DRAIN_PUMP_ON_DURATION_ADDR + sizeof(uint16_t)

#define EEPROM_PH_HIGH_CAL_EPOCH_ADDR \
  EEPROM_DRAIN_PUMP_OFF_DURATION_ADDR + sizeof(time_t)

#define EEPROM_PH_MID_CAL_EPOCH_ADDR \
  EEPROM_PH_HIGH_CAL_EPOCH_ADDR + sizeof(time_t)

#define EEPROM_PH_LOW_CAL_EPOCH_ADDR \
  EEPROM_PH_MID_CAL_EPOCH_ADDR + sizeof(time_t)

#define EEPROM_EC_DRY_CAL_EPOCH_ADDR \
  EEPROM_PH_LOW_CAL_EPOCH_ADDR + sizeof(time_t)

#define EEPROM_EC_HIGH_CAL_EPOCH_ADDR \
  EEPROM_EC_DRY_CAL_EPOCH_ADDR + sizeof(time_t)

#define EEPROM_EC_LOW_CAL_EPOCH_ADDR \
  EEPROM_EC_HIGH_CAL_EPOCH_ADDR + sizeof(time_t)

Stream *debugOutputStream = NULL;

// #define PROCESS_MODBUS

// Home Assistant integration

HADevice device(defaultMAC, sizeof(defaultMAC));
EthernetClient ethernetClient;
HAMqtt mqttClient(ethernetClient, device, 32);

const char *ORGNAME = "Cogito Methods";
const char *APPVERSION = "2.0.0";
const char *MODEL = "Mega";
const char *APP_NAME = "Grow Room";
const char *MANUFACTURER = "Cogito";

HASensorNumber phSensor("ph", HASensorNumber::PrecisionP1);
HASensorNumber ecSensor("ec", HASensorNumber::PrecisionP1);
HASensorNumber nutrientTempSensor("nutrientTemp", HASensorNumber::PrecisionP1);

HABinarySensor fanStatus("fanStatus");
HABinarySensor circulationPumpStatus("circulationPumpStatus");
// HABinarySensor drainPumpStatus("drainPumpStatus");
HABinarySensor heatingPadStatus("heatingPadStatus");
HABinarySensor growingChamberLEDStatus("growingChamberLEDStatus");
HABinarySensor seedingAreaLEDStatus("seedingAreaLEDStatus");
HABinarySensor heartBeatStatus("heartBeatStatus");
HABinarySensor inletValveStatus("inletValveStatus");
HABinarySensor binLevelHigh("binLevelHigh");

DA_HASelect circulationPumpHOA("circulationPumpHOA", EEPROM_HS_105HOA_ADDR);
DA_HASelect seedingAreaLEDHOA("seedingAreaLEDHOA", EEPROM_HS_102HOA_ADDR);
DA_HASelect growingChamberLEDHOA("growingChamberLEDHOA", EEPROM_HS_103HOA_ADDR);
DA_HASelect heatingPadHOA("heatingPadHOA", EEPROM_HS_104HOA_ADDR);
DA_HASelect fanHOA("fanHOA", EEPROM_HS_101HOA_ADDR);

HASelect phCalSel("phCalSel");
HASelect ecCalSel("ecCalSel");

HANumber circulationPumpOnDuration("circulationPumpOnDuration",
                                   HASensorNumber::PrecisionP0); // seconds);

HANumber circulationPumpOffDuration("circulationPumpOffDuration",
                                    HASensorNumber::PrecisionP0); // seconds);

HANumber fanOnDuration("fanOnDuration",
                       HASensorNumber::PrecisionP0); // seconds);

HANumber fanOffDuration("fanOffDuration",
                        HASensorNumber::PrecisionP0); // seconds);

HASensorNumber circulationPumpOnDurationPV("circulationPumpOnDurationPV",
                                           HASensorNumber::PrecisionP0); // seconds);
HASensorNumber circulationPumpOffDurationPV("circulationPumpOffDurationPV",
                                            HASensorNumber::PrecisionP0); // seconds);
HASensorNumber fanOnDurationPV("fanOnDurationPV",
                               HASensorNumber::PrecisionP0); // seconds);
HASensorNumber fanOffDurationPV("fanOffDurationPV",
                                HASensorNumber::PrecisionP0); // seconds);

HANumber growingChamberLEDOnTime("growingChamberLEDOnTime",
                                 HASensorNumber::PrecisionP0); // seconds);
HANumber growingChamberLEDOffTime("growingChamberLEDOffTime",
                                  HASensorNumber::PrecisionP0); // seconds);

HANumber seedingAreaLEDOnTime("seedingAreaLEDOnTime",
                              HASensorNumber::PrecisionP0); // seconds);
HANumber seedingAreaLEDOffTime("seedingAreaLEDOffTime",
                               HASensorNumber::PrecisionP0); // seconds);

HANumber seedingAreaPadOnTime("seedingAreaPadOnTime",
                              HASensorNumber::PrecisionP0); // seconds);
HANumber seedingAreaPadOffTime("seedingAreaPadOffTime",
                               HASensorNumber::PrecisionP0); // seconds);
HANumber rtcTime("rtcTime", HASensorNumber::PrecisionP0);    // epoch time
#define HOA_HAND 0
#define HOA_OFF 1
#define HOA_AUTO 2

const char *hoaOptions = "Hand;Off;Auto";
const char *calOptions = "High;Mid-Dry;Low;Clear";
const char *customStateTopic = "aha/deadbe0001fc/batch/state";
const char *customCmdTopic = "aha/deadbe0001fc/calibrate/cmd_t";

void onHOACommand(int8_t, HASelect *);

void onCirculationPumpDurationCommand(const HANumeric value,
                                      HANumber *sender);

void onFanDurationCommand(const HANumeric value,
                          HANumber *sender);

void onGrowingChamberLEDTimeCommand(const HANumeric value,
                                    HANumber *sender);

void onSeedingAreaLEDTimeCommand(const HANumeric value,
                                 HANumber *sender);

void seedingAreaPadTimeCommand(const HANumeric value,
                               HANumber *sender);

void setBoardTime(const HANumeric number, HANumber *sender);

// refresh intervals
const uint32_t POLL_CYCLE_MS = 2 * 1000;                       // every 2 s
const uint32_t TEMPERATURE_COMPENSATE_CYCLE = 60 * 60 * 1000L; // every hour

// Ph and EC probes
//
#define PH_I2C_ADDRESS 100
#define EC_I2C_ADDRESS 101
DA_AtlasPH AT_001 = DA_AtlasPH(PH_DEFAULT_I2C_ADDRESS);
DA_AtlasEC AT_002 = DA_AtlasEC(EC_DEFAULT_I2C_ADDRESS);

/**
 *  forward declarations
 * .ino file converted to plantformIO main.c
 */
time_t alarmTimeToUTC(time_t localAlarmTime);
time_t eventTimeToLocal(time_t utcAlarmTime);
void do_ONP_SPoll();
void on_RemoteLocalToggle(bool state,
                          int aPin);

void doOnTemperatureCompensate();
void onIdle();
void EEPROMLoadConfig();
time_t EEPROMReadTimeEntry(uint16_t atAddress);
void EEPROMWriteTimeEntry(time_t epoch,
                          uint16_t atAddress);
void EEPROMWriteDefaultConfig();
void EEPROMWriteUint16(uint16_t duration,
                       uint16_t atAddress);

void initOneWire();
void initOneWireDevice(DeviceAddress aDevice,
                       uint8_t aIndex);
uint16_t isEEPROMConfigured();
uint16_t EEPROMReadUint16(uint16_t atAddress);

void on_Circulation_Pump_Process(int8_t state);
// void on_DrainPump_Process(bool state,
//                           int aPin);
void on_GrowingChamberLED_Process(int8_t state);
void on_HeatingPad_Process(int8_t state);
void on_Fan_Process(int8_t state);
void on_InletValve_Process(bool state,
                           int aPin);
void on_SeedingAreaLED_Process(int8_t state);

void onAtlasECSample(IO_TYPE type,
                     float value);
void onAtlasPhSample(IO_TYPE type,
                     float value);

void printOneWireAddress(Stream *debugOutputStream,
                         DeviceAddress aDeviceAddress,
                         bool aCR);

void refreshDiscreteInputs();
void refreshDiscreteOutputs();

void setupRTC();
#if defined(PROCESS_TERMINAL)
void dateTimeToBuffer(time_t aTime,
                      char *bufferMem);
void timeToBuffer(time_t aTime,
                  char *bufferMem);
#endif

void on_watchdogTimerEvent(bool state);

#ifdef ENABLE_AMBIENT_T_H
// DHT-22 - one wire type humidity sensor (won't work with one wire lib)
#define DHT_BUS_PIN 5
DHT AT_101 = DHT(DHT_BUS_PIN, DHT22);
float AT_101T = NAN;
float AT_101H = NAN;
float AT_101HI = NAN;
#endif
// atlas sensor
IO_TYPE currentIOType = i2c_ph;

// One Wire
//
//
DeviceAddress ambientTemperatureAddress =
    {
        0x28, 0x6F, 0xE3, 0xA0, 0x04, 0x00, 0x00, 0x5A};

float TT_001T = 0.0f;
DeviceAddress mixtureTemperatureAddress =
    {
        0x28, 0xFF, 0xF4, 0xF6, 0x84, 0x16, 0x05, 0x0C};

#define WIRE_BUS_PIN 6 // pin
#define ONE_TEMPERATURE_PRECISION 9
OneWire oneWire(WIRE_BUS_PIN);
DallasTemperature sensors(&oneWire);

// Analog Inputs
//


// from top,  0-> undefined Water Level present value in cm

// Discete Outputs
// DO 36 AC spares
// DO DC 39, 41,42,43 spares
//
// Seeding LED 120 VAC
DA_DiscreteOutput DY_102 = DA_DiscreteOutput(31, LOW);

// Heating Pad
DA_DiscreteOutput DY_104 = DA_DiscreteOutput(35, LOW);

// Growing Chamber LED
DA_DiscreteOutput DY_103 = DA_DiscreteOutput(32, LOW);

// Heartbeat LEDdebugOutputStream
DA_DiscreteOutputTmr HY_001 = DA_DiscreteOutputTmr(13,
                                                   LOW,
                                                   DEFAULT_HEARTBEAT_DURATION,
                                                   DEFAULT_HEARTBEAT_DURATION);

// Circulation Pump -> 120V AC
// DA_DiscreteOutputTmr PY_001 = DA_DiscreteOutputTmr(33,
//                                                    LOW,
//                                                    DEFAULT_CIRCULATION_PUMP_ON_DURATION,
//                                                    DEFAULT_CIRCULATION_PUMP_OFF_DURATION);

// Circulation/Drain Pump 12 VDC

DA_DiscreteOutputTmr PY_001 = DA_DiscreteOutputTmr(38,
                                                   LOW,
                                                   DEFAULT_CIRCULATION_PUMP_ON_DURATION,
                                                   DEFAULT_CIRCULATION_PUMP_OFF_DURATION);

// Fan -> 120V AC
DA_DiscreteOutputTmr FY_001 = DA_DiscreteOutputTmr(34,
                                                   LOW,
                                                   DEFAULT_FAN_ON_DURATION,
                                                   DEFAULT_FAN_OFF_DURATION);

// Inlet H20 Valve 12 VDC
DA_DiscreteOutput VY_001A = DA_DiscreteOutput(37, LOW);

// Drain Pump 12 VDC
// DA_DiscreteOutputTmr PY_002 = DA_DiscreteOutputTmr(38,
//                                                   LOW,
//                                                   DEFAULT_DRAIN_PUMP_ON_DURATION,
//                                                   DEFAULT_DRAIN_PUMP_OFF_DURATION);
// DA_DiscreteOutput PY_002 = DA_DiscreteOutput(38, LOW);

// Discrete Inputs
//
// Nutrient Mixture Tag Hi-Hi Level switch 0=Hi-Hi
DA_DiscreteInput LSHH_002 = DA_DiscreteInput(42,
                                             DA_DiscreteInput::ToggleDetect,
                                             true);

// Drain pump Hand Status: Start/Stop
// DA_DiscreteInput HS_001 = DA_DiscreteInput(43,
//                                            DA_DiscreteInput::ToggleDetect,
//                                            true);

// Smoke Detector
// DA_DiscreteInput SSH_101 = DA_DiscreteInput(9);

// Inlet H20 Open/Close
DA_DiscreteInput HS_002 = DA_DiscreteInput(49,
                                           DA_DiscreteInput::ToggleDetect,
                                           true);

TimeChangeRule usMDT =
    {
        "MDT", Second, dowSunday, Mar, 12, -360};

TimeChangeRule usMST =
    {
        "MST", First, dowSunday, Nov, 5, -420};

// Mountain time zone
Timezone usMT(usMDT, usMST);

struct _onOffControlEntry
{
  time_t onEpoch;  // utc epoch
  time_t offEpoch; // utc epoch

  void (*onEventOn)();
  void (*onEventOff)();
};

typedef _onOffControlEntry OnOffControlEntry;

OnOffControlEntry growingChamberLights;
OnOffControlEntry seedingAreaLights;
OnOffControlEntry heatingPad;

DA_NonBlockingDelay KI_001 =
    DA_NonBlockingDelay(POLL_CYCLE_MS, do_ONP_SPoll);
#ifdef ENABLE_FLOWMETER
DA_NonBlockingDelay KI_002 = DA_NonBlockingDelay(FLOW_CALC_PERIOD_MS,
                                                 doOnCalcFlowRate);
#endif
DA_NonBlockingDelay KI_003 = DA_NonBlockingDelay(TEMPERATURE_COMPENSATE_CYCLE,
                                                 doOnTemperatureCompensate);

DA_NonBlockingDelay idlTmr = DA_NonBlockingDelay(PUBLISH_INTERVAL,
                                                 onIdle);


DS3232RTC RTC;                                                 
void onMQTTMessage(const char *topic, const uint8_t *payload, uint16_t length)
{

  if (strcmp(topic, customCmdTopic) == 0)
  {
    LOG_TRACE("TOPIC", topic);

    char payloadStr[4]; // Small buffer for single-digit payload (e.g., "1")
    if (length >= sizeof(payloadStr))
    {
      LOG_TRACE("Payload too long");
      return;
    }
    memcpy(payloadStr, payload, length);
    payloadStr[length] = '\0';
    int value = atoi(payloadStr);

    if (value == 0)
    {
      LOG_TRACE("onMQTTMessage: ph_high command received");
    }
  }
}

void onMQTTConnected()
{
  mqttClient.subscribe(customCmdTopic);
}

void setup()
{

  wdt_disable();
  delay(3000);
  wdt_enable(WDTO_8S);

#ifdef DBG_LOG
  Serial.begin(115200);
  debugOutputStream = &Serial;
#endif

  Ethernet.begin(defaultMAC, defaultIP, defaultGateway, defaultSubnet);
  device.setName(APP_NAME);
  device.setSoftwareVersion(APPVERSION);
  device.setManufacturer(MANUFACTURER);
  device.setModel(MODEL);

  phSensor.setIcon("mdi:ph");
  phSensor.setName("pH");
  phSensor.setUnitOfMeasurement("_");

  ecSensor.setIcon("mdi:induction");
  ecSensor.setName("EC");
  ecSensor.setUnitOfMeasurement("uS/cm");

  nutrientTempSensor.setIcon("mdi:temperature-celsius");
  nutrientTempSensor.setName("Nutrient Temp");
  nutrientTempSensor.setUnitOfMeasurement("°C");

  fanStatus.setIcon("mdi:fan");
  fanStatus.setName("Fan Status");

  inletValveStatus.setIcon("mdi:valve-open");
  inletValveStatus.setName("Inlet Valve Status");

  circulationPumpStatus.setIcon("mdi:sprinkler-fire");
  circulationPumpStatus.setName("CP Status");
  // drainPumpStatus.setIcon("mdi:water-pump");
  // drainPumpStatus.setName("Drain Pump Status");

  heatingPadStatus.setIcon("mdi:radiator");
  heatingPadStatus.setName("Heating Pad Status");

  growingChamberLEDStatus.setIcon("mdi:led-variant-on");
  growingChamberLEDStatus.setName("GC LED Status");

  seedingAreaLEDStatus.setIcon("mdi:led-variant-on");
  seedingAreaLEDStatus.setName("SA LED Status");

  heartBeatStatus.setIcon("mdi:heart-pulse");
  heartBeatStatus.setName("Heart Beat");

  binLevelHigh.setIcon("mdi:waves-arrow-up");
  binLevelHigh.setName("Nutrient Bin Level High");

  circulationPumpHOA.setName("CP HOA");
  circulationPumpHOA.setIcon("mdi:chandelier");
  circulationPumpHOA.setOptions(hoaOptions);
  circulationPumpHOA.onCommand(&onHOACommand);

  seedingAreaLEDHOA.setName("SA LED HOA");
  seedingAreaLEDHOA.setIcon("mdi:chandelier");
  seedingAreaLEDHOA.setOptions(hoaOptions);
  seedingAreaLEDHOA.onCommand(&onHOACommand);

  growingChamberLEDHOA.setName("GC LED HOA");
  growingChamberLEDHOA.setIcon("mdi:chandelier");
  growingChamberLEDHOA.setOptions(hoaOptions);
  growingChamberLEDHOA.onCommand(&onHOACommand);

  heatingPadHOA.setName("HP HOA");
  heatingPadHOA.setIcon("mdi:chandelier");
  heatingPadHOA.setOptions(hoaOptions);
  heatingPadHOA.onCommand(&onHOACommand);

  fanHOA.setName("Fan HOA");
  fanHOA.setIcon("mdi:chandelier");
  fanHOA.setOptions(hoaOptions);
  fanHOA.onCommand(&onHOACommand);

  phCalSel.setName("pH Cal");
  phCalSel.setIcon("mdi:flask");
  phCalSel.setOptions(calOptions);
  phCalSel.onCommand(&onHOACommand);

  ecCalSel.setName("EC Calibration");
  ecCalSel.setIcon("mdi:flask");
  ecCalSel.setOptions(calOptions);
  ecCalSel.onCommand(&onHOACommand);

  circulationPumpOnDuration.setName("CP On SP");
  circulationPumpOnDuration.setIcon("mdi:timer-edit-outline");
  circulationPumpOnDuration.setUnitOfMeasurement("s");
  circulationPumpOnDuration.onCommand(&onCirculationPumpDurationCommand);
  circulationPumpOnDuration.setMax(10 * 60); // 10 minutes
  circulationPumpOnDuration.setMin(10);      // seconds

  circulationPumpOnDurationPV.setName("CP On PV");
  circulationPumpOnDurationPV.setIcon("mdi:timer-refresh-outline");
  circulationPumpOnDurationPV.setUnitOfMeasurement("s");

  circulationPumpOffDuration.setName("CP Off SP");
  circulationPumpOffDuration.setIcon("mdi:timer-edit-outline");
  circulationPumpOffDuration.setUnitOfMeasurement("s");
  circulationPumpOffDuration.onCommand(&onCirculationPumpDurationCommand);
  circulationPumpOffDuration.setMax(60 * 60); // 60 minutes
  circulationPumpOffDuration.setMin(10);      // seconds

  circulationPumpOffDurationPV.setName("CP Off PV");
  circulationPumpOffDurationPV.setIcon("mdi:timer-refresh-outline");
  circulationPumpOffDurationPV.setUnitOfMeasurement("s");

  fanOnDuration.setName("Fan On SP");
  fanOnDuration.setIcon("mdi:timer-edit-outline");
  fanOnDuration.setUnitOfMeasurement("s");
  fanOnDuration.onCommand(&onFanDurationCommand);
  fanOnDuration.setMax(60 * 60); // 1 hour
  fanOnDuration.setMin(10);      // seconds

  fanOnDurationPV.setName("Fan On PV");
  fanOnDurationPV.setIcon("mdi:timer-refresh-outline");
  fanOnDurationPV.setUnitOfMeasurement("s");

  fanOffDuration.setName("Fan Off SP");
  fanOffDuration.setIcon("mdi:timer-edit-outline");
  fanOffDuration.setUnitOfMeasurement("s");
  fanOffDuration.onCommand(&onFanDurationCommand);
  fanOffDuration.setMax(60 * 60); // 1 hour
  fanOffDuration.setMin(10);      // seconds

  fanOffDurationPV.setName("Fan Off PV");
  fanOffDurationPV.setIcon("mdi:timer-refresh-outline");
  fanOffDurationPV.setUnitOfMeasurement("s");

  growingChamberLEDOnTime.setName("GC LED On SP");
  growingChamberLEDOnTime.setIcon("mdi:clock-digital");
  growingChamberLEDOnTime.setUnitOfMeasurement("s");
  growingChamberLEDOnTime.onCommand(&onGrowingChamberLEDTimeCommand);
  growingChamberLEDOnTime.setMax(2147483647); // 2038
  growingChamberLEDOnTime.setMin(0);          // seconds

  growingChamberLEDOffTime.setName("GC LED Off SP");
  growingChamberLEDOffTime.setIcon("mdi:clock-digital");
  growingChamberLEDOffTime.setUnitOfMeasurement("s");
  growingChamberLEDOffTime.onCommand(&onGrowingChamberLEDTimeCommand);
  growingChamberLEDOffTime.setMax(2147483647); // 2038
  growingChamberLEDOffTime.setMin(0);          // seconds

  seedingAreaPadOnTime.setName("SA Pad On SP");
  seedingAreaPadOnTime.setIcon("mdi:clock-digital");
  seedingAreaPadOnTime.setUnitOfMeasurement("s");
  seedingAreaPadOnTime.onCommand(&seedingAreaPadTimeCommand);
  seedingAreaPadOnTime.setMax(2147483647); // 2038
  seedingAreaPadOnTime.setMin(0);          // seconds

  seedingAreaPadOffTime.setName("SA PAD Off SP");
  seedingAreaPadOffTime.setIcon("mdi:clock-digital");
  seedingAreaPadOffTime.setUnitOfMeasurement("s");
  seedingAreaPadOffTime.onCommand(&seedingAreaPadTimeCommand);
  seedingAreaPadOffTime.setMax(2147483647); // 2038
  seedingAreaPadOffTime.setMin(0);          // seconds

  seedingAreaLEDOnTime.setName("SA LED On SP");
  seedingAreaLEDOnTime.setIcon("mdi:clock-digital");
  seedingAreaLEDOnTime.setUnitOfMeasurement("s");
  seedingAreaLEDOnTime.onCommand(&onSeedingAreaLEDTimeCommand);
  seedingAreaLEDOnTime.setMax(2147483647); // 2038
  seedingAreaLEDOnTime.setMin(0);          // seconds

  seedingAreaLEDOffTime.setName("SA LED Off SP");
  seedingAreaLEDOffTime.setIcon("mdi:clock-digital");
  seedingAreaLEDOffTime.setUnitOfMeasurement("s");
  seedingAreaLEDOffTime.onCommand(&onSeedingAreaLEDTimeCommand);
  seedingAreaLEDOffTime.setMax(2147483647); // 2038
  seedingAreaLEDOffTime.setMin(0);          // seconds

  rtcTime.setName("RTC Time");
  rtcTime.setIcon("mdi:clock-outline");
  rtcTime.setUnitOfMeasurement("s");
  rtcTime.setMin(0);          // seconds
  rtcTime.setMax(2147483647); // 2038
  rtcTime.onCommand(&setBoardTime);

  randomSeed(analogRead(0));

  HS_002.setPollingInterval(500);   // ms
  LSHH_002.setDebounceTime(1000);   // float switch bouncing around
  LSHH_002.setPollingInterval(500); // ms
  HS_002.setOnEdgeEvent(&on_InletValve_Process);
  LSHH_002.setOnEdgeEvent(&on_InletValve_Process);
  // HS_001.setPollingInterval(200); // ms
  // HS_001.setDebounceTime( 110);
  // HS_001.setOnEdgeEvent(&on_DrainPump_Process);

  // 1-wire
  sensors.begin();
  initOneWire();

  setupRTC();

  AT_001.setOnPollCallBack(onAtlasPhSample);
  AT_001.setPollingInterval(3000);
  AT_001.retrieveCompensatedTemperature();

  AT_002.setOnPollCallBack(onAtlasECSample);
  AT_002.setPollingInterval(2000);
  AT_002.retrieveCompensatedTemperature();

// // XIC_001.serialize(debugOutputStream, true);
// XIC_002.serialize(debugOutputStream, true);
// XIC_003.serialize(debugOutputStream, true);
//// XIC_001.setMaxFlowRate(27); // pH pump runs slower than the other two for some
// reason
//// XIC_001.setAutoControlParameters(10, 120);

// humidity sensor
#ifdef ENABLE_AMBIENT_T_H
  AT_101.begin();
#endif

#ifdef ENABLE_FLOWMETER
  FT_002.setMeterFactor(7.5);
  FT_003.setMeterFactor(7.5);
  ENABLE_FT002_SENSOR_INTERRUPTS;
  ENABLE_FT003_SENSOR_INTERRUPTS;
#ifdef ENABLE_CO2
  ENABLE_CO2_SENSOR_RISING_INTERRUPTS;
#endif
#endif
  if (isEEPROMConfigured() == EEPROM_CONFIGURED)
  {

    EEPROMLoadConfig();
  }
  else
  {
    EEPROMWriteDefaultConfig();
    EEPROMLoadConfig();
  }

  fanHOA.load();
  heatingPadHOA.load();
  growingChamberLEDHOA.load();
  seedingAreaLEDHOA.load();
  circulationPumpHOA.load();

  on_GrowingChamberLED_Process(growingChamberLEDHOA.getCurrentState());
  on_SeedingAreaLED_Process(seedingAreaLEDHOA.getCurrentState());
  on_HeatingPad_Process(heatingPadHOA.getCurrentState());

  FY_001.start(DA_DiscreteOutputTmr::Continuous);
  on_Fan_Process(fanHOA.getCurrentState());

  PY_001.start(DA_DiscreteOutputTmr::Continuous);
  on_Circulation_Pump_Process(circulationPumpHOA.getCurrentState());

  // PY_002.start(DA_DiscreteOutputTmr::Continuous);
  HY_001.setOnTimeEvent(&on_watchdogTimerEvent);
  HY_001.start(DA_DiscreteOutputTmr::Continuous);

  mqttClient.onConnected(&onMQTTConnected);
  mqttClient.onMessage(onMQTTMessage);
  // mqttClient.setBufferSize(512);
  mqttClient.begin(defaultMQTT);

  delay(1000);
}

void sendCustomMQTTMessage(const char *aTopic,
                           const char *aMessage)
{
  if (mqttClient.getState() == MQTT_CONNECTED)
  {

    DynamicJsonDocument doc(256);
    doc["phCalHigh"] = EEPROMReadTimeEntry(EEPROM_PH_HIGH_CAL_EPOCH_ADDR);
    doc["phCalMid"] = EEPROMReadTimeEntry(EEPROM_PH_MID_CAL_EPOCH_ADDR);
    doc["phCalLow"] = EEPROMReadTimeEntry(EEPROM_PH_LOW_CAL_EPOCH_ADDR);
    doc["ecCalDry"] = EEPROMReadTimeEntry(EEPROM_EC_DRY_CAL_EPOCH_ADDR);
    doc["ecCalHigh"] = EEPROMReadTimeEntry(EEPROM_EC_HIGH_CAL_EPOCH_ADDR);
    doc["ecCalLow"] = EEPROMReadTimeEntry(EEPROM_EC_LOW_CAL_EPOCH_ADDR);
    doc["RTCNow"] = now();
    doc["RCTStatus"] = timeStatus();
    // String payload;
    char payload[192];
    serializeJson(doc, payload, sizeof(payload));

    serializeJson(doc, payload);
    // LOG_TRACE("custom mqtt message", payload);
    mqttClient.publish(aTopic, payload, true);
  }
}

void onIdle()
{

#if defined(SIM_SENSOR_DATA)
  ecSensor.setValue(random(1000) / 10.0f);
  nutrientTempSensor.setValue(random(200) / 10.0f);
  phSensor.setValue(random(100) / 10.0f);
#else
  phSensor.setValue(AT_001.getSample());
  ecSensor.setValue(AT_002.getSample());
  nutrientTempSensor.setValue(TT_001T);
#endif

  circulationPumpStatus.setState(PY_001.isActive());

  fanStatus.setState(FY_001.isActive());
  inletValveStatus.setState(VY_001A.isActive());
  // drainPumpStatus.setState(HS_001.getSample() == HIGH);

  heatingPadStatus.setState(DY_104.isActive());

  growingChamberLEDStatus.setState(DY_103.isActive());

  seedingAreaLEDStatus.setState(DY_102.isActive());

  heartBeatStatus.setState(HY_001.isActive());

  binLevelHigh.setState(LSHH_002.getSample() == LOW);

  circulationPumpOnDuration.setState(HANumeric((uint16_t)(PY_001.getActiveDuration() / 1000), HASensorNumber::PrecisionP0));
  circulationPumpOnDurationPV.setValue((uint16_t)(PY_001.getCurrentActiveDuration() / 1000));

  circulationPumpOffDuration.setState(HANumeric((uint16_t)(PY_001.getInActiveDuration() / 1000), HASensorNumber::PrecisionP0));
  circulationPumpOffDurationPV.setValue((uint16_t)(PY_001.getCurrentInactiveDuration() / 1000));

  fanOnDuration.setState(HANumeric((uint16_t)(FY_001.getActiveDuration() / 1000), HASensorNumber::PrecisionP0));
  fanOnDurationPV.setValue((uint16_t)(FY_001.getCurrentActiveDuration() / 1000));

  fanOffDuration.setState(HANumeric((uint16_t)(FY_001.getInActiveDuration() / 1000), HASensorNumber::PrecisionP0));
  fanOffDurationPV.setValue((uint16_t)(FY_001.getCurrentInactiveDuration() / 1000));

  growingChamberLEDOnTime.setState(HANumeric((uint32_t)(growingChamberLights.onEpoch), HASensorNumber::PrecisionP0));
  growingChamberLEDOffTime.setState(HANumeric((uint32_t)(growingChamberLights.offEpoch), HASensorNumber::PrecisionP0));

  seedingAreaLEDOnTime.setState(HANumeric((uint32_t)(seedingAreaLights.onEpoch), HASensorNumber::PrecisionP0));
  seedingAreaLEDOffTime.setState(HANumeric((uint32_t)(seedingAreaLights.offEpoch), HASensorNumber::PrecisionP0));

  seedingAreaPadOnTime.setState(HANumeric((uint32_t)(heatingPad.onEpoch), HASensorNumber::PrecisionP0));
  seedingAreaPadOffTime.setState(HANumeric((uint32_t)(heatingPad.offEpoch), HASensorNumber::PrecisionP0));

  // LOG_TRACE("seedingAreaPadOnTime", heatingPad.onEpoch);
  // LOG_TRACE("seedingAreaPadOffTime", heatingPad.offEpoch);

  // LOG_TRACE("fan off dur", FY_001.getInActiveDuration() / 1000);
  // LOG_TRACE("off dur pv",FY_001.getCurrentInactiveDuration() / 1000);
  // LOG_TRACE("on dur",FY_001.getActiveDuration() / 1000);

  // LOG_TRACE("on dur pv",FY_001.getCurrentActiveDuration() / 1000);

  // mqttSendDiscreteIO(mqttStatusSendTopic);
  // mqttSendSetpointPV(mqttStatusSendTopic);
  // mqttSendHOA(mqttStatusSendTopic);

  // mqttSendDiscreteIO(mqttEUSendTopic);
  // mqttSendEU(mqttEUSendTopic);

  sendCustomMQTTMessage(customStateTopic, NULL);
}

void loop()
{

  mqttClient.loop();
  Ethernet.maintain();

  refreshDiscreteInputs();
  refreshDiscreteOutputs();

  if (currentIOType == i2c_ph)
  {
    AT_001.refresh();
  }
  else
  {
    AT_002.refresh();
  }
  idlTmr.refresh();
  KI_001.refresh();
#ifdef ENABLE_FLOWMETER
  KI_002.refresh();
#endif
  KI_003.refresh();

  //  // XIC_001.refresh();
}

/*
   Only open inlet H20 Valve iff no Hi-Hi water level
   note LSHH is high on high level (fail safe )
 */
void on_InletValve_Process(bool state, int aPin)
{

  if ((HS_002.getSample() == LOW) &&
      (LSHH_002.getSample() == HIGH))
    VY_001A.activate();
  else
    VY_001A.reset();
}

void on_watchdogTimerEvent(bool state)
{
  wdt_reset();
}

// void on_DrainPump_Process(bool state, int aPin)
// {
// #if defined(PROCESS_TERMINAL_VERBOSE)
//   *debugOutputStream << "on_DrainPump_Process HS_001" << endl;
//   HS_001.serialize(debugOutputStream, true);
// #endif // ifdef PROCESS_TERMINAL_VERBOSE

// if (HS_001.getSample() == LOW)
// {

//   PY_002.forceActive();
// }
// else
// {

//   PY_002.disable();
// }
// }

void on_Circulation_Pump_Process(int8_t state)
{
  // LOG_TRACE("on_Circulation_Pump_Process", state);

  switch (state)
  {
  case HOA_HAND:
    // LOG_TRACE("CircPmpHOA:Hand");
    circulationPumpHOA.setState(HOA_HAND, true);
    PY_001.pauseTimer();
    PY_001.disable();
    PY_001.forceActive(); // force the pump on

    break;

  case HOA_OFF:
    // LOG_TRACE("CircPmpHOA:Off");
    circulationPumpHOA.setState(HOA_OFF, true);
    PY_001.pauseTimer();
    PY_001.disable();

    break;

  case HOA_AUTO:
    // LOG_TRACE("CircPmpHOA:Auto");
    circulationPumpHOA.setState(HOA_AUTO, true);
    PY_001.enable();
    PY_001.resumeTimer();

    // PY_001.restart();
    // PY_001.resume();
    break;

  default:
    // LOG_TRACE("CircPmpHOA:Unknown");
    break;
  }
}

void on_HeatingPad_Process(int8_t state)
{

  // LOG_TRACE("on_HeatingPad_Process", state);
  switch (state)
  {
  case HOA_HAND:
    DY_104.disable();
    DY_104.forceActive(); // force the Flowing on
    heatingPadHOA.setState(HOA_HAND, true);
    break;

  case HOA_OFF:
    DY_104.disable();
    heatingPadHOA.setState(HOA_OFF, true);

    break;

  case HOA_AUTO:
    DY_104.enable();
    heatingPadHOA.setState(HOA_AUTO, true);
    break;

  default:
    break;
  }
}

void on_Fan_Process(int8_t state)
{

  // LOG_TRACE("on_Fan_Process", state);
  switch (state)
  {
  case HOA_HAND:

    fanHOA.setState(HOA_HAND, true);
    FY_001.pauseTimer();
    FY_001.disable();
    FY_001.forceActive(); // force the fan on

    break;

  case HOA_OFF:

    fanHOA.setState(HOA_OFF, true);
    FY_001.pauseTimer();
    FY_001.disable();

    break;

  case HOA_AUTO:

    fanHOA.setState(HOA_AUTO, true);
    FY_001.enable();
    FY_001.resumeTimer();

    // PY_001.restart();
    // PY_001.resume();
    break;

  default:
    LOG_TRACE("FanHOA:Unknown");
    break;
  }
}

void on_GrowingChamberLED_Process(int8_t state)
{
  // LOG_TRACE("on_GrowingChamberLED_Process", state);

  switch (state)
  {
  case HOA_HAND:
    growingChamberLEDHOA.setState(HOA_HAND, true);
    DY_103.disable();
    DY_103.forceActive(); // force the Flowing on
    break;

  case HOA_OFF:
    growingChamberLEDHOA.setState(HOA_OFF, true);
    DY_103.disable();
    break;

  case HOA_AUTO:
    growingChamberLEDHOA.setState(HOA_AUTO, true);
    DY_103.enable();
    break;

  default:
    break;
  }
}

void on_SeedingAreaLED_Process(int8_t state)
{

  // LOG_TRACE("on_SeedingAreaLED_Process", state);

  switch (state)
  {
  case HOA_HAND:
    seedingAreaLEDHOA.setState(HOA_HAND, true);
    DY_102.disable();
    DY_102.forceActive(); // force the Flowing on
    break;

  case HOA_OFF:
    seedingAreaLEDHOA.setState(HOA_OFF, true);
    DY_102.disable();
    break;

  case HOA_AUTO:
    seedingAreaLEDHOA.setState(HOA_AUTO, true);
    DY_102.enable();
    break;

  default:
    break;
  }
}

void setupRTC()
{
  RTC.begin();
  setSyncProvider([]()
                  { return RTC.get(); });

  // setSyncInterval(30);
  if (timeStatus() != timeSet)
  {
    LOG_ERROR("Unable to sync with the RTC");

    RTC.set(DEFAULT_TIME); // set the RTC and the system time to the received
                           // value
    setTime(DEFAULT_TIME); // Sync Arduino clock to the time received on the
                           // Serial2 port
  }
}

#if defined(PROCESS_TERMINAL)
void printOneWireAddress(Stream *debugOutputStream,
                         DeviceAddress aDeviceAddress,
                         bool aCR)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    // zero pad the address if necessary
    if (aDeviceAddress[i] < 16)
      *debugOutputStream << '0';
    debugOutputStream->print(aDeviceAddress[i], HEX);
  }

  if (aCR)
    *debugOutputStream << endl;
}

#endif // ifdef PROCESS_TERMINAL

void initOneWireDevice(DeviceAddress aDevice, uint8_t aIndex)
{
  if (!sensors.getAddress(aDevice, aIndex))
  {
    // TODO Alarm

#if defined(PROCESS_TERMINAL)
    *debugOutputStream << "Unable to find address for Device at index " << aIndex << "address:";
    printOneWireAddress(debugOutputStream, aDevice, true);
#endif // ifdef PROCESS_TERMINAL

    sensors.setResolution(aDevice, ONE_TEMPERATURE_PRECISION);
  }
}

void initOneWire()
{
  initOneWireDevice(ambientTemperatureAddress, 0);
  initOneWireDevice(mixtureTemperatureAddress, 1);
}

void onAtlasPhSample(IO_TYPE type, float value)
{
  // AT_001.serialize( debugOutputStream, true);
  currentIOType = i2c_ec;
}

void onAtlasECSample(IO_TYPE type, float value)
{
  // AT_002.serialize( debugOutputStream, true);
  currentIOType = i2c_ph;
}

#if defined(PROCESS_TERMINAL)
void timeToBuffer(time_t aTime, char *bufferMem)
{
  sprintf(bufferMem, "%02d:%02d:%02d", hour(aTime), minute(aTime), second(aTime));
}

void dateTimeToBuffer(time_t aTime, char *bufferMem)
{
  sprintf(bufferMem,
          "%02d/%02d/%04d %02d:%02d",
          month(aTime),
          day(aTime),
          year(aTime),
          hour(aTime),
          minute(aTime));
}
#endif
void refreshDiscreteInputs()
{
  LSHH_002.refresh();
  HS_002.refresh();
  // HS_001.refresh();
  // HS_105AB.refresh();
  // HS_101AB.refresh();
  // HS_102AB.refresh();
  // HS_103AB.refresh();
  // HS_104AB.refresh();
}

void refreshDiscreteOutputs()
{
  PY_001.refresh(); // on/off timer
  FY_001.refresh(); // on/off timer
  HY_001.refresh(); // on/off timer
}

bool isTimeForLightsOn(time_t currentEpoch, time_t offEpoch, time_t onEpoch)
{
  int curTimeInMins = hour(currentEpoch) * 60 + minute(currentEpoch);
  int offTimeInMins = hour(offEpoch) * 60 + minute(offEpoch);
  int onTimeInMins = hour(onEpoch) * 60 + minute(onEpoch);
  bool isOnEvent = false;

  if (offTimeInMins < onTimeInMins)
  {
    if (((curTimeInMins >= onTimeInMins) && (curTimeInMins >= offTimeInMins)) ||
        ((curTimeInMins <= onTimeInMins) &&
         (curTimeInMins <= offTimeInMins)))
      isOnEvent = true;
  }
  else
  {
    if ((curTimeInMins >= onTimeInMins) &&
        (curTimeInMins <= offTimeInMins))
      isOnEvent = true;
  }

  return isOnEvent;
}

void doLightControl(struct _onOffControlEntry *aEventControlEntry)
{
  if (isTimeForLightsOn(eventTimeToLocal(now()),
                        eventTimeToLocal(aEventControlEntry->offEpoch),
                        eventTimeToLocal(aEventControlEntry->onEpoch)))
    aEventControlEntry->onEventOn();
  else
    aEventControlEntry->onEventOff();
}

// alternate which sensor to compensate
//
bool altenateProbe = false;
void doOnTemperatureCompensate()
{
  if (altenateProbe)
  {
    currentIOType = i2c_ec;
    AT_002.setCompensatedTemperature(TT_001T);
  }
  else
  {
    currentIOType = i2c_ph;
    AT_001.setCompensatedTemperature(TT_001T);
  }
  altenateProbe = !altenateProbe;
}

// update sonar and 1-wire DHT-22 readings
void do_ONP_SPoll()
{
#if defined(NO_PING)
  float tLevel;
#endif // if defined(NO_PING)

  // *debugOutputStream << "CO2" << AT_102Raw << endl;

  if (growingChamberLEDHOA.getCurrentState() == HOA_AUTO)
    doLightControl(
        &growingChamberLights);

  if (seedingAreaLEDHOA.getCurrentState() == HOA_AUTO)
    doLightControl(
        &seedingAreaLights);

  if (heatingPadHOA.getCurrentState() == HOA_AUTO)
    doLightControl(&heatingPad);
#if defined(NO_PING)
  uint16_t distanceCM = LT_002.ping() / US_ROUNDTRIP_CM -
                        NUTRIENT_TANK_AIR_GAP;

  // compute distanace from high level mark
  tLevel = (NUTRIENT_TANK_MIXTURE_MAX - distanceCM) /
           NUTRIENT_TANK_MIXTURE_MAX; // NUTRIENT_TANK_MIXTURE_MAX;
  tLevel *= 100.0;
  LT_002Raw = tLevel;
  LT_002MedianFilter.add(LT_002Raw);
#endif // if defined(NO_PING)
  sensors.requestTemperatures();

  TT_001T = sensors.getTempC(mixtureTemperatureAddress);
  LOG_TRACE("Mixture Temp C:", TT_001T);
#ifdef ENABLE_AMBIENT_T_H
  AT_101H = AT_101.readHumidity(); // allow 1/4 sec to read
  AT_101T = AT_101.readTemperature();

  if (isnan(AT_101H) || isnan(AT_101T))
  {
#if defined(PROCESS_TERMINAL)

// *debugOutputStream << "Error reading from DHT-22 sensor." << endl;
#endif // ifdef PROCESS_TERMINAL
  }
  else
  {
    AT_101HI = AT_101.computeHeatIndex(AT_101T, AT_101H, false);
  }
#endif
  // ENABLE_CO2_SENSOR_RISING_INTERRUPTS;
}

void doReadInputs()
{
  // TE_001.refresh();
  // QE_001.refresh();
}

void doUpdateOutputs()
{
}

void doGrowingChamberLightsOn()
{
  DY_103.activate(); // if disabled, it won't activate
  // DY_103.activate(); // if disabled, it won't activate
}

void doGrowingChamberLightsOff()
{
  DY_103.reset();

  // DY_103.reset();
}

void doSeedingAreaLightsOn()
{
  DY_102.activate(); // if disabled, it won't activate
  // DY_103.activate(); // if disabled, it won't activate
}

void doSeedingAreaLightsOff()
{
  DY_102.reset();

  // DY_103.reset();
}

void doHeatingPadOn()
{
  DY_104.activate(); // if disabled, it won't activate
  // DY_103.activate(); // if disabled, it won't activate
}

void doHeatingPadOff()
{
  DY_104.reset();

  // DY_103.reset();
}

#if defined(PROCESS_TERMINAL)
void traceAlarmTime(char *label, time_t utcTime)
{
  char sprintfBuf[10];

  time_t localAlarmTime = eventTimeToLocal(utcTime);

  sprintf(sprintfBuf, "%02d:%02d", hour(localAlarmTime), minute(localAlarmTime));
  *debugOutputStream << label << "=" << sprintfBuf << (isAM(localAlarmTime) == true ? "AM" : "PM") << endl;
}

#endif // ifdef PROCESS_TERMINAL
// No-op
time_t alarmTimeToUTC(time_t localAlarmTime)
{
  // time_t utcAlarmTime = usMT.toUTC(localAlarmTime);

  /*
     if (usMT.utcIsDST(now()))
     {
     utcAlarmTime -= 60 * 60;
     }
     if (utcAlarmTime > SECS_PER_DAY)
     utcAlarmTime -= SECS_PER_DAY;
   */

  // *debugOutputStream << "alarmTimeToUTC:localAlarmTime="  <<  localAlarmTime
  // << "
  // UTC=" << utcAlarmTime << endl;
  return localAlarmTime;
}

time_t eventTimeToLocal(time_t utcAlarmTime)
{
  TimeChangeRule *tcr; // pointer to the time change rule, use to get the TZ
                       // abbrev
  time_t localAlarmTime = usMT.toLocal(utcAlarmTime, &tcr);

  /*
     int offset = tcr -> offset; // in minutes
     //if (usMT.utcIsDST(now()))
     // {
     offset += 60;
     localAlarmTime += 60 * 60;
     //}


     if ((hour(utcAlarmTime) + (int) offset / 60) < 0)
     {
     int hr = 12 + (hour(utcAlarmTime) - (int) offset / 60);

     localAlarmTime = AlarmHMS(hr, minute(utcAlarmTime), second(utcAlarmTime));
     }
   */
  return localAlarmTime;
}

void setCirculationPumpOnDuration(uint32_t aDuration)
{

  LOG_TRACE("setCirculationPumpOnDuration:", aDuration);

  PY_001.setActiveDuration((uint16_t)aDuration);

  EEPROMWriteUint16((uint16_t)aDuration,
                    EEPROM_CIRCULATION_PUMP_ON_DURATION_ADDR);
}

void setCirculationPumpOffDuration(uint32_t aDuration)
{
  LOG_TRACE("setCirculationPumpOffDuration:", aDuration);

  PY_001.setInactiveDuration((uint16_t)aDuration);

  EEPROMWriteUint16((uint16_t)aDuration,
                    EEPROM_CIRCULATION_PUMP_OFF_DURATION_ADDR);
}

void setFanOnDuration(uint32_t aDuration)
{
  LOG_TRACE("setFanOnDuration:", aDuration);

  FY_001.setActiveDuration((uint16_t)aDuration);

  EEPROMWriteUint16((uint16_t)aDuration,
                    EEPROM_FAN_ON_DURATION_ADDR);
}

void setFanOffDuration(uint32_t aDuration)
{
  LOG_TRACE("setFanOffDuration:", aDuration);

  FY_001.setInactiveDuration((uint16_t)aDuration);
  EEPROMWriteUint16((uint16_t)aDuration,
                    EEPROM_FAN_OFF_DURATION_ADDR);
}

void setHeatingPadOnTime(uint32_t anEpoch)
{
  LOG_TRACE("setHeatingPadOnTime:", anEpoch);
  heatingPad.onEpoch = alarmTimeToUTC(anEpoch);

  EEPROMWriteTimeEntry(anEpoch,
                       EEPROM_HEATING_PAD_ON_TIME_ADDR);
}

void setHeatingPadOffTime(uint32_t anEpoch)
{
  LOG_TRACE("setHeatingPadOffTime:", anEpoch);
  heatingPad.offEpoch = alarmTimeToUTC(anEpoch);

  EEPROMWriteTimeEntry(anEpoch,
                       EEPROM_HEATING_PAD_OFF_TIME_ADDR);
}

void setGrowingChamberLightsOnTime(uint32_t anEpoch)
{
  // LOG_TRACE("setGrowingChamberLightsOnTime:", anEpoch);

  growingChamberLights.onEpoch = alarmTimeToUTC(anEpoch);
  EEPROMWriteTimeEntry(growingChamberLights.onEpoch,
                       EEPROM_GROWING_CHAMBER_ON_TIME_ADDR);
}

void setGrowingChamberLightsOffTime(uint32_t anEpoch)
{
  // LOG_TRACE("setGrowingChamberLightsOffTime:", anEpoch);

  growingChamberLights.offEpoch = alarmTimeToUTC(anEpoch);
  EEPROMWriteTimeEntry(growingChamberLights.offEpoch,
                       EEPROM_GROWING_CHAMBER_OFF_TIME_ADDR);
}

void setSeedingAreaLightsOnTime(uint32_t anEpoch)
{
  // LOG_TRACE("setSeedingAreaLightsOnTime:", anEpoch);

  seedingAreaLights.onEpoch = alarmTimeToUTC(anEpoch);
  EEPROMWriteTimeEntry(seedingAreaLights.onEpoch,
                       EEPROM_SEEDING_AREA_ON_TIME_ADDR);
}

void setSeedingAreaLightsOffTime(uint32_t anEpoch)
{
  // LOG_TRACE("setSeedingAreaLightsOffTime:", anEpoch);

  seedingAreaLights.offEpoch = alarmTimeToUTC(anEpoch);
  EEPROMWriteTimeEntry(seedingAreaLights.offEpoch,
                       EEPROM_SEEDING_AREA_OFF_TIME_ADDR);
}

void setBoardTime(HANumeric number, HANumber *sender)
{
  LOG_TRACE("setBoardTime:", number.toUInt32());
  time_t anEpoch = (time_t)number.toUInt32();

  RTC.set(anEpoch); // set the RTC and the system time to the received value
  setTime(anEpoch); // Sync Arduino clock to the time received on the Serial2
}

void updateCalibration(int status, uint16_t eepromAddr)
{
  time_t timestamp = 0;
  if (status)
  {
    timestamp = now();
    EEPROMWriteTimeEntry(timestamp, eepromAddr);
  }
}

void doPHCalibration(int8_t mode)
{
  LOG_TRACE("doPHCalibration mode", mode);
  switch (mode)
  {
  case CALIBRATE_HIGH:
    updateCalibration(AT_001.calibrateHigh(), EEPROM_PH_HIGH_CAL_EPOCH_ADDR);
    // phCalSel.setState(0, true);
    break;
  case CALIBRATE_MID:
    updateCalibration(AT_001.calibrateMid(), EEPROM_PH_MID_CAL_EPOCH_ADDR);
    // phCalSel.setState(1, true);
    break;
  case CALIBRATE_LOW:
    updateCalibration(AT_001.calibrateLow(), EEPROM_PH_LOW_CAL_EPOCH_ADDR);
    // phCalSel.setState(2, true);
    break;
  case CALIBRATE_CLEAR:
    AT_001.calibrateClear();
    EEPROMWriteTimeEntry(0, EEPROM_PH_HIGH_CAL_EPOCH_ADDR);
    EEPROMWriteTimeEntry(0, EEPROM_PH_MID_CAL_EPOCH_ADDR);
    EEPROMWriteTimeEntry(0, EEPROM_PH_LOW_CAL_EPOCH_ADDR);
    // phCalSel.setState(3, true);

    break;
  case CALIBRATE_STATUS:

    // mqttSendCalibration(mqttStatusSendTopic);

    break;
  default:
    break;
  }
}

void doECCalibration(int8_t mode)
{
  LOG_TRACE("doECCalibration mode", mode);
  switch (mode)
  {
  case CALIBRATE_HIGH:
    updateCalibration(AT_002.calibrateHigh(), EEPROM_EC_HIGH_CAL_EPOCH_ADDR);
    // ecCalSel.setState(0, true);
    break;

  case CALIBRATE_LOW:
    updateCalibration(AT_002.calibrateLow(), EEPROM_EC_LOW_CAL_EPOCH_ADDR);
    // ecCalSel.setState(1, true);
    break;
  case CALIBRATE_CLEAR:
    AT_002.calibrateClear();
    EEPROMWriteTimeEntry(0, EEPROM_EC_DRY_CAL_EPOCH_ADDR);
    EEPROMWriteTimeEntry(0, EEPROM_EC_HIGH_CAL_EPOCH_ADDR);
    EEPROMWriteTimeEntry(0, EEPROM_EC_LOW_CAL_EPOCH_ADDR);
    // ecCalSel.setState(3, true);
    break;
  case CALIBRATE_DRY:

    updateCalibration(AT_002.calibrateDry(), EEPROM_EC_DRY_CAL_EPOCH_ADDR);
    // ecCalSel.setState(2, true);
    break;
  case CALIBRATE_STATUS:

    // mqttSendCalibration(mqttStatusSendTopic);

    break;
  default:
    break;
  }
}

// void mqttSendCalibration(const char *topic)
// {

//   char txBuff[MQTT_TX_SZ + 60];

//   sprintf(txBuff,
//           "{\"AT-001HTSF\": %ld, \"AT-001MTSF\":%ld, \"AT-001LTSF\":%ld, \"AT-002HTSF\":%ld,\"AT-002LTSF\": %ld, \"AT-002DTSF\":%ld, \"AT-001STF\":%d, \"AT-002STF\":%d}",
//           EEPROMReadTimeEntry(EEPROM_PH_HIGH_CAL_EPOCH_ADDR), EEPROMReadTimeEntry(EEPROM_PH_MID_CAL_EPOCH_ADDR), EEPROMReadTimeEntry(EEPROM_PH_LOW_CAL_EPOCH_ADDR),
//           EEPROMReadTimeEntry(EEPROM_EC_HIGH_CAL_EPOCH_ADDR), EEPROMReadTimeEntry(EEPROM_EC_LOW_CAL_EPOCH_ADDR), EEPROMReadTimeEntry(EEPROM_EC_DRY_CAL_EPOCH_ADDR),
//           AT_001.getProbeCommandStatus(), AT_002.getProbeCommandStatus());

// #ifdef PROCESS_TERMINAL
//   *debugOutputStream << "mqttSendCalibration buffer:" << txBuff << endl;
// #endif
//   mqttClient.publish(topic, txBuff);
// }

/*
** EEPROM functions
*/
void EEPROMWriteDefaultConfig()
{
  uint8_t configFlag = EEPROM_CONFIGURED;

  eeprom_write_block((const void *)&configFlag,
                     (void *)EEPROM_CONFIG_FLAG_ADDR,
                     sizeof(configFlag));

  time_t epoch = alarmTimeToUTC(DEFAULT_LIGHTS_ON_ALARM_TIME);
  EEPROMWriteTimeEntry(epoch, EEPROM_GROWING_CHAMBER_ON_TIME_ADDR);
  EEPROMWriteTimeEntry(epoch, EEPROM_SEEDING_AREA_ON_TIME_ADDR);

  epoch = alarmTimeToUTC(DEFAULT_LIGHTS_OFF_ALARM_TIME);
  EEPROMWriteTimeEntry(epoch, EEPROM_GROWING_CHAMBER_OFF_TIME_ADDR);
  EEPROMWriteTimeEntry(epoch, EEPROM_SEEDING_AREA_OFF_TIME_ADDR);

  epoch = alarmTimeToUTC(DEFAULT_HEATING_PAD_ON_TIME);
  EEPROMWriteTimeEntry(epoch, EEPROM_HEATING_PAD_ON_TIME_ADDR);

  epoch = alarmTimeToUTC(DEFAULT_HEATING_PAD_OFF_TIME);
  EEPROMWriteTimeEntry(epoch, EEPROM_HEATING_PAD_OFF_TIME_ADDR);

  EEPROMWriteUint16(DEFAULT_FAN_ON_DURATION,
                    EEPROM_FAN_ON_DURATION_ADDR);
  EEPROMWriteUint16(DEFAULT_FAN_OFF_DURATION,
                    EEPROM_FAN_OFF_DURATION_ADDR);

  EEPROMWriteUint16(DEFAULT_CIRCULATION_PUMP_ON_DURATION,
                    EEPROM_CIRCULATION_PUMP_ON_DURATION_ADDR);
  EEPROMWriteUint16(DEFAULT_CIRCULATION_PUMP_OFF_DURATION,
                    EEPROM_CIRCULATION_PUMP_OFF_DURATION_ADDR);

  EEPROMWriteUint16(DEFAULT_CIRCULATION_PUMP_OFF_DURATION,
                    EEPROM_CIRCULATION_PUMP_OFF_DURATION_ADDR);

  EEPROMWriteUint16(DEFAULT_PH_AUTO_VOLUME, EEPROM_PH_AUTO_VOL_ADDR);
  EEPROMWriteUint16(DEFAULT_PH_AUTO_INTERVAL, EEPROM_PH_AUTO_INTERVAL_ADDR);
  EEPROMWriteUint16(DEFAULT_PH_MANUAL_VOLUME, EEPROM_PH_MANUAL_VOL_ADDR);
  EEPROMWriteUint16(DEFAULT_N1N2_AUTO_VOLUME, EEPROM_N1N2_AUTO_VOL_ADDR);
  EEPROMWriteUint16(DEFAULT_N1N2_AUTO_INTERVAL, EEPROM_N1N2_AUTO_INTERVAL_ADDR);
  EEPROMWriteUint16(DEFAULT_N1N2_MANUAL_VOLUME, EEPROM_N1N2_MANUAL_VOL_ADDR);
  EEPROMWriteUint16(DEFAULT_EC_SETPOINT, EEPROM_EC_SETPOINT_ADDR);
  EEPROMWriteUint16(DEFAULT_PH_SETPOINT, EEPROM_PH_SETPOINT_ADDR);
  EEPROMWriteUint16(DEFAULT_XIC_MODE, EEPROM_PH_DOWN_HOA_ADDR);
  EEPROMWriteUint16(DEFAULT_XIC_MODE, EEPROM_N1N2_MODE_ADDR);
  EEPROMWriteUint16(DEFAULT_XIC_MODE, EEPROM_HS_105HOA_ADDR);
  EEPROMWriteUint16(DEFAULT_XIC_MODE, EEPROM_HS_101HOA_ADDR);
  EEPROMWriteUint16(DEFAULT_XIC_MODE, EEPROM_HS_102HOA_ADDR);
  EEPROMWriteUint16(DEFAULT_XIC_MODE, EEPROM_HS_103HOA_ADDR);
  EEPROMWriteUint16(DEFAULT_XIC_MODE, EEPROM_HS_104HOA_ADDR);

  EEPROMWriteTimeEntry(0, EEPROM_PH_HIGH_CAL_EPOCH_ADDR);
  EEPROMWriteTimeEntry(0, EEPROM_PH_MID_CAL_EPOCH_ADDR);
  EEPROMWriteTimeEntry(0, EEPROM_PH_LOW_CAL_EPOCH_ADDR);

  EEPROMWriteTimeEntry(0, EEPROM_EC_DRY_CAL_EPOCH_ADDR);
  EEPROMWriteTimeEntry(0, EEPROM_EC_HIGH_CAL_EPOCH_ADDR);
  EEPROMWriteTimeEntry(0, EEPROM_EC_LOW_CAL_EPOCH_ADDR);
}

void EEPROMWriteUint16(uint16_t duration, uint16_t atAddress)
{
  eeprom_write_block((const void *)&duration, (void *)atAddress,
                     sizeof(duration));
}

uint16_t EEPROMReadUint16(uint16_t atAddress)
{
  uint16_t duration = 0;

  eeprom_read_block((void *)&duration, (void *)atAddress, sizeof(duration));
  return duration;
}

void EEPROMWriteUint32(uint32_t value, uint16_t atAddress)
{
  eeprom_write_block((const void *)&value, (void *)atAddress,
                     sizeof(value));
}

uint32_t EEPROMReadUint32(uint16_t atAddress)
{
  uint32_t value = 0;

  eeprom_read_block((void *)&value, (void *)atAddress, sizeof(value));
  return value;
}

void EEPROMWriteTimeEntry(time_t epoch, uint16_t atAddress)
{
  eeprom_write_block((const void *)&epoch, (void *)atAddress, sizeof(epoch));
}

time_t EEPROMReadTimeEntry(uint16_t atAddress)
{
  time_t epoch = 0;

  eeprom_read_block((void *)&epoch, (void *)atAddress, sizeof(epoch));
  return epoch;
}

uint16_t isEEPROMConfigured()
{
  uint8_t configFlag;

  eeprom_read_block((void *)&configFlag, (void *)EEPROM_CONFIG_FLAG_ADDR,
                    sizeof(configFlag));
  return configFlag;
}

void EEPROMLoadConfig()
{
  LOG_TRACE("EEPROMLoadConfig");
  growingChamberLights.offEpoch = EEPROMReadTimeEntry(
      EEPROM_GROWING_CHAMBER_OFF_TIME_ADDR);
  growingChamberLights.onEventOff = doGrowingChamberLightsOff;

  growingChamberLights.onEpoch = EEPROMReadTimeEntry(
      EEPROM_GROWING_CHAMBER_ON_TIME_ADDR);
  growingChamberLights.onEventOn = doGrowingChamberLightsOn;

  seedingAreaLights.offEpoch = EEPROMReadTimeEntry(
      EEPROM_SEEDING_AREA_OFF_TIME_ADDR);
  seedingAreaLights.onEventOff = doSeedingAreaLightsOff;

  seedingAreaLights.onEpoch = EEPROMReadTimeEntry(
      EEPROM_SEEDING_AREA_ON_TIME_ADDR);
  seedingAreaLights.onEventOn = doSeedingAreaLightsOn;

  heatingPad.offEpoch = EEPROMReadTimeEntry(
      EEPROM_HEATING_PAD_OFF_TIME_ADDR);
  heatingPad.onEventOff = doHeatingPadOff;

  heatingPad.onEpoch = EEPROMReadTimeEntry(
      EEPROM_HEATING_PAD_ON_TIME_ADDR);
  heatingPad.onEventOn = doHeatingPadOn;

  PY_001.setActiveDuration(EEPROMReadUint16(
      EEPROM_CIRCULATION_PUMP_ON_DURATION_ADDR));
  PY_001.setInactiveDuration(EEPROMReadUint16(
      EEPROM_CIRCULATION_PUMP_OFF_DURATION_ADDR));

  // PY_002.setActiveDuration(EEPROMReadUint16(
  //     EEPROM_DRAIN_PUMP_ON_DURATION_ADDR));
  // PY_002.setInactiveDuration(EEPROMReadUint16(
  //     EEPROM_DRAIN_PUMP_OFF_DURATION_ADDR));

  FY_001.setActiveDuration(EEPROMReadUint16(EEPROM_FAN_ON_DURATION_ADDR));
  FY_001.setInactiveDuration(EEPROMReadUint16(EEPROM_FAN_OFF_DURATION_ADDR));

  // XIC_001.setSP(((float)EEPROMReadUint16(EEPROM_PH_SETPOINT_ADDR)) / 10.0);
  // XIC_001.setAutoVolume(EEPROMReadUint16(EEPROM_PH_AUTO_VOL_ADDR));
  // XIC_001.setAutoInterval(EEPROMReadUint16(EEPROM_PH_AUTO_INTERVAL_ADDR));
  // XIC_001.setManualVolume(EEPROMReadUint16(EEPROM_PH_MANUAL_VOL_ADDR));
  // XIC_001.setControlMode(EEPROMReadUint16(EEPROM_PH_DOWN_HOA_ADDR));

  //  XIC_002.setAutoVolume(EEPROMReadUint16(EEPROM_N1N2_AUTO_VOL_ADDR));
  //  XIC_002.setAutoInterval(EEPROMReadUint16(EEPROM_N1N2_AUTO_INTERVAL_ADDR));
  //  XIC_002.setManualVolume(EEPROMReadUint16(EEPROM_N1N2_MANUAL_VOL_ADDR));
  //  XIC_002.setControlMode(EEPROMReadUint16(EEPROM_N1N2_MODE_ADDR));
  //  XIC_002.setSP(EEPROMReadUint16(EEPROM_EC_SETPOINT_ADDR));

  //  XIC_003.setAutoVolume(EEPROMReadUint16(EEPROM_N1N2_AUTO_VOL_ADDR));
  //  XIC_003.setAutoInterval(EEPROMReadUint16(EEPROM_N1N2_AUTO_INTERVAL_ADDR));
  //  XIC_003.setManualVolume(EEPROMReadUint16(EEPROM_N1N2_MANUAL_VOL_ADDR));
  //  XIC_003.setControlMode(EEPROMReadUint16(EEPROM_N1N2_MODE_ADDR));
  //  XIC_003.setSP(EEPROMReadUint16(EEPROM_EC_SETPOINT_ADDR));

  // HS_105AB.setRemoteState(EEPROMReadUint16(EEPROM_HS_105HOA_ADDR));

  // HS_101AB.setRemoteState(EEPROMReadUint16(EEPROM_HS_101HOA_ADDR));
  // HS_102AB.setRemoteState(EEPROMReadUint16(EEPROM_HS_102HOA_ADDR));
  // HS_103AB.setRemoteState(EEPROMReadUint16(EEPROM_HS_103HOA_ADDR));
  // HS_104AB.setRemoteState(EEPROMReadUint16(EEPROM_HS_104HOA_ADDR));
}

void onHOACommand(int8_t index, HASelect *sender)
{
  LOG_TRACE("onHOACommand: sender:", sender->getName());
  if (sender == &circulationPumpHOA)
  {
    on_Circulation_Pump_Process(index);

    circulationPumpHOA.persist(index);
  }

  else if (sender == &seedingAreaLEDHOA)
  {

    on_SeedingAreaLED_Process(index);
    seedingAreaLEDHOA.persist(index);
  }
  else if (sender == &growingChamberLEDHOA)
  {

    on_GrowingChamberLED_Process(index);
    growingChamberLEDHOA.persist(index);
  }
  else if (sender == &heatingPadHOA)
  {
    on_HeatingPad_Process(index);
    heatingPadHOA.persist(index);
  }
  else if (sender == &fanHOA)
  {
    // setGrowingChamberFanHOA(index + 1);

    on_Fan_Process(index);
    fanHOA.persist(index);
  }
  // else if (sender == &drainPumpHOA)
  else if (sender == &phCalSel)
    doPHCalibration(index + 1);
  else if (sender == &ecCalSel)
    doECCalibration(index + 1);
}

void onCirculationPumpDurationCommand(HANumeric number, HANumber *sender)
{

  if (sender == &circulationPumpOnDuration)
  {
    setCirculationPumpOnDuration(number.toUInt32());
    circulationPumpOnDuration.setState(number);
  }
  else
  {
    setCirculationPumpOffDuration(number.toUInt32());
    circulationPumpOffDuration.setState(number);
  }
}

void onFanDurationCommand(HANumeric number, HANumber *sender)
{
  if (sender == &fanOnDuration)
  {
    setFanOnDuration(number.toUInt32());
    fanOnDuration.setState(number);
  }
  else
  {
    setFanOffDuration(number.toUInt32());
    fanOffDuration.setState(number);
  }
}

void onGrowingChamberLEDTimeCommand(HANumeric number, HANumber *sender)
{
  LOG_TRACE("onGrowingChamberLEDTimeCommand:");
  if (sender == &growingChamberLEDOnTime)
  {
    LOG_TRACE("onGrowingChamberLEDTimeCommand - On:", number.toUInt32());
    setGrowingChamberLightsOnTime(number.toUInt32());
    growingChamberLEDOnTime.setState(number);
  }
  else
  {
    LOG_TRACE("onGrowingChamberLEDTimeCommand - Off:", number.toUInt32());
    setGrowingChamberLightsOffTime(number.toUInt32());
    growingChamberLEDOffTime.setState(number);
  }
}

void onSeedingAreaLEDTimeCommand(HANumeric number, HANumber *sender)
{

  LOG_TRACE("onSeedingAreaLEDTimeCommand:");
  if (sender == &seedingAreaLEDOnTime)
  {

    setSeedingAreaLightsOnTime(number.toUInt32());
    seedingAreaLEDOnTime.setState(number);
  }
  else
  {

    setSeedingAreaLightsOffTime(number.toUInt32());
    seedingAreaLEDOffTime.setState(number);
  }
}

void seedingAreaPadTimeCommand(HANumeric number, HANumber *sender)
{
  LOG_TRACE("seedingAreaHeatingPadOffTimeCommand():", number.toUInt32());

  if (sender == &seedingAreaPadOnTime)
  {

    setHeatingPadOnTime(number.toUInt32());
    seedingAreaPadOnTime.setState(number);
  }
  else
  {
    setHeatingPadOffTime(number.toUInt32());
    seedingAreaPadOffTime.setState(number);
  }
}
