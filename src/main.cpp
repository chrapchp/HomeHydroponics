  #include <Arduino.h>

/**
 *  @file   main.cpp
 *  @author  peter c
 *  @date    2018Mar6
 *  @version 1.0.0
 *
 *
 *  @section DESCRIPTION
 *  Control an ebb and flow hydroponic setup. Data published via as a modbus
 * slave and setpoints based on arduino code migrated to platformIO
 *
 */

#include <Time.h>
#include <TimeLib.h>
#include <Timezone.h> // https://github.com/JChristensen/Timezone
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>      // for RTC
#include <DS3232RTC.h> // http://github.com/JChristensen/DS3232RTC
#include <avr/eeprom.h>
#include <Streaming.h>
#include <NewPing.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>               // DHT-22 humidity sensor
#include <RunningMedian.h>     // https://github.com/RobTillaart/Arduino/tree/master/libraries/RunningMedian

#include <DA_Flowmeter.h>
#include <DA_Discreteinput.h>
#include <DA_DiscreteOutput.h>
#include <DA_DiscreteOutputTmr.h>
#include <DA_HOASwitch.h>
#include <DA_AtlasPH.h>
#include <DA_AtlasEC.h>
#include <DA_PeristalticPump.h>
#include <DA_NonBlockingDelay.h>

#include "PlantModbus.h"
#include "DA_NutrientController.h"
//#include "DA_NutrientController.h"
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

#define DEFAULT_HEARTBEAT_DURATION 2              // seconds

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

#define EEPROM_CONFIGURED 2       // this value stored at address
                                  // CONFIG_FLAG_ADDR
#define EEPROM_CONFIG_FLAG_ADDR 0 // is 0 if nothing was written
#define EEPROM_GROWING_CHAMBER_ON_TIME_ADDR EEPROM_CONFIG_FLAG_ADDR + \
                                                sizeof(uint8_t)
#define EEPROM_GROWING_CHAMBER_OFF_TIME_ADDR EEPROM_GROWING_CHAMBER_ON_TIME_ADDR + \
                                                 sizeof(time_t)
#define EEPROM_SEEDING_AREA_ON_TIME_ADDR EEPROM_GROWING_CHAMBER_OFF_TIME_ADDR + \
                                             sizeof(time_t)
#define EEPROM_SEEDING_AREA_OFF_TIME_ADDR EEPROM_SEEDING_AREA_ON_TIME_ADDR + \
                                              sizeof(time_t)
#define EEPROM_FAN_ON_DURATION_ADDR EEPROM_SEEDING_AREA_OFF_TIME_ADDR + \
                                        sizeof(time_t)
#define EEPROM_FAN_OFF_DURATION_ADDR EEPROM_FAN_ON_DURATION_ADDR + \
                                         sizeof(time_t)
#define EEPROM_CIRCULATION_PUMP_ON_DURATION_ADDR EEPROM_FAN_OFF_DURATION_ADDR + \
                                                     sizeof(uint16_t)
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

#define VERSION 140 // implied two decimal

// comment out to not include terminal processing
// #define PROCESS_TERMINAL
// #define PROCESS_TERMINAL_VERBOSE
Stream *debugOutputStream = NULL;

// #define PROCESS_MODBUS

// refresh intervals
const uint32_t POLL_CYCLE_MS = 2 * 1000;                       // every 2 s
const uint32_t TEMPERATURE_COMPENSATE_CYCLE = 60 * 60 * 1000L; // every hour
const uint32_t FLOW_CALC_PERIOD_MS = 1 * 1000;                 // every 1 sec

// flow meter
#define FT002_SENSOR_INTERUPT_PIN 2

#define ENABLE_FT002_SENSOR_INTERRUPTS attachInterrupt(digitalPinToInterrupt(          \
                                                           FT002_SENSOR_INTERUPT_PIN), \
                                                       onFT_002_PulseIn,               \
                                                       RISING)
#define DISABLE_FT002_SENSOR_INTERRUPTS detachInterrupt(digitalPinToInterrupt( \
    FT002_SENSOR_INTERUPT_PIN))

// iterrupt pin,calculation period in seconds
DA_FlowMeter FT_002(FT002_SENSOR_INTERUPT_PIN, FLOW_CALC_PERIOD_MS / 1000);

#define FT003_SENSOR_INTERUPT_PIN 3
#define ENABLE_FT003_SENSOR_INTERRUPTS attachInterrupt(digitalPinToInterrupt(          \
                                                           FT003_SENSOR_INTERUPT_PIN), \
                                                       onFT_003_PulseIn,               \
                                                       RISING)
#define DISABLE_FT003_SENSOR_INTERRUPTS detachInterrupt(digitalPinToInterrupt( \
    FT003_SENSOR_INTERUPT_PIN))

// iterrupt pin,calculation period in seconds
DA_FlowMeter FT_003(FT003_SENSOR_INTERUPT_PIN, FLOW_CALC_PERIOD_MS / 1000);

#define CO2_INTERRUPT_PIN 18
#define ENABLE_CO2_SENSOR_RISING_INTERRUPTS attachInterrupt(digitalPinToInterrupt(  \
                                                                CO2_INTERRUPT_PIN), \
                                                            on_AT_102_Rising,       \
                                                            RISING)
#define ENABLE_CO2_SENSOR_FALLING_INTERRUPTS attachInterrupt(digitalPinToInterrupt(  \
                                                                 CO2_INTERRUPT_PIN), \
                                                             on_AT_102_Falling,      \
                                                             FALLING)
#define DISABLE_CO2_SENSOR_INTERRUPTS detachInterrupt(digitalPinToInterrupt( \
    CO2_INTERRUPT_PIN))

// CO2 timing vars
#define CO2_PERIOD 1004 // 1 PWM cycle in millisecs
volatile uint32_t timeOn_AT_102Start = 0;
volatile uint16_t AT_102Raw = 0;
RunningMedian AT_102MedianFilter = RunningMedian(5);



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
void displayDateTime();
void displayHOAStatuses(bool clearScreen);
void displayHomeScreen(bool clearScreen);
void displayMiscStatuses(bool clearScreen);
void displayTimerStatuses(bool clearScreen);
void do_ONP_SPoll();
void on_RemoteLocalToggle(bool state,
                          int aPin);

void doOnCalcFlowRate();
void doOnMidnight();
void doOnTemperatureCompensate();
void EEPROMLoadConfig();
time_t EEPROMReadEventEntry(uint16_t atAddress);
void EEPROMWriteAlarmEntry(time_t epoch,
                           uint16_t atAddress);
void EEPROMWriteDefaultConfig();
void EEPROMWriteUint16(uint16_t duration,
                       uint16_t atAddress);
bool getModbusCoilValue(uint8_t startAddress,
                        uint8_t bitPos);
void initOneWire();
void initOneWireDevice(DeviceAddress aDevice,
                       uint8_t aIndex);
uint16_t isEEPROMConfigured();
uint16_t EEPROMReadUint16(uint16_t atAddress);
void on_AT_102_Falling();
void on_AT_102_Falling();
void on_AT_102_Rising();
void on_Circulation_Pump_Process(DA_HOASwitch::HOADetectType state);
void on_DrainPump_Process(bool state,
                          int aPin);
void on_GrowingChamberLED_Process(DA_HOASwitch::HOADetectType state);
void on_HeatingPad_Process(DA_HOASwitch::HOADetectType state);
void on_Fan_Process(DA_HOASwitch::HOADetectType state);
void on_InletValve_Process(bool state,
                           int aPin);
void on_SeedingAreaLED_Process(DA_HOASwitch::HOADetectType state);
void on_pHDownProcess(DA_HOASwitch::HOADetectType state);
void on_AeratorProcess(DA_HOASwitch::HOADetectType state);

void onAtlasECSample(IO_TYPE type,
                     float value);
void onAtlasPhSample(IO_TYPE type,
                     float value);
void onFT_002_PulseIn();
void onFT_003_PulseIn();
void printOneWireAddress(Stream *debugOutputStream,
                         DeviceAddress aDeviceAddress,
                         bool aCR);
void processModbusCommands();
void processModbusECCommands();
void processModbusPHCommands();
void processModbusXICCommands();
void processModbusRemoteHOAs();
void refreshDiscreteInputs();
void refreshDiscreteOutputs();
void refreshModbusRegisters();
void setConfigToDefaults();
void setModbusCirculationPumpOffDuration();
void setModbusCirculationPumpOnDuration();
void setModbusFanOffDuration();
void setModbusFanOnDuration();
void setModbusGrowingChamberLightsOffTime();
void setModbusGrowingChamberLightsOnTime();
void setModbusSeedingAreaLightsOffTime();
void setModbusSeedingAreaLightsOffTime();
void setModbusSeedingAreaLightsOnTime();
void setModbusHeatingPadOnTime();
void setModbusHeatingPadOffTime();
void setModbusCirculationDrainOffDuration();
void setModbusCirculationDrainOnDuration();

void setModbusTime();
void setupRTC();
void dateTimeToBuffer(time_t aTime,
                      char *bufferMem);
void timeToBuffer(time_t aTime,
                  char *bufferMem);
void writeModbusCoil(uint8_t startAddress,
                     uint8_t bitPos,
                     bool value);

void on_AeratorProcess();
#if defined(PROCESS_TERMINAL)
void processTerminalCommands();
void processCalibrateMessage(IO_TYPE aIO_Type);
void processTemperatureCompensation();
void processSerializeMessage();
void processTimeSetMessage();
void processControllerMessage();
void processSoftHOAMessage();
void showCommands();
void processFanDurations();
void processCirculationPumpDurations();
void processDrainPumpDurations();
void processSAAlarmMessage();
void processGCAlarmMessage();
void processLightEntryMessage(time_t *anEpoch,
                              uint16_t eepromAddr);

void processLightsMessage();
void processDisplayMessage();
void processDisplayIOMessage();
void displayEventControlEntry(char *who,
                              time_t anEpoch);

#endif // ifdef PROCESS_TERMINAL


// DHT-22 - one wire type humidity sensor (won't work with one wire lib)
#define DHT_BUS_PIN 5
DHT AT_101 = DHT(DHT_BUS_PIN, DHT22);
float AT_101T = NAN;
float AT_101H =  NAN;
float AT_101HI = NAN;

// atlas sensor
IO_TYPE currentIOType = i2c_ph;

// One Wire
//
//
DeviceAddress ambientTemperatureAddress =
    {
        0x28, 0x6F, 0xE3, 0xA0, 0x04, 0x00, 0x00, 0x5A};

float TT_001T = NAN;
DeviceAddress mixtureTemperatureAddress =
    {
        0x28, 0xFF, 0xF4, 0xF6, 0x84, 0x16, 0x05, 0x0C};

#define WIRE_BUS_PIN 4 // pin
#define ONE_TEMPERATURE_PRECISION 9
OneWire oneWire(WIRE_BUS_PIN);
DallasTemperature sensors(&oneWire);

// Analog Inputs
//
// NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
#define NUTRIENT_TANK_HEIGHT 45    // height of nutrient tank in cm
#define NUTRIENT_TANK_AIR_GAP 16.5 // space between sensor and max water level
                                   // in cm
#define NUTRIENT_DEAD_BAND 5.0     // % change allowed between samples as sensor
                                   // bounces because of noise issue
// volume 28.5 * 76.2 * 45.75 => 100 L volume peroxyde ration 3 ml per 3.8 L
const float NUTRIENT_TANK_MIXTURE_MAX = NUTRIENT_TANK_HEIGHT -
                                        NUTRIENT_TANK_AIR_GAP;

#if defined(NO_PING)
NewPing LT_002(15, 16, NUTRIENT_TANK_HEIGHT); // Water Level
float LT_002Raw = 0.0;
RunningMedian LT_002MedianFilter = RunningMedian(5);
#endif // if defined(NO_PING)

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


// Heartbeat LED
DA_DiscreteOutputTmr HY_001 = DA_DiscreteOutputTmr(13,
                                                   LOW,
                                                   DEFAULT_HEARTBEAT_DURATION,
                                                   DEFAULT_HEARTBEAT_DURATION);

// Circulation Pump -> 120V AC
DA_DiscreteOutputTmr PY_001 = DA_DiscreteOutputTmr(33,
                                                   LOW,
                                                   DEFAULT_CIRCULATION_PUMP_ON_DURATION,
                                                   DEFAULT_CIRCULATION_PUMP_OFF_DURATION);

// Fan -> 120V AC
DA_DiscreteOutputTmr MY_101 = DA_DiscreteOutputTmr(34,
                                                   LOW,
                                                   DEFAULT_FAN_ON_DURATION,
                                                   DEFAULT_FAN_OFF_DURATION);

// Inlet H20 Valve 12 VDC
DA_DiscreteOutput VY_001A = DA_DiscreteOutput(37, LOW);

// Drain Pump 12 VDC
//DA_DiscreteOutputTmr PY_002 = DA_DiscreteOutputTmr(38,
//                                                   LOW,
//                                                   DEFAULT_DRAIN_PUMP_ON_DURATION,
//                                                   DEFAULT_DRAIN_PUMP_OFF_DURATION);
DA_DiscreteOutput PY_002 = DA_DiscreteOutput(38, LOW);

// Discrete Inputs
//
// Nutrient Mixture Tag Hi-Hi Level switch 0=Hi-Hi
DA_DiscreteInput LSHH_002 = DA_DiscreteInput(42,
                                             DA_DiscreteInput::ToggleDetect,
                                             true);

// Drain pump Hand Status: Start/Stop
DA_DiscreteInput HS_001 = DA_DiscreteInput(43,
                                           DA_DiscreteInput::ToggleDetect,
                                           true);

// Smoke Detector
DA_DiscreteInput SSH_101 = DA_DiscreteInput(9);

// Inlet H20 Open/Close
DA_DiscreteInput HS_002 = DA_DiscreteInput(49,
                                           DA_DiscreteInput::ToggleDetect,
                                           true);



// Circulation Pump Hand : HOA
DA_HOASwitch HS_105AB = DA_HOASwitch();

// Seeding Area LED : HOA
DA_HOASwitch HS_102AB = DA_HOASwitch();

// Growing Chamber LED : HOA
DA_HOASwitch HS_103AB = DA_HOASwitch();

// Heating pad HOA (remote only)
DA_HOASwitch HS_104AB = DA_HOASwitch();

// Fan HOA (remote only)
DA_HOASwitch HS_101AB = DA_HOASwitch();

// pH Down Pump HOA (remote only)
DA_HOASwitch HS_011 = DA_HOASwitch();

// Aerator HOA (remote only)
DA_HOASwitch HS_201 = DA_HOASwitch();

DA_NutrientController XIC_001 = DA_NutrientController(45, HIGH, 6.5,
                                                      DA_NutrientController::RISINGTREND);

// Nutrient 1 Pump Controller
//DA_NutrientController XIC_002 = DA_NutrientController(48, HIGH,

// Nutrient 2 Pump Controller
//DA_NutrientController XIC_003 = DA_NutrientController(47,

// Hydrogen Peroxide Pump
// DA_PeristalticPump XY_004 = DA_PeristalticPump(47, HIGH);

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
DA_NonBlockingDelay KI_002 = DA_NonBlockingDelay(FLOW_CALC_PERIOD_MS,
                                                 doOnCalcFlowRate);
DA_NonBlockingDelay KI_003 = DA_NonBlockingDelay(TEMPERATURE_COMPENSATE_CYCLE,
                                                 doOnTemperatureCompensate);

void setup()
{
#if defined(PROCESS_TERMINAL)
  //Serial3.begin(9600);
  //Stream *debugOutputStream = &Serial3;
  Serial.begin(9600);
  debugOutputStream = &Serial;
#endif // ifdef PROCESS_TERMINAL

#if defined(PROCESS_MODBUS)
#if defined(MODBUS_IP)
  //Ethernet.init(53);  
//pinMode(53, OUTPUT);
  Ethernet.begin(defaultMAC, defaultIP, defaultGateway, defaultSubnet);
#else
  slave.begin(MB_SERIAL_BAUD);
#endif
#endif // ifdef PROCESS_MODBUS

  randomSeed(analogRead(0));

  setupRTC();

  HS_002.setPollingInterval(500);   // ms
  LSHH_002.setDebounceTime(1000);   // float switch bouncing around
  LSHH_002.setPollingInterval(500); // ms
  HS_002.setOnEdgeEvent(&on_InletValve_Process);
  LSHH_002.setOnEdgeEvent(&on_InletValve_Process);
  HS_001.setPollingInterval(200); // ms
  // HS_001.setDebounceTime( 110);
  HS_001.setOnEdgeEvent(&on_DrainPump_Process);



  HS_105AB.setOnStateChangeDetect(&on_Circulation_Pump_Process);
  HS_101AB.setOnStateChangeDetect(&on_Fan_Process);
  HS_102AB.setOnStateChangeDetect(&on_SeedingAreaLED_Process);
  HS_103AB.setOnStateChangeDetect(&on_GrowingChamberLED_Process);
  HS_104AB.setOnStateChangeDetect(&on_HeatingPad_Process);

  HS_011.setOnStateChangeDetect(&on_pHDownProcess);

  HS_201.setOnStateChangeDetect(&on_AeratorProcess);

  AT_001.setOnPollCallBack(onAtlasPhSample);
  AT_001.setPollingInterval(3000);
  AT_001.retrieveCompensatedTemperature();

  AT_002.setOnPollCallBack(onAtlasECSample);
  AT_002.setPollingInterval(2000);
  AT_002.retrieveCompensatedTemperature();

  // XIC_001.serialize(debugOutputStream, true);
  // XIC_002.serialize(debugOutputStream, true);
  // XIC_003.serialize(debugOutputStream, true);
  XIC_001.setMaxFlowRate(27); // pH pump runs slower than the other two for some
                              // reason
  XIC_001.setAutoControlParameters(10, 120);
  // 1-wire
  sensors.begin();
  initOneWire();

  // humidity sensor
  AT_101.begin();
  ENABLE_FT002_SENSOR_INTERRUPTS;
  FT_002.setMeterFactor(7.5);
  FT_003.setMeterFactor(7.5);
  ENABLE_CO2_SENSOR_RISING_INTERRUPTS;

  if (isEEPROMConfigured() == EEPROM_CONFIGURED)
  {

    EEPROMLoadConfig();
  }
  else
  {
    EEPROMWriteDefaultConfig();
    EEPROMLoadConfig();
  }

  MY_101.start(DA_DiscreteOutputTmr::Continuous);
  PY_001.start(DA_DiscreteOutputTmr::Continuous);
  //PY_002.start(DA_DiscreteOutputTmr::Continuous);
  HY_001.start(DA_DiscreteOutputTmr::Continuous);
}

void loop()
{

#if defined(PROCESS_MODBUS)
#if defined(MODBUS_IP)
  slave.MbsRun();
#else
  slave.poll(modbusRegisters, MODBUS_REG_COUNT);
#endif
  refreshModbusRegisters();
  processModbusCommands();
#endif // ifdef PROCESS_MODBUS

#if defined(PROCESS_TERMINAL)
  processTerminalCommands();
#endif // ifdef PROCESS_TERMINAL

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

  KI_001.refresh();
  KI_002.refresh();
  KI_003.refresh();

#ifndef PROCESS_TERMINAL
  XIC_001.setPV(AT_001.getSample());
#endif
  XIC_001.refresh();
}

void onFT_002_PulseIn()
{
  FT_002.handleFlowDetection();
}

void onFT_003_PulseIn()
{
  FT_003.handleFlowDetection();
}

void on_AT_102_Rising()
{
  DISABLE_CO2_SENSOR_INTERRUPTS;
  timeOn_AT_102Start = millis();

  // uint32_t timestamp = micros();
  // timeCycle = micros();

  // timeOff_AT_102Raw= timeOn_AT_102Start - timeOff_AT_102Start;
  ENABLE_CO2_SENSOR_FALLING_INTERRUPTS;
}

void on_AT_102_Falling()
{
  DISABLE_CO2_SENSOR_INTERRUPTS;
  uint32_t timeOn = (uint32_t)abs(millis() - timeOn_AT_102Start);
  uint16_t tempAT_102Raw;

  uint32_t timeOff = CO2_PERIOD - timeOn;

  tempAT_102Raw = (uint16_t)(2000 * (timeOn - 2) / (timeOn + timeOff - 4));

  if (tempAT_102Raw <= 2000)
  {
    AT_102Raw = tempAT_102Raw;
  }

  // Serial << "AT-102="<< AT_102Raw<< " timeOn=" << timeOn << " timeOff=" <<
  // timeOff << endl;
  ENABLE_CO2_SENSOR_RISING_INTERRUPTS;
}

/*
   Only open inlet H20 Valve iff no Hi-Hi water level
   note LSHH is high on high level (fail safe )
 */
void on_InletValve_Process(bool state, int aPin)
{
#if defined(PROCESS_TERMINAL_VERBOSE)
  *debugOutputStream << "on_InletValve_Process HS_002, LSHH_002" << endl;
  HS_002.serialize(debugOutputStream, true);
  LSHH_002.serialize(debugOutputStream, true);
#endif // ifdef PROCESS_TERMINAL_VERBOSE

  if ((HS_002.getSample() == LOW) &&
      (LSHH_002.getSample() == HIGH))
    VY_001A.activate();
  else
    VY_001A.reset();
}

void on_DrainPump_Process(bool state, int aPin)
{
#if defined(PROCESS_TERMINAL_VERBOSE)
  *debugOutputStream << "on_DrainPump_Process HS_001" << endl;
  HS_001.serialize(debugOutputStream, true);
#endif // ifdef PROCESS_TERMINAL_VERBOSE

  if (HS_001.getSample() == LOW)
  {
    //PY_002.enable();
    //PY_002.reset();
      PY_002.forceActive();
  }
  else
  {
    //PY_002.pauseTimer();
    PY_002.disable();

  }

}

void on_Circulation_Pump_Process(DA_HOASwitch::HOADetectType state)
{
#if defined(PROCESS_TERMINAL_VERBOSE)
  *debugOutputStream << "on_Circulation_Pump_Process HS_105AB" << endl;
  HS_105AB.serialize(debugOutputStream, true);
#endif // ifdef PROCESS_TERMINAL_VERBOSE

  switch (state)
  {
  case DA_HOASwitch::Hand:

    PY_001.pauseTimer();
    PY_001.disable();
    PY_001.forceActive(); // force the pump on

    break;

  case DA_HOASwitch::Off:
    PY_001.pauseTimer();
    PY_001.disable();

    break;

  case DA_HOASwitch::Auto:
    PY_001.enable();
    PY_001.resumeTimer();

    // PY_001.restart();
    // PY_001.resume();
    break;

  default:
    break;
  }
}

void on_HeatingPad_Process(DA_HOASwitch::HOADetectType state)
{
#if defined(PROCESS_TERMINAL_VERBOSE)
  *debugOutputStream << "on_HeatingPad_Process HS_104AB" << endl;
  HS_104AB.serialize(debugOutputStream, true);
#endif // ifdef PROCESS_TERMINAL_VERBOSE

  switch (state)
  {
  case DA_HOASwitch::Hand:
    DY_104.disable();
    DY_104.forceActive(); // force the Flowing on
    break;

  case DA_HOASwitch::Off:
    DY_104.disable();
    break;

  case DA_HOASwitch::Auto:
    DY_104.enable();
    break;

  default:
    break;
  }
}

void on_AeratorProcess(DA_HOASwitch::HOADetectType state)
{
#if defined(PROCESS_TERMINAL_VERBOSE)
  *debugOutputStream << "on_AeratorProcess HS_201" << endl;
  HS_201.serialize(debugOutputStream, true);
#endif // ifdef PROCESS_TERMINAL_VERBOSE

  switch (state)
  {
  case DA_HOASwitch::Hand:

    MY_101.pauseTimer();
    MY_101.disable();
    MY_101.forceActive(); // force the fan on

    break;

  case DA_HOASwitch::Off:
    MY_101.pauseTimer();
    MY_101.disable();

    break;

  case DA_HOASwitch::Auto:
    MY_101.enable();
    MY_101.resumeTimer();

    // PY_001.restart();
    // PY_001.resume();
    break;

  default:
    break;
  }
}

void on_Fan_Process(DA_HOASwitch::HOADetectType state)
{
#if defined(PROCESS_TERMINAL_VERBOSE)
  *debugOutputStream << "on_Fan_Process HS_101AB" << endl;
  HS_101AB.serialize(debugOutputStream, true);
#endif // ifdef PROCESS_TERMINAL_VERBOSE

  switch (state)
  {
  case DA_HOASwitch::Hand:

    MY_101.pauseTimer();
    MY_101.disable();
    MY_101.forceActive(); // force the fan on

    break;

  case DA_HOASwitch::Off:
    MY_101.pauseTimer();
    MY_101.disable();

    break;

  case DA_HOASwitch::Auto:
    MY_101.enable();
    MY_101.resumeTimer();

    // PY_001.restart();
    // PY_001.resume();
    break;

  default:
    break;
  }
}

void on_GrowingChamberLED_Process(DA_HOASwitch::HOADetectType state)
{
#if defined(PROCESS_TERMINAL)
  *debugOutputStream << "on_GrowingChamberLED_Process HS_103AB" << endl;
  HS_103AB.serialize(debugOutputStream, true);
#endif // ifdef PROCESS_TERMINAL_VERBOSE

  switch (state)
  {
  case DA_HOASwitch::Hand:
    DY_103.disable();
    DY_103.forceActive(); // force the Flowing on
    break;

  case DA_HOASwitch::Off:
    DY_103.disable();
    break;

  case DA_HOASwitch::Auto:
    DY_103.enable();
    break;

  default:
    break;
  }
}

void on_SeedingAreaLED_Process(DA_HOASwitch::HOADetectType state)
{
#if defined(PROCESS_TERMINAL_VERBOSE)
  *debugOutputStream << "on_NonFlowingLED_Process HS_102AB" << endl;
  HS_102AB.serialize(debugOutputStream, true);
#endif // ifdef PROCESS_TERMINAL_VERBOSE

  switch (state)
  {
  case DA_HOASwitch::Hand:
    DY_102.disable();
    DY_102.forceActive(); // force the Flowing on
    break;

  case DA_HOASwitch::Off:
    DY_102.disable();
    break;

  case DA_HOASwitch::Auto:
    DY_102.enable();
    break;

  default:
    break;
  }
}

void on_pHDownProcess(DA_HOASwitch::HOADetectType state)
{
#if defined(PROCESS_TERMINAL)
  *debugOutputStream << "on_pHDownProcess HS_011" << endl;
  HS_011.serialize(debugOutputStream, true);
#endif // ifdef PROCESS_TERMINAL_VERBOSE
  XIC_001.setControlMode(HS_011.getCurrentState());
}

void setupRTC()
{
  setSyncProvider(RTC.get);

  // setSyncInterval(30);
  if (timeStatus() != timeSet)
  {
#if defined(PROCESS_TERMINAL)
    *debugOutputStream << F("Unable to sync with the RTC-setting to default") << endl;
#endif                     // ifdef PROCESS_TERMINAL
    RTC.set(DEFAULT_TIME); // set the RTC and the system time to the received
                           // value
    setTime(DEFAULT_TIME); // Sync Arduino clock to the time received on the
                           // Serial2 port
  }
  else
  {
#if defined(PROCESS_TERMINAL)
    *debugOutputStream << F("RTC has set the system time") << endl;
    *debugOutputStream << F("Enter Command:") << endl;
#endif // ifdef PROCESS_TERMINAL
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

void refreshDiscreteInputs()
{
  LSHH_002.refresh();
  HS_002.refresh();
  HS_001.refresh();
  HS_105AB.refresh();
  HS_101AB.refresh();
  HS_102AB.refresh();
  HS_103AB.refresh();
  HS_104AB.refresh();
  HS_011.refresh();
}

void refreshDiscreteOutputs()
{
  PY_001.refresh(); // on/off timer
  MY_101.refresh(); // on/off timer
  HY_001.refresh(); // on/off timer

}

void doOnMidnight()
{
  // FT_002.dayRollOver();
  // *debugOutputStream << "day Rollover:";
  // FT_002.serialize(debugOutputStream, true);
}

void doOnCalcFlowRate()
{
  DISABLE_FT002_SENSOR_INTERRUPTS;
  FT_002.end();
  FT_002.begin();
  DISABLE_FT003_SENSOR_INTERRUPTS;
  FT_003.end();
  FT_003.begin();

  AT_102MedianFilter.add(AT_102Raw);
  ENABLE_FT002_SENSOR_INTERRUPTS;
  ENABLE_FT003_SENSOR_INTERRUPTS;
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
  if (HS_103AB.getCurrentState() == DA_HOASwitch::Auto)
    doLightControl(
        &growingChamberLights);

  if (HS_102AB.getCurrentState() == DA_HOASwitch::Auto)
    doLightControl(
        &seedingAreaLights);

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
  AT_101H = AT_101.readHumidity(); // allow 1/4 sec to read
  AT_101T = AT_101.readTemperature();
  TT_001T = sensors.getTempC(mixtureTemperatureAddress);

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

#if defined(PROCESS_TERMINAL)

// *debugOutputStream << "...Growing Chamber Lights on" << endl;
#endif // ifdef PROCESS_TERMINAL
}

void doGrowingChamberLightsOff()
{
  DY_103.reset();

// DY_103.reset();

#if defined(PROCESS_TERMINAL)

// *debugOutputStream << "...Growing Chamber Lights off" << endl;
#endif // ifdef PROCESS_TERMINAL
}

void doSeedingAreaLightsOn()
{
  DY_102.activate(); // if disabled, it won't activate
// DY_103.activate(); // if disabled, it won't activate

#if defined(PROCESS_TERMINAL)

// *debugOutputStream << "...Seeding Lights on" << endl;
#endif // ifdef PROCESS_TERMINAL
}

void doSeedingAreaLightsOff()
{
  DY_102.reset();

// DY_103.reset();

#if defined(PROCESS_TERMINAL)

// *debugOutputStream << "...Seeding Lights off" << endl;
#endif // ifdef PROCESS_TERMINAL
}

void doHeatingPadOn()
{
  DY_104.activate(); // if disabled, it won't activate
// DY_103.activate(); // if disabled, it won't activate

#if defined(PROCESS_TERMINAL)

// *debugOutputStream << "...Seeding Lights on" << endl;
#endif // ifdef PROCESS_TERMINAL
}

void doHeatingPadOff()
{
  DY_104.reset();

// DY_103.reset();

#if defined(PROCESS_TERMINAL)

// *debugOutputStream << "...Seeding Lights off" << endl;
#endif // ifdef PROCESS_TERMINAL
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

/*
** Modbus related functions
*/
void refreshModbusRegisters()
{
  writeModbusCoil(COIL_STATUS_READ_WRITE_OFFSET, CS_HS_002, HS_002.getSample());
  writeModbusCoil(COIL_STATUS_READ_WRITE_OFFSET, CS_LSHH_002,
                  LSHH_002.getSample());
  writeModbusCoil(COIL_STATUS_READ_WRITE_OFFSET, CS_HS_001, HS_001.getSample());
  writeModbusCoil(COIL_STATUS_READ_WRITE_OFFSET, CW_DY_102, DY_102.isActive());
  writeModbusCoil(COIL_STATUS_READ_WRITE_OFFSET, CW_DY_103, DY_103.isActive());
  writeModbusCoil(COIL_STATUS_READ_WRITE_OFFSET, CW_PY_001, PY_001.isActive());
  writeModbusCoil(COIL_STATUS_READ_WRITE_OFFSET, CW_MY_101, MY_101.isActive());
  writeModbusCoil(COIL_STATUS_READ_WRITE_OFFSET, CW_DY_104, DY_104.isActive());
  writeModbusCoil(COIL_STATUS_READ_WRITE_OFFSET, CW_PY_002, PY_002.isActive());



  isNaanModbus( AT_101H );  
  modbusRegisters[HR_AT_101] = bfconvert.regsf[0];
  modbusRegisters[HR_AT_101 + 1] = bfconvert.regsf[1];

  isNaanModbus( AT_101T );  
  modbusRegisters[HR_TT_101] = bfconvert.regsf[0];
  modbusRegisters[HR_TT_101 + 1] = bfconvert.regsf[1];

#if defined(NO_PING)
  bfconvert.val = LT_002MedianFilter.getMedian();
  modbusRegisters[HR_LT_002] = bfconvert.regsf[0];
  modbusRegisters[HR_LT_002 + 1] = bfconvert.regsf[1];
#endif // if defined(NO_PING)

  bfconvert.val = FT_002.getCurrentFlowRate();
  modbusRegisters[HR_FT_002] = bfconvert.regsf[0];
  modbusRegisters[HR_FT_002 + 1] = bfconvert.regsf[1];

  bfconvert.val = FT_003.getCurrentFlowRate();
  modbusRegisters[HR_FT_003] = bfconvert.regsf[0];
  modbusRegisters[HR_FT_003 + 1] = bfconvert.regsf[1];

  bfconvert.val = TT_001T; // sensors.getTempC(mixtureTemperatureAddress);
  modbusRegisters[HR_TT_001] = bfconvert.regsf[0];
  modbusRegisters[HR_TT_001 + 1] = bfconvert.regsf[1];

  modbusRegisters[HR_KY_002] = VERSION;

  blconvert.val = heatingPad.onEpoch;
  modbusRegisters[HR_DY_104_ONT_CV] = blconvert.regsl[0];
  modbusRegisters[HR_DY_104_ONT_CV + 1] = blconvert.regsl[1];

  blconvert.val = heatingPad.offEpoch;
  modbusRegisters[HR_DY_104_OFT_CV] = blconvert.regsl[0];
  modbusRegisters[HR_DY_104_OFT_CV + 1] = blconvert.regsl[1];

  blconvert.val = seedingAreaLights.onEpoch;
  modbusRegisters[HR_DY_102_ONT_CV] = blconvert.regsl[0];
  modbusRegisters[HR_DY_102_ONT_CV + 1] = blconvert.regsl[1];

  blconvert.val = seedingAreaLights.offEpoch;
  modbusRegisters[HR_DY_102_OFT_CV] = blconvert.regsl[0];
  modbusRegisters[HR_DY_102_OFT_CV + 1] = blconvert.regsl[1];

  blconvert.val = RTC.get();

  modbusRegisters[HR_QT_001_CV] = blconvert.regsl[0];
  modbusRegisters[HR_QT_001_CV + 1] = blconvert.regsl[1];

  blconvert.val = growingChamberLights.onEpoch;
  modbusRegisters[HR_DY_103_ONT_CV] = blconvert.regsl[0];
  modbusRegisters[HR_DY_103_ONT_CV + 1] = blconvert.regsl[1];

  blconvert.val = growingChamberLights.offEpoch;
  modbusRegisters[HR_DY_103_OFT_CV] = blconvert.regsl[0];
  modbusRegisters[HR_DY_103_OFT_CV + 1] = blconvert.regsl[1];

  modbusRegisters[HR_MY_101_ONP_CV] = MY_101.getCurrentActiveDuration() / 1000;
  modbusRegisters[HR_MY_101_OFP_CV] = MY_101.getCurrentInactiveDuration() / 1000;
  modbusRegisters[HR_PY_001_ONP_CV] = PY_001.getCurrentActiveDuration() / 1000;
  modbusRegisters[HR_PY_001_OFP_CV] = PY_001.getCurrentInactiveDuration() / 1000;
  //modbusRegisters[HR_PY_002_ONP_CV] = PY_002.getCurrentActiveDuration() / 1000;
  //modbusRegisters[HR_PY_002_OFP_CV] = PY_002.getCurrentInactiveDuration() / 1000;

  // Circulation pump, gc light, seeding area HOA
  modbusRegisters[HR_HS_105_HOA_CV] = HS_105AB.getCurrentState();
  modbusRegisters[HR_HS_103_HOA_CV] = HS_103AB.getCurrentState();
  modbusRegisters[HR_HS_102_HOA_CV] = HS_102AB.getCurrentState();

  // CO2
  modbusRegisters[HR_AT_102] = (uint16_t)AT_102MedianFilter.getMedian();

  // pH
  bfconvert.val = AT_001.getSample();
  modbusRegisters[HR_AT_001] = bfconvert.regsf[0];
  modbusRegisters[HR_AT_001 + 1] = bfconvert.regsf[1];

  bfconvert.val = AT_001.getCompensatedTemperature();
  modbusRegisters[HR_AT_001_TV] = bfconvert.regsf[0];
  modbusRegisters[HR_AT_001_TV + 1] = bfconvert.regsf[1];

  // EC
  bfconvert.val = AT_002.getSample();
  modbusRegisters[HR_AT_002] = bfconvert.regsf[0];
  modbusRegisters[HR_AT_002 + 1] = bfconvert.regsf[1];

  bfconvert.val = AT_002.getTDS();
  modbusRegisters[HR_AT_002TDS] = bfconvert.regsf[0];
  modbusRegisters[HR_AT_002TDS + 1] = bfconvert.regsf[1];

  bfconvert.val = AT_002.getSalinity();
  modbusRegisters[HR_AT_002SAL] = bfconvert.regsf[0];
  modbusRegisters[HR_AT_002SAL + 1] = bfconvert.regsf[1];

  bfconvert.val = AT_002.getSpecificGravity();
  modbusRegisters[HR_AT_002SG] = bfconvert.regsf[0];
  modbusRegisters[HR_AT_002SG + 1] = bfconvert.regsf[1];

  bfconvert.val = AT_002.getCompensatedTemperature();
  modbusRegisters[HR_AT_002_TV] = bfconvert.regsf[0];
  modbusRegisters[HR_AT_002_TV + 1] = bfconvert.regsf[1];

  // XIC-001
  modbusRegisters[HR_AT_001_SP_CV] = (uint16_t)(XIC_001.getSP() * 10); // pH Setpoint (pH * 10 )
  modbusRegisters[HR_AT_011_AV_CV] = XIC_001.getAutoVolume();          // pH volume (ml)
  modbusRegisters[HR_AT_011_AT_CV] = XIC_001.getAutoInterval();        // ph volume dispensed every T (s) during auto
  modbusRegisters[HR_AT_011_MV_CV] = XIC_001.getManualVolume();        // manual volume (ml)
}

void setModbusTime()
{
  if (modbusRegisters[HW_QT_001_SP] != 0)
  {
    uint32_t pctime;
    blconvert.regsl[0] = modbusRegisters[HW_QT_001_SP];
    blconvert.regsl[1] = modbusRegisters[HW_QT_001_SP + 1];
    pctime = blconvert.val;
    RTC.set(pctime); // set the RTC and the system time to the received value
    setTime(pctime); // Sync Arduino clock to the time received on the Serial2
                     // port
    modbusRegisters[HW_QT_001_SP] = 0;
    modbusRegisters[HW_QT_001_SP + 1] = 0;
  }
}

void setModbusGrowingChamberLightsOnTime()
{
  if (modbusRegisters[HW_DY_103_ONT_SP] != 0)
  {
    blconvert.regsl[0] = modbusRegisters[HW_DY_103_ONT_SP];
    blconvert.regsl[1] = modbusRegisters[HW_DY_103_ONT_SP + 1];
    growingChamberLights.onEpoch = alarmTimeToUTC(blconvert.val);
    EEPROMWriteAlarmEntry(growingChamberLights.onEpoch,
                          EEPROM_GROWING_CHAMBER_ON_TIME_ADDR);
    modbusRegisters[HW_DY_103_ONT_SP] = 0;
    modbusRegisters[HW_DY_103_ONT_SP + 1] = 0;
  }
}

void setModbusGrowingChamberLightsOffTime()
{
  if (modbusRegisters[HW_DY_103_OFT_SP] != 0)
  {
    blconvert.regsl[0] = modbusRegisters[HW_DY_103_OFT_SP];
    blconvert.regsl[1] = modbusRegisters[HW_DY_103_OFT_SP + 1];
    growingChamberLights.offEpoch = alarmTimeToUTC(blconvert.val);
    EEPROMWriteAlarmEntry(growingChamberLights.offEpoch,
                          EEPROM_GROWING_CHAMBER_OFF_TIME_ADDR);
    modbusRegisters[HW_DY_103_OFT_SP] = 0;
    modbusRegisters[HW_DY_103_OFT_SP + 1] = 0;
  }
}

void setModbusSeedingAreaLightsOnTime()
{
  if (modbusRegisters[HW_DY_102_ONT_SP] != 0)
  {
    blconvert.regsl[0] = modbusRegisters[HW_DY_102_ONT_SP];
    blconvert.regsl[1] = modbusRegisters[HW_DY_102_ONT_SP + 1];
    seedingAreaLights.onEpoch = alarmTimeToUTC(blconvert.val);
    EEPROMWriteAlarmEntry(seedingAreaLights.onEpoch,
                          EEPROM_SEEDING_AREA_ON_TIME_ADDR);
    modbusRegisters[HW_DY_102_ONT_SP] = 0;
    modbusRegisters[HW_DY_102_ONT_SP + 1] = 0;
  }
}

void setModbusSeedingAreaLightsOffTime()
{
  if (modbusRegisters[HW_DY_102_OFT_SP] != 0)
  {
    blconvert.regsl[0] = modbusRegisters[HW_DY_102_OFT_SP];
    blconvert.regsl[1] = modbusRegisters[HW_DY_102_OFT_SP + 1];

    seedingAreaLights.offEpoch = alarmTimeToUTC(blconvert.val);
    EEPROMWriteAlarmEntry(seedingAreaLights.offEpoch,
                          EEPROM_SEEDING_AREA_OFF_TIME_ADDR);
    modbusRegisters[HW_DY_102_OFT_SP] = 0;
    modbusRegisters[HW_DY_102_OFT_SP + 1] = 0;
  }
}

void setModbusHeatingPadOnTime()
{
  if (modbusRegisters[HW_DY_104_ONT_SP] != 0)
  {
    blconvert.regsl[0] = modbusRegisters[HW_DY_104_ONT_SP];
    blconvert.regsl[1] = modbusRegisters[HW_DY_104_ONT_SP + 1];
    heatingPad.onEpoch = alarmTimeToUTC(blconvert.val);
    EEPROMWriteAlarmEntry(heatingPad.onEpoch,
                          EEPROM_HEATING_PAD_ON_TIME_ADDR);
    modbusRegisters[HW_DY_104_ONT_SP] = 0;
    modbusRegisters[HW_DY_104_ONT_SP + 1] = 0;
  }
}

void setModbusHeatingPadOffTime()
{
  if (modbusRegisters[HW_DY_104_OFT_SP] != 0)
  {
    blconvert.regsl[0] = modbusRegisters[HW_DY_104_OFT_SP];
    blconvert.regsl[1] = modbusRegisters[HW_DY_104_OFT_SP + 1];

    heatingPad.offEpoch = alarmTimeToUTC(blconvert.val);
    EEPROMWriteAlarmEntry(heatingPad.offEpoch,
                          EEPROM_HEATING_PAD_OFF_TIME_ADDR);
    modbusRegisters[HW_DY_104_OFT_SP] = 0;
    modbusRegisters[HW_DY_104_OFT_SP + 1] = 0;
  }
}

void setModbusFanOnDuration()
{
  if (modbusRegisters[HW_MY_101_ONP_SP] != 0)
  {
    MY_101.setActiveDuration(modbusRegisters[HW_MY_101_ONP_SP]);

    EEPROMWriteUint16(modbusRegisters[HW_MY_101_ONP_SP],
                      EEPROM_FAN_ON_DURATION_ADDR);
    modbusRegisters[HW_MY_101_ONP_SP] = 0;
  }
}

void setModbusFanOffDuration()
{
  if (modbusRegisters[HW_MY_101_OFP_SP] != 0)
  {
    MY_101.setInactiveDuration(modbusRegisters[HW_MY_101_OFP_SP]);
    EEPROMWriteUint16(modbusRegisters[HW_MY_101_OFP_SP],
                      EEPROM_FAN_OFF_DURATION_ADDR);
    modbusRegisters[HW_MY_101_OFP_SP] = 0;
  }
}

void setModbusCirculationPumpOnDuration()
{
  if (modbusRegisters[HW_PY_001_ONP_SP] != 0)
  {
    PY_001.setActiveDuration(modbusRegisters[HW_PY_001_ONP_SP]);

    EEPROMWriteUint16(modbusRegisters[HW_PY_001_ONP_SP],
                      EEPROM_CIRCULATION_PUMP_ON_DURATION_ADDR);
    modbusRegisters[HW_PY_001_ONP_SP] = 0;
  }
}

void setModbusCirculationPumpOffDuration()
{
  if (modbusRegisters[HW_PY_001_OFP_SP] != 0)
  {
    PY_001.setInactiveDuration(modbusRegisters[HW_PY_001_OFP_SP]);

    EEPROMWriteUint16(modbusRegisters[HW_PY_001_OFP_SP],
                      EEPROM_CIRCULATION_PUMP_OFF_DURATION_ADDR);
    modbusRegisters[HW_PY_001_OFP_SP] = 0;
  }
}

void setModbusDrainPumpOnDuration()
{
  if (modbusRegisters[HW_PY_002_ONP_SP] != 0)
  {
    //PY_002.setActiveDuration(modbusRegisters[HW_PY_002_ONP_SP]);

    EEPROMWriteUint16(modbusRegisters[HW_PY_002_ONP_SP],
                      EEPROM_DRAIN_PUMP_ON_DURATION_ADDR);
    modbusRegisters[HW_PY_002_ONP_SP] = 0;
  }
}

void setModbusDrainPumpOffDuration()
{
  if (modbusRegisters[HW_PY_002_OFP_SP] != 0)
  {
    //PY_002.setInactiveDuration(modbusRegisters[HW_PY_002_OFP_SP]);

    EEPROMWriteUint16(modbusRegisters[HW_PY_002_OFP_SP],
                      EEPROM_DRAIN_PUMP_OFF_DURATION_ADDR);
    modbusRegisters[HW_PY_002_OFP_SP] = 0;
  }
}


void setConfigToDefaults()
{
  if (getModbusCoilValue(COIL_STATUS_READ_WRITE_OFFSET, CW_KY_001))
  {
    EEPROMWriteDefaultConfig();
    EEPROMLoadConfig();
    writeModbusCoil(COIL_STATUS_READ_WRITE_OFFSET, CW_KY_001, false);
  }
}

bool getModbusCoilValue(uint8_t startAddress, uint8_t bitPos)
{
  return bitRead(modbusRegisters[startAddress + (int)(bitPos / 16)], bitPos % 16);
}

void writeModbusCoil(uint8_t startAddress,
                     uint8_t bitPos,
                     bool value)
{
  bitWrite(modbusRegisters[startAddress + (int)(bitPos / 16)], bitPos % 16,
           value);
}

void processModbusECCommands()
{
  bool status;

  // calibrate low command
  if (getModbusCoilValue(COIL_STATUS_READ_WRITE_OFFSET, CW_AT_002_CL))
  {
    status = AT_002.calibrateLow();
    modbusRegisters[HR_AT_002_CS] = AT_002.getProbeCommandStatus();
    writeModbusCoil(COIL_STATUS_READ_WRITE_OFFSET, CW_AT_002_CL, false);
  }
  else if (getModbusCoilValue(COIL_STATUS_READ_WRITE_OFFSET, CW_AT_002_CH))
  {
    status = AT_002.calibrateHigh();
    modbusRegisters[HR_AT_002_CS] = AT_002.getProbeCommandStatus();
    writeModbusCoil(COIL_STATUS_READ_WRITE_OFFSET, CW_AT_002_CH, false);
  }
  else if (getModbusCoilValue(COIL_STATUS_READ_WRITE_OFFSET, CW_AT_002_CD))
  {
    status = AT_002.calibrateDry();
    modbusRegisters[HR_AT_002_CS] = AT_002.getProbeCommandStatus();
    writeModbusCoil(COIL_STATUS_READ_WRITE_OFFSET, CW_AT_002_CD, false);
  }
  else if (getModbusCoilValue(COIL_STATUS_READ_WRITE_OFFSET, CW_AT_002_CR))
  {
    status = AT_002.calibrateClear();
    modbusRegisters[HR_AT_002_CS] = AT_002.getProbeCommandStatus();
    writeModbusCoil(COIL_STATUS_READ_WRITE_OFFSET, CW_AT_002_CR, false);
  }
  else if (getModbusCoilValue(COIL_STATUS_READ_WRITE_OFFSET, CW_AT_002_CQ))
  {
    int calCount = AT_002.calibrateQuery();
    modbusRegisters[HR_AT_002_CS] = AT_002.getProbeCommandStatus();
    modbusRegisters[HR_AT_002_CC] = calCount;
    writeModbusCoil(COIL_STATUS_READ_WRITE_OFFSET, CW_AT_002_CQ, false);
  }
  else if (getModbusCoilValue(COIL_STATUS_READ_WRITE_OFFSET, CW_AT_002_TC))
  {
    currentIOType = i2c_ec;
    AT_002.setCompensatedTemperature(TT_001T);
    writeModbusCoil(COIL_STATUS_READ_WRITE_OFFSET, CW_AT_002_TC, false);
  }
}

void processModbusPHCommands()
{
  bool status;

  // calibrate low command
  if (getModbusCoilValue(COIL_STATUS_READ_WRITE_OFFSET, CW_AT_001_CL))
  {
    status = AT_001.calibrateLow();
    modbusRegisters[HR_AT_001_CS] = AT_001.getProbeCommandStatus();
    writeModbusCoil(COIL_STATUS_READ_WRITE_OFFSET, CW_AT_001_CL, false);
  }
  else if (getModbusCoilValue(COIL_STATUS_READ_WRITE_OFFSET, CW_AT_001_CH))
  {
    status = AT_001.calibrateHigh();
    modbusRegisters[HR_AT_001_CS] = AT_001.getProbeCommandStatus();
    writeModbusCoil(COIL_STATUS_READ_WRITE_OFFSET, CW_AT_001_CH, false);
  }
  else if (getModbusCoilValue(COIL_STATUS_READ_WRITE_OFFSET, CW_AT_001_CM))
  {
    status = AT_001.calibrateMid();
    modbusRegisters[HR_AT_001_CS] = AT_001.getProbeCommandStatus();
    writeModbusCoil(COIL_STATUS_READ_WRITE_OFFSET, CW_AT_001_CM, false);
  }
  else if (getModbusCoilValue(COIL_STATUS_READ_WRITE_OFFSET, CW_AT_001_CR))
  {
    status = AT_001.calibrateClear();
    modbusRegisters[HR_AT_001_CS] = AT_001.getProbeCommandStatus();
    writeModbusCoil(COIL_STATUS_READ_WRITE_OFFSET, CW_AT_001_CR, false);
  }
  else if (getModbusCoilValue(COIL_STATUS_READ_WRITE_OFFSET, CW_AT_001_CQ))
  {
    int calCount = AT_001.calibrateQuery();
    modbusRegisters[HR_AT_001_CS] = AT_001.getProbeCommandStatus();
    modbusRegisters[HR_AT_001_CC] = calCount;
    writeModbusCoil(COIL_STATUS_READ_WRITE_OFFSET, CW_AT_001_CQ, false);
  }
  else if (getModbusCoilValue(COIL_STATUS_READ_WRITE_OFFSET, CW_AT_001_TC))
  {
    currentIOType = i2c_ec;
    AT_001.setCompensatedTemperature(TT_001T);
    writeModbusCoil(COIL_STATUS_READ_WRITE_OFFSET, CW_AT_001_TC, false);
  }
}

void processModbusXICCommands()
{
  // set and persist pH setpoint
  if (modbusRegisters[HW_AT_001_SP] != 0)
  {
    XIC_001.setSP(((float)(modbusRegisters[HW_AT_001_SP]) / 10.0));

    EEPROMWriteUint16(modbusRegisters[HW_AT_001_SP],
                      EEPROM_PH_SETPOINT_ADDR);
    modbusRegisters[HW_AT_001_SP] = 0;
  }

  // set and persist EC setpoint
  if (modbusRegisters[HW_AT_002_SP] != 0)
  {
    //XIC_002.setSP(modbusRegisters[HW_AT_002_SP]);
    //XIC_003.setSP(modbusRegisters[HW_AT_002_SP]);
    //EEPROMWriteUint16(modbusRegisters[HW_AT_002_SP],
    //                EEPROM_EC_SETPOINT_ADDR);
    modbusRegisters[HW_AT_002_SP] = 0;
  }

  // pH Auto Volume
  if (modbusRegisters[HW_AT_011_AV] != 0)
  {
    XIC_001.setAutoVolume(modbusRegisters[HW_AT_011_AV]);

    EEPROMWriteUint16(modbusRegisters[HW_AT_011_AV],
                      EEPROM_PH_AUTO_VOL_ADDR);
    modbusRegisters[HW_AT_011_AV] = 0;
  }

  // pH Down Auto Interval
  if (modbusRegisters[HW_AT_011_AT] != 0)
  {
    XIC_001.setAutoInterval(modbusRegisters[HW_AT_011_AT]);

    EEPROMWriteUint16(modbusRegisters[HW_AT_011_AT],
                      EEPROM_PH_AUTO_INTERVAL_ADDR);
    modbusRegisters[HW_AT_011_AT] = 0;
  }

  // pH Down Manual Volume
  if (modbusRegisters[HW_AT_011_MV] != 0)
  {
    XIC_001.setManualVolume(modbusRegisters[HW_AT_011_MV]);

    EEPROMWriteUint16(modbusRegisters[HW_AT_011_MV],
                      EEPROM_PH_MANUAL_VOL_ADDR);
    modbusRegisters[HW_AT_011_MV] = 0;
  }

  // N1/N2 auto Volume
  if (modbusRegisters[HW_AT_002_AV] != 0)
  {
    //  XIC_002.setAutoVolume(modbusRegisters[HW_AT_002_AV]);
    //  XIC_003.setAutoVolume(modbusRegisters[HW_AT_002_AV]);
    //  EEPROMWriteUint16(modbusRegisters[HW_AT_002_AV],
    //                    EEPROM_N1N2_AUTO_VOL_ADDR);
    //  modbusRegisters[HW_AT_002_AV] = 0;
  }

  // N1/N2 auto Interval
  if (modbusRegisters[HW_AT_002_AT] != 0)
  {
    //  XIC_002.setAutoInterval(modbusRegisters[HW_AT_002_AT]);
    //  XIC_003.setAutoInterval(modbusRegisters[HW_AT_002_AT]);
    //  EEPROMWriteUint16(modbusRegisters[HW_AT_002_AT],
    //                    EEPROM_N1N2_AUTO_INTERVAL_ADDR);
    //  modbusRegisters[HW_AT_002_AT] = 0;
  }

  // N1/N2 Manual Volume
  if (modbusRegisters[HW_AT_002_MV] != 0)
  {
    //  XIC_002.setManualVolume(modbusRegisters[HW_AT_002_MV]);
    //  XIC_003.setManualVolume(modbusRegisters[HW_AT_002_MV]);

    //  EEPROMWriteUint16(modbusRegisters[HW_AT_002_MV],
    //                    EEPROM_N1N2_MANUAL_VOL_ADDR);
    modbusRegisters[HW_AT_002_MV] = 0;
  }

  // N1/N2 Manual HOA
  if (modbusRegisters[HW_HS_012_HOA_SP] != 0)
  {
    //  XIC_002.setControlMode(modbusRegisters[HW_HS_012_HOA_SP]);
    //  XIC_003.setControlMode(modbusRegisters[HW_HS_012_HOA_SP]);

    //  EEPROMWriteUint16(modbusRegisters[HW_HS_012_HOA_SP],
    //                    EEPROM_N1N2_MODE_ADDR);
    modbusRegisters[HW_HS_012_HOA_SP] = 0;
  }
}

void processModbusCommands()
{
  setModbusTime();

  setModbusGrowingChamberLightsOnTime();
  setModbusGrowingChamberLightsOffTime();
  setModbusSeedingAreaLightsOnTime();
  setModbusSeedingAreaLightsOffTime();

  setModbusFanOnDuration();
  setModbusFanOffDuration();

  setModbusHeatingPadOnTime();
  setModbusHeatingPadOffTime();

  setModbusCirculationPumpOnDuration();
  setModbusCirculationPumpOffDuration();

  // setModbusLightsOff();
  setConfigToDefaults();

  // atlas sensors
  processModbusPHCommands();
  processModbusECCommands();

  // XICs
  processModbusXICCommands();

  // remote HOAs
  processModbusRemoteHOAs();
}

void processModbusRemoteHOAs()
{
  // Circulation Pump remote HOA

  if (HS_105AB.setRemoteState(modbusRegisters[HW_HS_105HOA_SP]))
  {
    EEPROMWriteUint16(modbusRegisters[HW_HS_105HOA_SP],
                      EEPROM_HS_105HOA_ADDR);
  }
  else
    modbusRegisters[HW_HS_105HOA_SP] = HS_105AB.getCurrentState();
  ;

  // Fan remote HOA

  if (HS_101AB.setRemoteState(modbusRegisters[HW_HS_101HOA_SP]))
  {
    EEPROMWriteUint16(modbusRegisters[HW_HS_101HOA_SP],
                      EEPROM_HS_101HOA_ADDR);
  }
  else
    modbusRegisters[HW_HS_101HOA_SP] = HS_101AB.getCurrentState();
  ;

  // seeding area light  remote HOA

  if (HS_102AB.setRemoteState(modbusRegisters[HW_HS_102HOA_SP]))
  {
    EEPROMWriteUint16(modbusRegisters[HW_HS_102HOA_SP], EEPROM_HS_102HOA_ADDR);
  }
  else
    modbusRegisters[HW_HS_102HOA_SP] = HS_102AB.getCurrentState();
  

  // GC area light  remote HOA
  if (HS_103AB.setRemoteState(modbusRegisters[HW_HS_103HOA_SP]))
  {
    EEPROMWriteUint16(modbusRegisters[HW_HS_103HOA_SP],
                      EEPROM_HS_103HOA_ADDR);
  }
  else
    modbusRegisters[HW_HS_103HOA_SP] = HS_103AB.getCurrentState();

  // heating pad
  if (HS_104AB.setRemoteState(modbusRegisters[HW_HS_104HOA_SP]))
  {
    EEPROMWriteUint16(modbusRegisters[HW_HS_104HOA_SP],
                      EEPROM_HS_104HOA_ADDR);
  }
  else
    modbusRegisters[HW_HS_104HOA_SP] = HS_104AB.getCurrentState();

  // ph Down HOA

  if (HS_011.setRemoteState(modbusRegisters[HW_HS_011_HOA_SP]))
  {

    EEPROMWriteUint16(modbusRegisters[HW_HS_011_HOA_SP],
                      EEPROM_PH_DOWN_HOA_ADDR);
  }
  else
    modbusRegisters[HW_HS_011_HOA_SP] = HS_011.getCurrentState();
}

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
  EEPROMWriteAlarmEntry(epoch, EEPROM_GROWING_CHAMBER_ON_TIME_ADDR);
  EEPROMWriteAlarmEntry(epoch, EEPROM_SEEDING_AREA_ON_TIME_ADDR);

  epoch = alarmTimeToUTC(DEFAULT_LIGHTS_OFF_ALARM_TIME);
  EEPROMWriteAlarmEntry(epoch, EEPROM_GROWING_CHAMBER_OFF_TIME_ADDR);
  EEPROMWriteAlarmEntry(epoch, EEPROM_SEEDING_AREA_OFF_TIME_ADDR);

  epoch = alarmTimeToUTC(DEFAULT_HEATING_PAD_ON_TIME);
  EEPROMWriteAlarmEntry(epoch, EEPROM_HEATING_PAD_ON_TIME_ADDR);

  epoch = alarmTimeToUTC(DEFAULT_HEATING_PAD_OFF_TIME);
  EEPROMWriteAlarmEntry(epoch, EEPROM_HEATING_PAD_OFF_TIME_ADDR);

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

void EEPROMWriteAlarmEntry(time_t epoch, uint16_t atAddress)
{
  eeprom_write_block((const void *)&epoch, (void *)atAddress, sizeof(epoch));
}

time_t EEPROMReadEventEntry(uint16_t atAddress)
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

  growingChamberLights.offEpoch = EEPROMReadEventEntry(
      EEPROM_GROWING_CHAMBER_OFF_TIME_ADDR);
  growingChamberLights.onEventOff = doGrowingChamberLightsOff;

  growingChamberLights.onEpoch = EEPROMReadEventEntry(
      EEPROM_GROWING_CHAMBER_ON_TIME_ADDR);
  growingChamberLights.onEventOn = doGrowingChamberLightsOn;

  seedingAreaLights.offEpoch = EEPROMReadEventEntry(
      EEPROM_SEEDING_AREA_OFF_TIME_ADDR);
  seedingAreaLights.onEventOff = doSeedingAreaLightsOff;

  seedingAreaLights.onEpoch = EEPROMReadEventEntry(
      EEPROM_SEEDING_AREA_ON_TIME_ADDR);
  seedingAreaLights.onEventOn = doSeedingAreaLightsOn;

  heatingPad.offEpoch = EEPROMReadEventEntry(
      EEPROM_HEATING_PAD_OFF_TIME_ADDR);
  heatingPad.onEventOff = doHeatingPadOff;

  heatingPad.onEpoch = EEPROMReadEventEntry(
      EEPROM_HEATING_PAD_ON_TIME_ADDR);
  heatingPad.onEventOn = doHeatingPadOn;

  PY_001.setActiveDuration(EEPROMReadUint16(
      EEPROM_CIRCULATION_PUMP_ON_DURATION_ADDR));
  PY_001.setInactiveDuration(EEPROMReadUint16(
      EEPROM_CIRCULATION_PUMP_OFF_DURATION_ADDR));

  //PY_002.setActiveDuration(EEPROMReadUint16(
  //    EEPROM_DRAIN_PUMP_ON_DURATION_ADDR));
  //PY_002.setInactiveDuration(EEPROMReadUint16(
  //    EEPROM_DRAIN_PUMP_OFF_DURATION_ADDR));      

  MY_101.setActiveDuration(EEPROMReadUint16(EEPROM_FAN_ON_DURATION_ADDR));
  MY_101.setInactiveDuration(EEPROMReadUint16(EEPROM_FAN_OFF_DURATION_ADDR));

  XIC_001.setSP(((float)EEPROMReadUint16(EEPROM_PH_SETPOINT_ADDR)) / 10.0);
  XIC_001.setAutoVolume(EEPROMReadUint16(EEPROM_PH_AUTO_VOL_ADDR));
  XIC_001.setAutoInterval(EEPROMReadUint16(EEPROM_PH_AUTO_INTERVAL_ADDR));
  XIC_001.setManualVolume(EEPROMReadUint16(EEPROM_PH_MANUAL_VOL_ADDR));
  XIC_001.setControlMode(EEPROMReadUint16(EEPROM_PH_DOWN_HOA_ADDR));

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

  HS_105AB.setRemoteState(EEPROMReadUint16(EEPROM_HS_105HOA_ADDR));
  HS_101AB.setRemoteState(EEPROMReadUint16(EEPROM_HS_101HOA_ADDR));
  HS_102AB.setRemoteState(EEPROMReadUint16(EEPROM_HS_102HOA_ADDR));
  HS_103AB.setRemoteState(EEPROMReadUint16(EEPROM_HS_103HOA_ADDR));
  HS_104AB.setRemoteState(EEPROMReadUint16(EEPROM_HS_104HOA_ADDR));
  HS_011.setRemoteState(EEPROMReadUint16(EEPROM_PH_DOWN_HOA_ADDR));


}

#if defined(PROCESS_TERMINAL)

// single character message tags from terminal host
#define TIME_HEADER 'T'    // Header tag for Serial2 time sync message
#define DISPLAY_HEADER 'D' // Display header tag
#define DISPLAY_TIME 't'
#define DISPLAY_ALARMS 'a'
#define DISPLAY_DUTY_CYCLE 'd'
#define TIMER_GROWING_CHAMBER_HEADER 'G' // Growing Chamber Alarm Header tag
#define TIMER_SEEDING_AREA_HEADER 'A'    // Seeding Area Alarm Header tag
#define TIMER_ALARM_ON '1'               // Timer Alarm On A14:00:00->turn on
                                         // lights at 4AM
#define TIMER_ALARM_OFF '0'              // Timer Alarm On A023:00:00->turn
                                         // off lighst at 11 PM
#define HELP_HEADER '?'
#define LIGHT_HEADER 'L'                  // Light Header tag
#define LIGHTS_ON '1'                     // Turn lights on L1
#define LIGHTS_OFF '0'                    // Turn lights off L0
#define CIRCULATION_PUMP_HEADER 'C'       // Circulation pump Header tag
#define DRAIN_PUMP_HEADER 'R'       // Drain punmp Header tag
#define FAN_HEADER 'F'                    // Light Header tag
#define TEMPERATURE_COMPENSATE_HEADER 'E' // Atlas Sensor Tempearture
                                          // Compensate
#define CALIBRATE_EC_HEADER 'Y'
#define CALIBRATE_PH_HEADER 'Z'

#define LIGHTS_ON '1'  // Turn lights on L1
#define LIGHTS_OFF '0' // Turn lights off L0

// efine LIGHTS_TOOGLE 't'         // toggle
#define LIGHTS_DUTY_CYCLE 'd'        // Ld60->60% dominant color default mostly
                                     // red 60-90 percent allowed
#define LIGHTS_DUTY_CYCLE_PERIOD 'r' // Lr3600->change the duty cycle between
                                     // 60-90 % every hour
#define LIGHTS_RESET_TO_DEFAULTS 'c'
#define IO_HEADER 'I' // read IO points
#define IO_AMBIENT_TEMP 't'
#define IO_SOIL_MOISTURE 'm'
#define SERIALIZE_HEADER 'S'
#define SERIALIZE_CIRCULATION_PUMP 'c'
#define SERIALIZE_DRAIN_PUMP 'r'
#define SERIALIZE_CIRCULATION_FAN 'f'
#define SERIALIZE_PH 'p'
#define SERIALIZE_EC 'e'
#define SERIALIZE_XIC 'x'
#define SERIALIZE_GC_HOA 'g'

#define CALIBRATE_LOW 'l'   // EC, pH
#define CALIBRATE_MID 'm'   // pH
#define CALIBRATE_HIGH 'h'  // EC, pH
#define CALIBRATE_DRY 'd'   // EC
#define CALIBRATE_CLEAR 'c' // EC,pH
#define CALIBRATE_QUERY 'q' // EC,pH

#define SOFT_HOA_HEADER 'H'
#define SOFT_HOA_GC 'g'
#define CONTROLLER_XIC_001_HEADER 'X'
#define CONTROLLER_XIC_001_PV 'p'      // PV
#define CONTROLLER_XIC_001_SP 's'      // set point
#define CONTROLLER_XIC_001_TM 't'      // trend mode
#define CONTROLLER_XIC_001_CM 'm'      // control mode
#define CONTROLLER_XIC_001_SC 'c'      // stop condition
#define CONTROLLER_XIC_001_DB 'd'      // stop deadband
#define CONTROLLER_XIC_001_REFRESH 'r' // refresh controller

void displayEventControlEntry(char const *who, time_t anEpoch)
{
  char bufferMem[STRLEN("HH:MM:SS")];

  timeToBuffer(anEpoch, bufferMem);

  *debugOutputStream << who << " UTC:" << bufferMem << " epoch:" << anEpoch;

  time_t localAlarmTime = eventTimeToLocal(anEpoch);
  timeToBuffer(localAlarmTime, bufferMem);
  *debugOutputStream << " Local:" << bufferMem << " epoch:" << localAlarmTime << endl;
}

void processDisplayIOMessage()
{
  char c = debugOutputStream->read();

  if (c == IO_AMBIENT_TEMP)
  {
// float h = AT_101.readHumidity(); // allow 1/4 sec to read
// float t = AT_101.readTemperature();
#if defined(NO_PING)
    *debugOutputStream << "Sonar:cm:" << LT_002Raw << endl;
    *debugOutputStream << " LT_002Raw= " << LT_002Raw << endl;
#endif // if defined(NO_PING)
    *debugOutputStream << "Ambient Temperature:" << sensors.getTempC(ambientTemperatureAddress) << "C" << endl;
    *debugOutputStream << "Mixture Temperature:" << sensors.getTempC(mixtureTemperatureAddress) << "C" << endl;
    *debugOutputStream << "HT-101: "
                       << "Rel Humidity:" << AT_101H << " % Temperature:" << AT_101T;
    *debugOutputStream << " C Heat Index " << AT_101HI << endl;

    // AT_101.serialize( debugOutputStream, true);
    // *debugOutputStream << "Abient Temp = " << TE_001.getScaledSample() << "
    // C" <<
    // endl;
    // float hic = AT_101.computeHeatIndex(t, h, false);
    // *debugOutputStream << "DHT-22 Relative Humidity:" << h << " Temperature:"
    // << t <<
    // " C Humidex:" << hic << endl;
  }
  else if (c == IO_SOIL_MOISTURE)
  {
    // *debugOutputStream << "Soil Moisture = " << QE_001.getScaledSample() <<
    // "%" <<
    // endl;
  }
}

void processDisplayMessage()
{
  char c = debugOutputStream->read();

  if (c == DISPLAY_TIME)
  {
    char bufferMem[STRLEN("MM/DD/YYYY HH:MM") + 1];
    TimeChangeRule *tcr; // pointer to the time change rule, use to get the TZ
                         // abbrev
    time_t atime = now();

    atime = usMT.toLocal(atime, &tcr);
    dateTimeToBuffer(atime, bufferMem);

    *debugOutputStream << bufferMem << endl;
     //refreshModbusRegisters();
    //*debugOutputStream << modbusRegisters[HR_QT_001_CV]  << " " << modbusRegisters[HR_QT_001_CV+1] <<endl;
  }
  else if (c == DISPLAY_ALARMS)
  {
    *debugOutputStream << endl;
    displayEventControlEntry("...GC Off", growingChamberLights.offEpoch);
    displayEventControlEntry("...GC On", growingChamberLights.onEpoch);
    displayEventControlEntry("...SA Off", seedingAreaLights.offEpoch);
    displayEventControlEntry("...SA On", seedingAreaLights.onEpoch);
    displayEventControlEntry("...Heating On", heatingPad.onEpoch);
    displayEventControlEntry("...Heating Off", heatingPad.offEpoch);

    // displayAlarm("...Reset Midnight", onMidnight);

    // *debugOutputStream << "Duty Cycle Period = " <<
    // dutyCycleChangeAlarm.epoch << " s
    // id=" << dutyCycleChangeAlarm.id << endl;
  }
  else if (c == DISPLAY_DUTY_CYCLE)
  {
    // *debugOutputStream << "Duty Cycle = " << plantStrip.getDutyCycle() <<
    // endl;
  }
}

void processLightsMessage()
{
  char c = debugOutputStream->read();

  switch (c)
  {
  case LIGHTS_ON:
    doGrowingChamberLightsOn();
    break;

  case LIGHTS_OFF:
    doGrowingChamberLightsOff();
    break;

  case LIGHTS_DUTY_CYCLE:

    // plantStrip.setDutyCycle( debugOutputStream->parseInt());
    break;

  case LIGHTS_RESET_TO_DEFAULTS:
    EEPROMWriteDefaultConfig();
    EEPROMLoadConfig();
    *debugOutputStream << F("Settings set to Defaults") << endl;
    break;

  default:
    break;
  }
}

void processLightEntryMessage(time_t *anEpoch, uint16_t eepromAddr)
{
  // Alarm.free(aAlarmEntry->id);
  // uint16_t shour = debugOutputStream -> parseInt();
  // uint16_t sminute = debugOutputStream -> parseInt();
  // uint16_t ssecond = debugOutputStream -> parseInt();
  time_t localAlarmTime = debugOutputStream->parseInt();

  // *anEpoch = alarmTimeToUTC(AlarmHMS(shour, sminute, ssecond));
  *anEpoch = alarmTimeToUTC(localAlarmTime);

  // Serial << "alarm epoch " << aAlarmEntry->epoch << " alarm ID " <<
  // aAlarmEntry->id <<endl;
  EEPROMWriteAlarmEntry(*anEpoch, eepromAddr);
}

void processGCAlarmMessage()
{
  char c = debugOutputStream->read();

  if (c == TIMER_ALARM_OFF)
  {
    processLightEntryMessage(&growingChamberLights.offEpoch,
                             EEPROM_GROWING_CHAMBER_OFF_TIME_ADDR);
    displayEventControlEntry("GC OFF", growingChamberLights.offEpoch);
  }
  else if (c == TIMER_ALARM_ON)
  {
    processLightEntryMessage(&growingChamberLights.onEpoch,
                             EEPROM_GROWING_CHAMBER_ON_TIME_ADDR);
    displayEventControlEntry("GC On", growingChamberLights.onEpoch);
  }
}

void processSAAlarmMessage()
{
  char c = debugOutputStream->read();

  if (c == TIMER_ALARM_OFF)
  {
    processLightEntryMessage(&seedingAreaLights.offEpoch,
                             EEPROM_SEEDING_AREA_OFF_TIME_ADDR);
    displayEventControlEntry("SA OFF", seedingAreaLights.offEpoch);
  }
  else if (c == TIMER_ALARM_ON)
  {
    processLightEntryMessage(&seedingAreaLights.onEpoch,
                             EEPROM_SEEDING_AREA_ON_TIME_ADDR);
    displayEventControlEntry("SA ON", seedingAreaLights.onEpoch);
  }
}

void processCirculationPumpDurations()
{
  char c = debugOutputStream->read();
  uint16_t duration;

  duration = (uint16_t)debugOutputStream->parseInt();

  if (c == TIMER_ALARM_ON)
  {
    PY_001.setActiveDuration(duration);
    EEPROMWriteUint16(duration, EEPROM_CIRCULATION_PUMP_ON_DURATION_ADDR);
  }
  else
  {
    PY_001.setInactiveDuration(duration);
    EEPROMWriteUint16(duration, EEPROM_CIRCULATION_PUMP_OFF_DURATION_ADDR);
  }
}

void processDrainPumpDurations()
{
  char c = debugOutputStream->read();
  uint16_t duration;

  duration = (uint16_t)debugOutputStream->parseInt();

  if (c == TIMER_ALARM_ON)
  {
    //PY_002.setActiveDuration(duration);
    EEPROMWriteUint16(duration, EEPROM_DRAIN_PUMP_ON_DURATION_ADDR);
  }
  else
  {
    //PY_002.setInactiveDuration(duration);
    EEPROMWriteUint16(duration, EEPROM_DRAIN_PUMP_OFF_DURATION_ADDR);
  }
}


void processFanDurations()
{
  char c = debugOutputStream->read();
  uint16_t duration;

  duration = (uint16_t)debugOutputStream->parseInt();

  if (c == TIMER_ALARM_ON)
  {
    MY_101.setActiveDuration(duration);
    EEPROMWriteUint16(duration, EEPROM_FAN_ON_DURATION_ADDR);
  }
  else
  {
    MY_101.setInactiveDuration(duration);
    EEPROMWriteUint16(duration, EEPROM_FAN_OFF_DURATION_ADDR);
  }
}

void showCommands()
{
  *debugOutputStream << "-------------------------------------------------------------------" << endl;
  *debugOutputStream << F("Dt - Display Date/Time") << endl;
  *debugOutputStream << F("Da - Display Alarms") << endl;
  *debugOutputStream << F("T9999999999 - Set time using UNIX Epoch numner") << endl;
  *debugOutputStream << F("G1HH:MM:SS - GC on") << endl;
  *debugOutputStream << F("G0HH:MM:SS  - GC Off ") << endl;
  *debugOutputStream << F("A1HH:MM:SS - SA on") << endl;
  *debugOutputStream << F("A0HH:MM:SS  - SA Off ") << endl;
  *debugOutputStream << F("Lc  - reset/clear settings to defaults ") << endl;
  *debugOutputStream << F("It  - display 1 wire temps and humidity, sonar") << endl;
  *debugOutputStream << F("Im  - display Soil Moisture") << endl;
  *debugOutputStream << F("?? - Display commands") << endl;
  *debugOutputStream << F("Sc - Serialize Circulation Pump") << endl;
  *debugOutputStream << F("Sr - Serialize Drain Pump") << endl;
  *debugOutputStream << F("Sf - Serialize Fan") << endl;
  *debugOutputStream << F("Sp - Serialize pH") << endl;
  *debugOutputStream << F("Se - Serialize EC") << endl;
  *debugOutputStream << F("Sx - Serialize XIC") << endl;
  *debugOutputStream << F("Sg - Serialize GC HOA") << endl;
  *debugOutputStream << F("Ep99.99 - Temperature Compensate pH") << endl;
  *debugOutputStream << F("Zl  - Calibrate pH Low") << endl;
  *debugOutputStream << F("Zm - Calibrate pH mid") << endl;
  *debugOutputStream << F("Zh - Calibrate pH High") << endl;
  *debugOutputStream << F("Zc - Clear pH Calibration") << endl;
  *debugOutputStream << F(
                            "Zq - Return calibration state (0=none, 1=1point, 2=2 point, 3=3point")
                     << endl;
  *debugOutputStream << F("Ee99.99 - Temperature Compensate EC") << endl;
  *debugOutputStream << F("Yl - Calibrate EC Low") << endl;
  *debugOutputStream << F("Yh - Calibrate EC High") << endl;
  *debugOutputStream << F("Yd - Calibrate EC Dry") << endl;
  *debugOutputStream << F("Yc - Clear EC Calibration") << endl;
  *debugOutputStream << F(
                            "Yq - Return calibration state (0=none, 1=1point, 2=2 point")
                     << endl;
  *debugOutputStream << F("C19999999 - Circulation Pump On Duration in s") << endl;
  *debugOutputStream << F("CO9999999 - Circulation Pump Off Duration in s") << endl;
  *debugOutputStream << F("R19999999 - Drain Pump On Duration in s") << endl;
  *debugOutputStream << F("RO9999999 - Drain Pump Off Duration in s") << endl;
  *debugOutputStream << F("F19999999 - Fan On Duration in s") << endl;
  *debugOutputStream << F("FO9999999 - Fan Off Duration in s") << endl;

  *debugOutputStream << F("Xs9999999 - set XIC SP") << endl;
  *debugOutputStream << F("Xp9999999 - set XIC PV") << endl;
  *debugOutputStream << F("Xc9999999 - set XIC StopCond") << endl;
  *debugOutputStream << F("Xm1|2|3   - set XIC MODE 1=OFF, 2=HAND, 3=AUTO") << endl;
  *debugOutputStream << F(
                            "Xt0|1   - set XIC trend type 0=trendinup, 1=trendingdown")
                     << endl;
  *debugOutputStream << F("Xr  - Refresh Controller") << endl;
  *debugOutputStream << F("Hg1|2|3   - set GC Soft HOA 1=OFF, 2=HAND, 3=AUTO") << endl;
  *debugOutputStream << "------------------------------------------------------------------" << endl;
}

void processTimeSetMessage()
{
  uint32_t pctime;

  pctime = debugOutputStream->parseInt();

  // *debugOutputStream << pctime << endl;
  if (pctime >= DEFAULT_TIME)

  // check the integer is a valid time (greater than Jan 1 2013)
  {
    RTC.set(pctime); // set the RTC and the system time to the received
                     // value
    setTime(pctime); // Sync Arduino clock to the time received on the

    // Serial2 port
    char bufferMem[STRLEN("MM/DD/YYYY HH:MM") + 1];
    TimeChangeRule *tcr; // pointer to the time change rule, use to get the TZ
                         // abbrev
    time_t atime = now();
    atime = usMT.toLocal(atime, &tcr);
    dateTimeToBuffer(atime, bufferMem);
    *debugOutputStream << bufferMem << endl;
  }
}

void processSerializeMessage()
{
  char c = debugOutputStream->read();

  switch (c)
  {
  case SERIALIZE_CIRCULATION_PUMP:
    PY_001.serialize(debugOutputStream, true);
    break;
  case SERIALIZE_DRAIN_PUMP:
    PY_002.serialize(debugOutputStream, true);
    break;

  case SERIALIZE_CIRCULATION_FAN:
    MY_101.serialize(debugOutputStream, true);
    HS_101AB.serialize(debugOutputStream, true);
    break;
  case SERIALIZE_GC_HOA:
    *debugOutputStream << F("GC HOA:HS_103AB") << endl;
    HS_103AB.serialize(debugOutputStream, true);
    *debugOutputStream << F("GC LIGHT:DY_103") << endl;
    DY_103.serialize(debugOutputStream, true);
    break;
  case SERIALIZE_PH:
    AT_001.serialize(debugOutputStream, true);
    break;

  case SERIALIZE_EC:
    AT_002.serialize(debugOutputStream, true);
    break;

  case LIGHTS_DUTY_CYCLE:

    // plantStrip.setDutyCycle( debugOutputStream->parseInt());
    break;

  case SERIALIZE_XIC:
    XIC_001.serialize(debugOutputStream, true);

    break;

  default:
    break;
  }
}

void processSoftHOAMessage()
{
  char c = debugOutputStream->read();
  int x;

  switch (c)
  {

  case SOFT_HOA_GC:
    x = debugOutputStream->parseInt();

    if (x >= 1 && x <= 3)
    {
      *debugOutputStream << F("GC HOA:HS_103AB Before") << endl;
      HS_103AB.serialize(debugOutputStream, true);

      if (HS_103AB.setRemoteState(x))
      {
        //HS_102AB.setRemoteState(x);
         EEPROMWriteUint16(x,
                          EEPROM_HS_102HOA_ADDR);
        *debugOutputStream << F("GC SOFT HOA:Writing") << endl;
        EEPROMWriteUint16(x,
                          EEPROM_HS_103HOA_ADDR);

        *debugOutputStream << "105 mem:" << EEPROMReadUint16(EEPROM_HS_105HOA_ADDR) << " address:" << EEPROM_HS_105HOA_ADDR << endl;

        *debugOutputStream << "101 mem:" << EEPROMReadUint16(EEPROM_HS_101HOA_ADDR) << " address:" << EEPROM_HS_101HOA_ADDR << endl;
        *debugOutputStream << "102 mem:" << EEPROMReadUint16(EEPROM_HS_102HOA_ADDR) << " address:" << EEPROM_HS_102HOA_ADDR << endl;
        *debugOutputStream << "103 mem:" << EEPROMReadUint16(EEPROM_HS_103HOA_ADDR) << " address:" << EEPROM_HS_103HOA_ADDR << endl;
        *debugOutputStream << "modbusRegisters[HR_MY_101_ONP_CV]:" << modbusRegisters[HR_MY_101_ONP_CV] << endl;
      }

      *debugOutputStream << F("GC HOA:HS_103AB After") << endl;
      HS_103AB.serialize(debugOutputStream, true);
      *debugOutputStream << F("GC LIGHT:DY_103") << endl;
      DY_103.serialize(debugOutputStream, true);
    }
    else
    {
      *debugOutputStream << F("Invalid GC Soft HOA") << endl;
    }

    break;

  default:
    *debugOutputStream << "invalid SOFT HOA command" << endl;
  }
}

void processControllerMessage()
{
  char c = debugOutputStream->read();
  int x;
  float y;

  switch (c)
  {
  case CONTROLLER_XIC_001_PV:
    y = debugOutputStream->parseFloat();
    XIC_001.setPV(y);
    break;

  case CONTROLLER_XIC_001_SP:
    y = debugOutputStream->parseFloat();
    XIC_001.setSP(y);
    break;

  case CONTROLLER_XIC_001_SC:
    y = debugOutputStream->parseFloat();
    XIC_001.setStopSetpoint(y);
    break;

  case CONTROLLER_XIC_001_DB:
    y = debugOutputStream->parseFloat();
    XIC_001.setStopDeadband(y);
    break;

  case CONTROLLER_XIC_001_REFRESH:
    XIC_001.refresh();
    XIC_001.serialize(debugOutputStream, true);
    break;

  case CONTROLLER_XIC_001_CM:
    x = debugOutputStream->parseInt();
    XIC_001.serialize(debugOutputStream, true);

    if (x == 1)
      XIC_001.setControlMode(DA_NutrientController::Hand);
    else if (x == 2)
      XIC_001.setControlMode(DA_NutrientController::Off);
    else
      XIC_001.setControlMode(DA_NutrientController::Auto);

    break;

  case CONTROLLER_XIC_001_TM:

    x = debugOutputStream->parseInt();

    if (x == 0)
      XIC_001.setTrendingMode(DA_NutrientController::RISINGTREND);
    else
      XIC_001.setTrendingMode(DA_NutrientController::FALLINGTREND);

    break;

  default:
    *debugOutputStream << "invalid controller command" << endl;
  }
}

void processTemperatureCompensation()
{
  char c = debugOutputStream->read();
  float temperature;

  temperature = debugOutputStream->parseFloat();

  if (c == SERIALIZE_PH)
  {
    // AT_002.pause();
    currentIOType = i2c_ph;
    AT_001.setCompensatedTemperature(temperature);
  }
  else
  {
    // AT_001.pause();
    currentIOType = i2c_ec;
    AT_002.setCompensatedTemperature(temperature);
  }
}

void processCalibrateMessage(IO_TYPE aIO_Type)
{
  char calCmd = debugOutputStream->read();

  switch (calCmd)
  {
  case CALIBRATE_LOW:

    if (aIO_Type ==
        i2c_ec)
      *debugOutputStream << "EC Cal Low:" << AT_002.calibrateLow() << endl;
    else
      *debugOutputStream << "pH Cal Low:" << AT_001.calibrateLow() << endl;
    break;

  case CALIBRATE_MID:

    if (aIO_Type ==
        i2c_ec)
      *debugOutputStream << "EC Cal Mid does not exist:" << endl;
    else
      *debugOutputStream << "pH Cal Mid:" << AT_001.calibrateMid() << endl;
    break;

  case CALIBRATE_HIGH:

    if (aIO_Type ==
        i2c_ec)
      *debugOutputStream << "EC Cal High:" << AT_002.calibrateHigh() << endl;
    else
      *debugOutputStream << "pH Cal High:" << AT_001.calibrateHigh() << endl;

    break;

  case CALIBRATE_DRY:

    if (aIO_Type ==
        i2c_ec)
      *debugOutputStream << "EC Cal Dry:" << AT_002.calibrateDry() << endl;
    else
      *debugOutputStream << "pH Cal Dry does not exist." << endl;
    break;

  case CALIBRATE_CLEAR:

    if (aIO_Type ==
        i2c_ec)
      *debugOutputStream << "EC Cal Clear:" << AT_002.calibrateClear() << endl;
    else
      *debugOutputStream << "pH Cal Clear:" << AT_001.calibrateClear() << endl;
    break;
    break;

  case CALIBRATE_QUERY:

    if (aIO_Type ==
        i2c_ec)
      *debugOutputStream << "EC Points Calibrated:" << AT_002.calibrateQuery() << endl;
    else
      *debugOutputStream << "pH Points Calibrated:" << AT_001.calibrateQuery() << endl;
    break;

  default:
    *debugOutputStream << "Invalid command" << endl;
    break;
  }
}

void processTerminalCommands()
{
  if (debugOutputStream->available() > 1)
  {
    // wait for at least two characters
    char c = debugOutputStream->read();

    // *debugOutputStream << c << endl;
    if (c == TIME_HEADER)
    {
      processTimeSetMessage();
    }
    else if (c == DISPLAY_HEADER)
    {
      processDisplayMessage();
    }
    else if (c == LIGHT_HEADER)
    {
      processLightsMessage();
    }
    else if (c == HELP_HEADER)
    {
      debugOutputStream->read();
      showCommands();
    }
    else if (c == TIMER_GROWING_CHAMBER_HEADER)
    {
      processGCAlarmMessage();
    }
    else if (c == TIMER_SEEDING_AREA_HEADER)
    {
      processSAAlarmMessage();
    }
    else if (c == IO_HEADER)
    {
      processDisplayIOMessage();
    }
    else if (c == SERIALIZE_HEADER)
    {
      processSerializeMessage();
    }
    else if (c == CIRCULATION_PUMP_HEADER)
    {
      processCirculationPumpDurations();
    }
    else if (c == DRAIN_PUMP_HEADER)
    {
      processDrainPumpDurations();
    }
    else if (c == FAN_HEADER)
    {
      processFanDurations();
    }
    else if (c == CONTROLLER_XIC_001_HEADER)
    {
      processControllerMessage();
    }
    else if (c == SOFT_HOA_HEADER)
    {
      processSoftHOAMessage();
    }
    else if (c == CALIBRATE_EC_HEADER)
      processCalibrateMessage(i2c_ec);
    else if (c == CALIBRATE_PH_HEADER)
      processCalibrateMessage(i2c_ph);
    else if (c == TEMPERATURE_COMPENSATE_HEADER)
      processTemperatureCompensation();
  }
}

#endif // ifdef PROCESS_TERMINAL
