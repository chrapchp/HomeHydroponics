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
#include <avr/wdt.h>
#include <Streaming.h>
#include <Adafruit_Sensor.h>
#include <DHT.h> // DHT-22 humidity sensor
//#include <MemoryFree.h>

#if defined(ENABLE_FLOWMETER)
#include <DA_Flowmeter.h>
#endif

#include <DA_Discreteinput.h>
#include <DA_DiscreteOutput.h>
#include <DA_DiscreteOutputTmr.h>
#include <DA_HOASwitch.h>
#include <DA_AtlasPH.h>
#include <DA_AtlasEC.h>
//#include <DA_PeristalticPump.h>
#include <DA_NonBlockingDelay.h>

#include "hydroponics.h"
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
#ifdef ENABLE_FLOWMETER
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
#endif

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
#ifdef ENABLE_FLOWMETER
void doOnCalcFlowRate();
#endif

void doOnTemperatureCompensate();
void EEPROMLoadConfig();
time_t EEPROMReadTimeEntry(uint16_t atAddress);
void EEPROMWriteTimeEntry(time_t epoch,
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
#if defined(ENABLE_MODBUS_IP) || defined(ENABLE_MODBUS_SERIAL)
void refreshModbusRegisters();
#endif
void setConfigToDefaults();
void setCirculationPumpOffDuration();
void setCirculationPumpOnDuration();
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
void processMiscCommands();

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
void on_watchdogTimerEvent(bool state);

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

float TT_001T = NAN;
DeviceAddress mixtureTemperatureAddress =
    {
        0x28, 0xFF, 0xF4, 0xF6, 0x84, 0x16, 0x05, 0x0C};

#define WIRE_BUS_PIN 6 // pin
#define ONE_TEMPERATURE_PRECISION 9
OneWire oneWire(WIRE_BUS_PIN);
DallasTemperature sensors(&oneWire);

// Analog Inputs
//
// NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
#define NUTRIENT_TANK_HEIGHT 45    // height of nutrient tank in cm
#define NUTRIENT_TANK_AIR_GAP 16.5 // space between sensor and max water level in cm
#define NUTRIENT_DEAD_BAND 5.0     // % change allowed between samples as sensor  bounces because of noise issue
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
// DA_DiscreteOutputTmr PY_002 = DA_DiscreteOutputTmr(38,
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

// DA_NutrientController // XIC_001 = DA_NutrientController(45, HIGH, 6.5,
//                                                       DA_NutrientController::RISINGTREND);

// Nutrient 1 Pump Controller
// DA_NutrientController XIC_002 = DA_NutrientController(48, HIGH,

// Nutrient 2 Pump Controller
// DA_NutrientController XIC_003 = DA_NutrientController(47,

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
#ifdef ENABLE_FLOWMETER
DA_NonBlockingDelay KI_002 = DA_NonBlockingDelay(FLOW_CALC_PERIOD_MS,
                                                 doOnCalcFlowRate);
#endif
DA_NonBlockingDelay KI_003 = DA_NonBlockingDelay(TEMPERATURE_COMPENSATE_CYCLE,
                                                 doOnTemperatureCompensate);

#ifdef ENABLE_MQTT
DA_NonBlockingDelay KI_004 = DA_NonBlockingDelay(MQTT_CONNECT_CHECK_RATE,
                                                 doOnMQQTCheck);
#endif
void setup()
{

  wdt_disable();
  delay(3000);
  wdt_enable(WDTO_8S);
  //   Serial.begin(9600);
  // debugOutputStream = &Serial;
#if defined(PROCESS_TERMINAL)
  // Serial3.begin(9600);
  // Stream *debugOutputStream = &Serial3;
  Serial.begin(9600);
  debugOutputStream = &Serial;
#endif // ifdef PROCESS_TERMINAL

#if defined(PROCESS_MODBUS)
#if defined(MODBUS_IP)
  // Ethernet.init(53);
  // pinMode(53, OUTPUT);
  Ethernet.begin(defaultMAC, defaultIP, defaultGateway, defaultSubnet);
#else
  slave.begin(MB_SERIAL_BAUD);
#endif
#endif // ifdef PROCESS_MODBUS

#if defined(ENABLE_MQTT)
  // mqttClient.setBufferSize(380);
  mqttClient.setClient(ethernetClient);
  mqttClient.setServer(defaultMQTT, 1883);
  mqttClient.setCallback(onMQTTMessage);
  mqttClient.setBufferSize(MQTT_TX_SZ);
  Ethernet.begin(defaultMAC, defaultIP, defaultGateway, defaultSubnet);
  delay(1500);
  initIOData();

#else

#endif
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

  // HS_011.setOnStateChangeDetect(&on_pHDownProcess);

  HS_201.setOnStateChangeDetect(&on_AeratorProcess);

  // 1-wire
  sensors.begin();
  initOneWire();

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

  MY_101.start(DA_DiscreteOutputTmr::Continuous);
  PY_001.start(DA_DiscreteOutputTmr::Continuous);
  // PY_002.start(DA_DiscreteOutputTmr::Continuous);
  HY_001.setOnTimeEvent(&on_watchdogTimerEvent);
  HY_001.start(DA_DiscreteOutputTmr::Continuous);
}

#ifdef ENABLE_MQTT
void doOnMQQTCheck()
{

  if (!mqttClient.connected())
  {
    mqttReconnect();

#ifdef PROCESS_TERMINAL

    if (!mqttClient.connected())
      *debugOutputStream << "doOnMQQTCheck: Sleep" << endl;
#endif
  }
  else
  {

    mqttSendDiscreteIO(mqttStatusSendTopic);
    mqttSendSetpointPV(mqttStatusSendTopic);
    mqttSendHOA(mqttStatusSendTopic);

    mqttSendDiscreteIO(mqttEUSendTopic);
    mqttSendEU(mqttEUSendTopic);
  }
}
#endif
void loop()
{

#if defined(PROCESS_TERMINAL)
  processTerminalCommands();
#endif // ifdef PROCESS_TERMINAL

#if defined(ENABLE_MQTT)
  if (mqttClient.connected())
    mqttClient.loop();

#endif
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
#ifdef ENABLE_MQTT
  KI_004.refresh();
#endif
  KI_001.refresh();
#ifdef ENABLE_FLOWMETER
  KI_002.refresh();
#endif
  KI_003.refresh();

#ifndef PROCESS_TERMINAL
  // XIC_001.setPV(AT_001.getSample());
#endif
  //  // XIC_001.refresh();
}

#ifdef ENABLE_FLOWMETER
void onFT_002_PulseIn()
{
  FT_002.handleFlowDetection();
}

void onFT_003_PulseIn()
{
  FT_003.handleFlowDetection();
}
#endif

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

void on_watchdogTimerEvent(bool state)
{
  wdt_reset();
}

void on_DrainPump_Process(bool state, int aPin)
{
#if defined(PROCESS_TERMINAL_VERBOSE)
  *debugOutputStream << "on_DrainPump_Process HS_001" << endl;
  HS_001.serialize(debugOutputStream, true);
#endif // ifdef PROCESS_TERMINAL_VERBOSE

  if (HS_001.getSample() == LOW)
  {
    // PY_002.enable();
    // PY_002.reset();
    PY_002.forceActive();
  }
  else
  {
    // PY_002.pauseTimer();
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
#if defined(PROCESS_TERMINAL_VERBOSE)
    *debugOutputStream << "Hand" << endl;
#endif
    PY_001.pauseTimer();
    PY_001.disable();
    PY_001.forceActive(); // force the pump on

    break;

  case DA_HOASwitch::Off:
#if defined(PROCESS_TERMINAL_VERBOSE)
    *debugOutputStream << "Off" << endl;
#endif
    PY_001.pauseTimer();
    PY_001.disable();

    break;

  case DA_HOASwitch::Auto:
#if defined(PROCESS_TERMINAL_VERBOSE)
    *debugOutputStream << "Auto" << endl;
#endif
    PY_001.enable();
    PY_001.resumeTimer();

    // PY_001.restart();
    // PY_001.resume();
    break;

  default:
#if defined(PROCESS_TERMINAL_VERBOSE)
    *debugOutputStream << "UNKNOWN HOA" << endl;
#endif
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
  //// XIC_001.setControlMode(HS_011.getCurrentState());
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

#ifdef ENABLE_FLOWMETER
void doOnCalcFlowRate()
{
  DISABLE_FT002_SENSOR_INTERRUPTS;
  FT_002.end();
  FT_002.begin();
  DISABLE_FT003_SENSOR_INTERRUPTS;
  FT_003.end();
  FT_003.begin();
#ifdef ENABLE_CO2
  AT_102MedianFilter.add(AT_102Raw);
#endif
  ENABLE_FT002_SENSOR_INTERRUPTS;
  ENABLE_FT003_SENSOR_INTERRUPTS;
}
#endif

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

  if (HS_104AB.getCurrentState() == DA_HOASwitch::Auto)
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

/*******************************************************************************************************
** START MQTT related functions
********************************************************************************************************/

boolean mqttReconnect()
{

  bool retVal = false;
  if (!mqttClient.connected())
  {
#ifdef PROCESS_TERMINAL
    *debugOutputStream << "Attempting MQTT connection..." << endl;
#endif

    //     // Attempt to connect
    if (mqttClient.connect(deviceID))
    //  if (mqttClient.connect("arduinoClient"))
    {
#ifdef PROCESS_TERMINAL
      *debugOutputStream << "Connected.." << endl;
#endif
      // mqttClient.publish(mqttStatusSendTopic, "hello world");
      mqttClient.subscribe(hostCommandTopic);
      retVal = true;
    }
    else
    {
#ifdef PROCESS_TERMINAL
      *debugOutputStream << "MQTT Connect failed:(" << mqttClient.state() << ")" << endl;
#endif
    }
  }
  return retVal;
}

void onMQTTMessage(char *topic, byte *payload, unsigned int length)
{
  StaticJsonDocument<MQTT_MSG_IN_SZ> mqttMsgIn;
  DeserializationError err = deserializeJson(mqttMsgIn, payload);

#if defined(PROCESS_TERMINAL)
  *debugOutputStream << "Message On Topic arrived [" << topic << "] Msg = ";
  serializeJson(mqttMsgIn, *debugOutputStream);
  *debugOutputStream << endl;
#endif
  for (JsonPair p : mqttMsgIn.as<JsonObject>())
  {
    JsonString key = p.key();
    uint32_t j = mqttMsgIn[key.c_str()];
    uint32_t hdlr = IOData[key.c_str()]["handlerID"];
    if (hdlr != 0)
    {
      (*setHandlers[hdlr])(j);
     
      // IOData[pvkey.c_str()]["dirty"] = true;
      // IOData[pvkey.c_str()]["value"] = j;
    }
#ifdef PROCESS_TERMINAL
    else
    {
      *debugOutputStream << "onMQTTMessage()-key:" << key.c_str() << " Not found" << endl;
    }
#endif
  }
}

void initIOData()
{
  // set Circulation pump on/off durations

  IOData["PY-001-OFP-SP"]["handlerID"] = PY_001_OFP_SP_HDLR;
  // IOData["PY-001-OFP-SPF"]["value"] = -1;
  // IOData["PY-001-OFP-SPF"]["dirty"] = false;

  IOData["PY-001-ONP-SP"]["handlerID"] = PY_001_ONP_SP_HDLR;
  // IOData["PY-001-ONP-SPF"]["value"] = -1;
  // IOData["PY-001-ONP-SPF"]["dirty"] = false;

  IOData["HS-001-HOA-SP"]["handlerID"] = PY_001_HOA_HDLR;
  // IOData["HS-001-HOA-SPF"]["value"] = -1;
  // IOData["HS-001-HOA-SPF"]["dirty"] = false;

  // IOData["PY-001"]["value"] = -1;
  // IOData["PY-001"]["dirty"] = false;

  // set Fan on/off durations
  IOData["MY-101-OFP-SP"]["handlerID"] = MY_101_OFP_SP_HDLR;
  // IOData["MY-101-OFP-SPF"]["value"] = -1;
  // IOData["MY-101-OFP-SPF"]["dirty"] = false;

  IOData["MY-101-ONP-SP"]["handlerID"] = MY_101_ONP_SP_HDLR;
  // IOData["MY-101-ONP-SPF"]["value"] = -1;
  // IOData["MY-101-ONP-SPF"]["dirty"] = false;

  IOData["HS-101-HOA-SP"]["handlerID"] = MY_101_HOA_HDLR;
  // IOData["HS-101-HOA-SPF"]["value"] = -1;
  // IOData["HS-101-HOA-SPF"]["dirty"] = false;

  // IOData["MY-101"]["value"] = -1;
  // IOData["MY-101"]["dirty"] = false;

  // set heating pad on/off time

  IOData["DY-104-OFT-SP"]["handlerID"] = DY_104_OFT_SP_HDLR;
  // IOData["DY-104-OFT-SPF"]["value"] = -1;
  // IOData["DY-104-OFT-SPF"]["dirty"] = false;

  IOData["DY-104-ONT-SP"]["handlerID"] = DY_104_ONT_SP_HDLR;
  // IOData["DY-104-ONT-SPF"]["value"] = -1;
  // IOData["DY-104-ONT-SPF"]["dirty"] = false;

  IOData["HS-104-HOA-SP"]["handlerID"] = DY_104_HOA_HDLR;
  // IOData["HS-104-HOA-SPF"]["value"] = -1;
  // IOData["HS-104-HOA-SPF"]["dirty"] = false;

  // IOData["DY-104"]["value"] = -1;
  // IOData["DY-104"]["dirty"] = false;

  // set growing chamber  on/off time

  IOData["DY-103-OFT-SP"]["handlerID"] = DY_103_OFT_SP_HDLR;
  // IOData["DY-103-OFT-SPF"]["value"] = -1;
  // IOData["DY-103-OFT-SPF"]["dirty"] = false;

  IOData["DY-103-ONT-SP"]["handlerID"] = DY_103_ONT_SP_HDLR;
  // IOData["DY-103-ONT-SPF"]["value"] = -1;
  // IOData["DY-103-ONT-SPF"]["dirty"] = false;

  IOData["HS-103-HOA-SP"]["handlerID"] = DY_103_HOA_HDLR;
  // IOData["HS-103-HOA-SPF"]["value"] = -1;
  // IOData["HS-103-HOA-SPF"]["dirty"] = false;

  // IOData["DY-103"]["value"] = -1;
  // IOData["DY-103"]["dirty"] = false;

  // set seeding area on/off time

  IOData["DY-102-OFT-SP"]["handlerID"] = DY_102_OFT_SP_HDLR;
  // IOData["DY-102-OFT-SPF"]["value"] = -1;
  // IOData["DY-102-OFT-SPF"]["dirty"] = false;

  IOData["DY-102-ONT-SP"]["handlerID"] = DY_102_ONT_SP_HDLR;
  // IOData["DY-102-ONT-SPF"]["value"] = -1;
  // IOData["DY-102-ONT-SPF"]["dirty"] = false;

  IOData["HS-102-HOA-SP"]["handlerID"] = DY_102_HOA_HDLR;
  // IOData["HS-102-HOA-SPF"]["value"] = -1;
  // IOData["HS-102-HOA-SPF"]["dirty"] = false;

  // IOData["DY-102"]["value"] = -1;
  // IOData["DY-102"]["dirty"] = false;

  // Board Time

  IOData["QT-001-SP"]["handlerID"] = QT_001_SP_HDLR;
  // IOData["QT-001-SPF"]["value"] = -1;
  // IOData["QT-001-SPF"]["dirty"] = false;

  IOData["PH_CAL"]["handlerID"] = PH_CAL_HDLR;

  IOData["EC_CAL"]["handlerID"] = EC_CAL_HDLR;

#ifdef PROCESS_TERMINAL
  serializeJson(IOData, *debugOutputStream);
  *debugOutputStream << endl;

  for (JsonPair kv : IOData.as<JsonObject>())
  {
    Serial.println(kv.key().c_str());
    JsonObject x = kv.value().as<JsonObject>();
    char *keyExists = x["dirty"];
    serializeJson(x, *debugOutputStream);
    *debugOutputStream << endl;
    if (x.containsKey("dirty"))
    //  if( keyExists != nullptr)

    {
      *debugOutputStream << "dirty Start:" << endl;
    }
    else
      *debugOutputStream << "NOT DIRTY" << endl;

    // Serial.prinln(IOData[kv.key().c_str()]["value"])
    for (JsonPair kv2 : kv.value().as<JsonObject>())
    {
      // Serial.println(kv2.key().c_str());
      // Serial.println( kv2.value().is<bool>());
      // Serial.println( kv2.value().is<int>());
      // Serial.println( kv2.value().is<float>());

      //      Serial.println(kv2.key().c_str());
      //     if (kv2.value().is<bool>())
      //     {
      //       Serial.print("bool:");
      //       Serial.println(kv2.value().as<bool>());
      //     }
      //     else
      //     {
      //       Serial.print("int:");
      //       Serial.println(kv2.value().as<int>());
      //     }
    }
  }
#endif
}

// JSON Set tag is invalid
void faultHandler(uint32_t arg)
{
}

void setCirculationPumpOnDuration(uint32_t aDuration)
{

#ifdef PROCESS_TERMINAL
  *debugOutputStream << "setCirculationPumpOnDuration:" << aDuration << endl;
#endif
  PY_001.setActiveDuration((uint16_t)aDuration);

  EEPROMWriteUint16((uint16_t)aDuration,
                    EEPROM_CIRCULATION_PUMP_ON_DURATION_ADDR);
}

void setCirculationPumpOffDuration(uint32_t aDuration)
{

#ifdef PROCESS_TERMINAL
  *debugOutputStream << "setCirculationPumpOffDuration:" << aDuration << endl;
#endif
  PY_001.setInactiveDuration((uint16_t)aDuration);

  EEPROMWriteUint16((uint16_t)aDuration,
                    EEPROM_CIRCULATION_PUMP_OFF_DURATION_ADDR);
}

void setFanOnDuration(uint32_t aDuration)
{
#ifdef PROCESS_TERMINAL
  *debugOutputStream << "setFanOnDuration:" << aDuration << endl;
#endif
  MY_101.setActiveDuration((uint16_t)aDuration);

  EEPROMWriteUint16((uint16_t)aDuration,
                    EEPROM_FAN_ON_DURATION_ADDR);
}

void setFanOffDuration(uint32_t aDuration)
{

#ifdef PROCESS_TERMINAL
  *debugOutputStream << "setFanOffDuration:" << aDuration << endl;
#endif
  MY_101.setInactiveDuration((uint16_t)aDuration);
  EEPROMWriteUint16((uint16_t)aDuration,
                    EEPROM_FAN_OFF_DURATION_ADDR);
}

void setHeatingPadOnTime(uint32_t anEpoch)
{

#ifdef PROCESS_TERMINAL
  *debugOutputStream << "setHeatingPadOnTime:" << anEpoch << endl;
#endif

  heatingPad.onEpoch = alarmTimeToUTC(anEpoch);

  EEPROMWriteTimeEntry(anEpoch,
                       EEPROM_HEATING_PAD_ON_TIME_ADDR);
}

void setHeatingPadOffTime(uint32_t anEpoch)
{

#ifdef PROCESS_TERMINAL
  *debugOutputStream << "setHeatingPadOffTime:" << anEpoch << endl;
#endif

  heatingPad.offEpoch = alarmTimeToUTC(anEpoch);

  EEPROMWriteTimeEntry(anEpoch,
                       EEPROM_HEATING_PAD_OFF_TIME_ADDR);
}

void setGrowingChamberLightsOnTime(uint32_t anEpoch)
{

#ifdef PROCESS_TERMINAL
  *debugOutputStream << "setGrowingChamberLightsOnTime:" << anEpoch << endl;
#endif

  growingChamberLights.onEpoch = alarmTimeToUTC(anEpoch);
  EEPROMWriteTimeEntry(growingChamberLights.onEpoch,
                       EEPROM_GROWING_CHAMBER_ON_TIME_ADDR);
}

void setGrowingChamberLightsOffTime(uint32_t anEpoch)
{
#ifdef PROCESS_TERMINAL
  *debugOutputStream << "setChamberLightsOffTime:" << anEpoch << endl;
#endif

  growingChamberLights.offEpoch = alarmTimeToUTC(anEpoch);
  EEPROMWriteTimeEntry(growingChamberLights.offEpoch,
                       EEPROM_GROWING_CHAMBER_OFF_TIME_ADDR);
}

void setSeedingAreaLightsOnTime(uint32_t anEpoch)
{

#ifdef PROCESS_TERMINAL
  *debugOutputStream << "setSeedingAreaLightsOnTime:" << anEpoch << endl;
#endif

  seedingAreaLights.onEpoch = alarmTimeToUTC(anEpoch);
  EEPROMWriteTimeEntry(seedingAreaLights.onEpoch,
                       EEPROM_SEEDING_AREA_ON_TIME_ADDR);
}

void setSeedingAreaLightsOffTime(uint32_t anEpoch)
{
#ifdef PROCESS_TERMINAL
  *debugOutputStream << "setSeedingAreaLightsOffTime:" << anEpoch << endl;
#endif
  seedingAreaLights.offEpoch = alarmTimeToUTC(anEpoch);
  EEPROMWriteTimeEntry(seedingAreaLights.offEpoch,
                       EEPROM_SEEDING_AREA_OFF_TIME_ADDR);
}

// Circulaion Pummp HOA
void setCirculationPumpHOA(uint32_t aState)
{

  uint16_t hoaState = (uint16_t)aState;

#ifdef PROCESS_TERMINAL
  *debugOutputStream << "setCirculationPumpHOA:" << hoaState << endl;
#endif

  if (HS_105AB.setRemoteState(hoaState))
  {
    EEPROMWriteUint16(hoaState,
                      EEPROM_HS_105HOA_ADDR);
  }
}

// Fan remote HOA
void setGrowingChamberFanHOA(uint32_t aState)
{

  uint16_t hoaState = (uint16_t)aState;

#ifdef PROCESS_TERMINAL
  *debugOutputStream << "setGrowingChamberFanHOA:" << hoaState << endl;
#endif

  if (HS_101AB.setRemoteState(hoaState))
  {
    EEPROMWriteUint16(hoaState,
                      EEPROM_HS_101HOA_ADDR);
  }
}

// seeding area light  remote HOA
void setSeedingLightHOA(uint32_t aState)
{

  uint16_t hoaState = (uint16_t)aState;

#ifdef PROCESS_TERMINAL
  *debugOutputStream << "setSeedingLightHOA:" << hoaState << endl;
#endif

  if (HS_102AB.setRemoteState(hoaState))
  {
    EEPROMWriteUint16(hoaState,
                      EEPROM_HS_102HOA_ADDR);
  }
}

// GC area light  remote HOA
void setGrowingChamberLightHOA(uint32_t aState)
{

  uint16_t hoaState = (uint16_t)aState;

#ifdef PROCESS_TERMINAL
  *debugOutputStream << "setGrowingChamberLightHOA:" << hoaState << endl;
#endif

  if (HS_103AB.setRemoteState(hoaState))
  {
    EEPROMWriteUint16(hoaState,
                      EEPROM_HS_103HOA_ADDR);
  }
}

// heating pad
void setHeatingPadHOA(uint32_t aState)
{

  uint16_t hoaState = (uint16_t)aState;

#ifdef PROCESS_TERMINAL
  *debugOutputStream << "setHeatingPadtHOA:" << hoaState << endl;
#endif

  if (HS_104AB.setRemoteState(hoaState))
  {
    EEPROMWriteUint16(hoaState,
                      EEPROM_HS_104HOA_ADDR);
  }
}

void setBoardTime(uint32_t anEpoch)
{
#ifdef PROCESS_TERMINAL
  *debugOutputStream << "setBoardTime:" << anEpoch << endl;
#endif
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
#ifdef PROCESS_TERMINAL
  *debugOutputStream << "updateCalibration:"
                     << " status:" << status << " eepromArrd:" << eepromAddr << endl;
#endif
  mqttSendCalibration(mqttStatusSendTopic);
}

void doPHCalibration(uint32_t mode)
{
#ifdef PROCESS_TERMINAL
  *debugOutputStream << "doPHCalibration:" << mode << endl;
#endif
  switch (mode)
  {
  case CALIBRATE_HIGH:
    updateCalibration(AT_001.calibrateHigh(), EEPROM_PH_HIGH_CAL_EPOCH_ADDR);
    break;
  case CALIBRATE_MID:
    updateCalibration(AT_001.calibrateMid(), EEPROM_PH_MID_CAL_EPOCH_ADDR);

    break;
  case CALIBRATE_LOW:
    updateCalibration(AT_001.calibrateLow(), EEPROM_PH_LOW_CAL_EPOCH_ADDR);
    break;
  case CALIBRATE_CLEAR:
    AT_001.calibrateClear();
    EEPROMWriteTimeEntry(0, EEPROM_PH_HIGH_CAL_EPOCH_ADDR);
    EEPROMWriteTimeEntry(0, EEPROM_PH_MID_CAL_EPOCH_ADDR);
    EEPROMWriteTimeEntry(0, EEPROM_PH_LOW_CAL_EPOCH_ADDR);

    break;
  case CALIBRATE_STATUS:

    mqttSendCalibration(mqttStatusSendTopic);

    break;
  default:
    break;
  }
}

void doECCalibration(uint32_t mode)
{
#ifdef PROCESS_TERMINAL
  *debugOutputStream << "doECCalibration:" << mode << endl;
#endif
  switch (mode)
  {
  case CALIBRATE_HIGH:
    updateCalibration(AT_002.calibrateHigh(), EEPROM_EC_HIGH_CAL_EPOCH_ADDR);

    break;

  case CALIBRATE_LOW:
    updateCalibration(AT_002.calibrateLow(), EEPROM_EC_LOW_CAL_EPOCH_ADDR);
    break;
  case CALIBRATE_CLEAR:
    AT_002.calibrateClear();
    EEPROMWriteTimeEntry(0, EEPROM_EC_DRY_CAL_EPOCH_ADDR);
    EEPROMWriteTimeEntry(0, EEPROM_EC_HIGH_CAL_EPOCH_ADDR);
    EEPROMWriteTimeEntry(0, EEPROM_EC_LOW_CAL_EPOCH_ADDR);
    break;
  case CALIBRATE_DRY:
 
    updateCalibration(AT_002.calibrateDry(), EEPROM_EC_DRY_CAL_EPOCH_ADDR);
    break;
  case CALIBRATE_STATUS:

    mqttSendCalibration(mqttStatusSendTopic);

    break;
  default:
    break;
  }
}

void mqttSendCalibration(const char *topic)
{

  char txBuff[MQTT_TX_SZ + 60];

  sprintf(txBuff,
          "{\"AT-001HTSF\": %ld, \"AT-001MTSF\":%ld, \"AT-001LTSF\":%ld, \"AT-002HTSF\":%ld,\"AT-002LTSF\": %ld, \"AT-002DTSF\":%ld, \"AT-001STF\":%d, \"AT-002STF\":%d}",
          EEPROMReadTimeEntry(EEPROM_PH_HIGH_CAL_EPOCH_ADDR), EEPROMReadTimeEntry(EEPROM_PH_MID_CAL_EPOCH_ADDR), EEPROMReadTimeEntry(EEPROM_PH_LOW_CAL_EPOCH_ADDR),
          EEPROMReadTimeEntry(EEPROM_EC_HIGH_CAL_EPOCH_ADDR), EEPROMReadTimeEntry(EEPROM_EC_LOW_CAL_EPOCH_ADDR), EEPROMReadTimeEntry(EEPROM_EC_DRY_CAL_EPOCH_ADDR),
          AT_001.getProbeCommandStatus(), AT_002.getProbeCommandStatus()
          );

#ifdef PROCESS_TERMINAL
  *debugOutputStream << "mqttSendCalibration buffer:" << txBuff << endl;
#endif
  mqttClient.publish(topic, txBuff);
}

void mqttSendSetpointPV(const char *topic)
{
#ifdef PROCESS_TERMINAL
  *debugOutputStream << "mqttSendSetpointPV:" << endl;
#endif

  char txBuff[MQTT_TX_SZ];

  sprintf(txBuff,
          "{\"DY-102-ONT-SPF\": %ld, \"DY-102-OFT-SPF\":%ld, \"DY-103-ONT-SPF\":%ld, \"DY-103-OFT-SPF\":%ld,\"DY-104-ONT-SPF\": %ld, \"DY-104-OFT-SPF\":%ld}",
          seedingAreaLights.onEpoch, seedingAreaLights.offEpoch, growingChamberLights.onEpoch, growingChamberLights.offEpoch, heatingPad.onEpoch, heatingPad.offEpoch);
  mqttClient.publish(topic, txBuff);

  sprintf(txBuff,
          "{\"PY-001-ONP-SPF\": %ld, \"PY-001-OFP-SPF\":%ld, \"MY-101-ONP-SPF\":%ld, \"MY-101-OFP-SPF\":%ld}",
          PY_001.getActiveDuration() / 1000, PY_001.getInActiveDuration() / 1000, MY_101.getActiveDuration() / 1000, MY_101.getInActiveDuration() / 1000);
  mqttClient.publish(topic, txBuff);

  sprintf(txBuff,
          "{\"PY-001-ONP-TMR\": %ld, \"PY-001-OFP-TMR\":%ld, \"MY-101-ONP-TMR\":%ld, \"MY-101-OFP-TMR\":%ld}",
          PY_001.getCurrentActiveDuration() / 1000, PY_001.getCurrentInactiveDuration() / 1000, MY_101.getCurrentActiveDuration() / 1000, MY_101.getCurrentInactiveDuration() / 1000);
  mqttClient.publish(topic, txBuff);
}

void mqttSendHOA(const char *topic)
{
#ifdef PROCESS_TERMINAL
  *debugOutputStream << "mqttSendHOA:" << endl;
#endif

  char txBuff[MQTT_TX_SZ];

  sprintf(txBuff,
          "{\"HS-102-HOA-SPF\": %d, \"HS-103-HOA-SPF\":%d, \"HS-104-HOA-SPF\":%d, \"HS-101-HOA-SPF\":%d,\"HS-001-HOA-SPF\": %d}",
          HS_102AB.getCurrentState(), HS_103AB.getCurrentState(), HS_104AB.getCurrentState(), HS_101AB.getCurrentState(), HS_105AB.getCurrentState());
  mqttClient.publish(topic, txBuff);
}

void mqttSendEU(const char *topic)
{
#ifdef PROCESS_TERMINAL
  *debugOutputStream << "mqttSendEU:" << endl;
#endif

  char txBuff[MQTT_TX_SZ];
  char str_tmp[8];
  char str_tmp2[8];
  char str_tmp3[8];
  char str_tmp4[8];
  char str_tmp5[8];
  char str_tmp6[8];
  char str_tmp7[8];

  (TT_001T > 0.0) ? dtostrf(TT_001T, 3, 1, str_tmp)
                  : dtostrf(0, 3, 1, str_tmp);

  dtostrf(AT_001.getSample(), 3, 1, str_tmp2); // pH
  dtostrf(AT_001.getCompensatedTemperature(), 3, 1, str_tmp3);
  dtostrf(AT_002.getSample(), 5, 1, str_tmp4); // EC
  dtostrf(AT_002.getTDS(), 6, 1, str_tmp5);
  dtostrf(AT_002.getSalinity(), 6, 1, str_tmp6);
  dtostrf(AT_002.getSpecificGravity(), 6, 4, str_tmp7);

  sprintf(txBuff,
          "{\"TT-001\": %s, \"AT-001PH\":%s, \"AT_001T\":%s, \"AT-002EC\":%s,\"AT-002TDS\": %s, \"AT-002SAL\":%s, \"AT-002SG\":%s}",
          str_tmp, str_tmp2, str_tmp3, str_tmp4, str_tmp5, str_tmp6, str_tmp7);
  mqttClient.publish(topic, txBuff);
}

void mqttSendDiscreteIO(const char *topic)
{
#ifdef PROCESS_TERMINAL
  *debugOutputStream << "mqttSendDiscreteIO:" << endl;
#endif

  char txBuff[MQTT_TX_SZ];

  sprintf(txBuff,
          "{\"LSHH-002\": %d, \"MY-101\":%d, \"PY-001\":%d, \"DY-102\":%d,\"DY-103\": %d, \"DY-104\":%d}",
          LSHH_002.getSample(), MY_101.isActive(), PY_001.isActive(), DY_102.isActive(), DY_103.isActive(), DY_104.isActive());
  mqttClient.publish(topic, txBuff);
}

/*******************************************************************************************************
** END MQTT related functions
********************************************************************************************************/
/*******************************************************************************************************
** START Modbus related functions
********************************************************************************************************/
#if defined(ENABLE_MODBUS_IP) || defined(ENABLE_MODBUS_SERIAL)
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
  // writeModbusCoil(COIL_STATUS_READ_WRITE_OFFSET, CW_PY_002, PY_002.isActive());
#ifdef ENABLE_AMBIENT_T_H
  isNaanModbus(AT_101H);
  modbusRegisters[HR_AT_101] = bfconvert.regsf[0];
  modbusRegisters[HR_AT_101 + 1] = bfconvert.regsf[1];

  isNaanModbus(AT_101T);
  modbusRegisters[HR_TT_101] = bfconvert.regsf[0];
  modbusRegisters[HR_TT_101 + 1] = bfconvert.regsf[1];
#endif
#if defined(NO_PING)
  bfconvert.val = LT_002MedianFilter.getMedian();
  modbusRegisters[HR_LT_002] = bfconvert.regsf[0];
  modbusRegisters[HR_LT_002 + 1] = bfconvert.regsf[1];
#endif // if defined(NO_PING)

#ifdef ENABLE_FLOWMETER
  bfconvert.val = FT_002.getCurrentFlowRate();
  modbusRegisters[HR_FT_002] = bfconvert.regsf[0];
  modbusRegisters[HR_FT_002 + 1] = bfconvert.regsf[1];

  bfconvert.val = FT_002.getCummulativeVolume();
  modbusRegisters[HR_FT_002_VOL] = bfconvert.regsf[0];
  modbusRegisters[HR_FT_002_VOL + 1] = bfconvert.regsf[1];

  bfconvert.val = FT_003.getCurrentFlowRate();
  modbusRegisters[HR_FT_003] = bfconvert.regsf[0];
  modbusRegisters[HR_FT_003 + 1] = bfconvert.regsf[1];
#endif
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
  // modbusRegisters[HR_PY_002_ONP_CV] = PY_002.getCurrentActiveDuration() / 1000;
  // modbusRegisters[HR_PY_002_OFP_CV] = PY_002.getCurrentInactiveDuration() / 1000;

  // Circulation pump, gc light, seeding area HOA
  modbusRegisters[HR_HS_105_HOA_CV] = HS_105AB.getCurrentState();
  modbusRegisters[HR_HS_103_HOA_CV] = HS_103AB.getCurrentState();
  modbusRegisters[HR_HS_102_HOA_CV] = HS_102AB.getCurrentState();

// CO2
#ifdef ENABLE_CO2
  modbusRegisters[HR_AT_102] = (uint16_t)AT_102MedianFilter.getMedian();
#endif

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
  modbusRegisters[HR_AT_001_SP_CV] = (uint16_t)(// XIC_001.getSP() * 10); // pH Setpoint (pH * 10 )
  modbusRegisters[HR_AT_011_AV_CV] = // XIC_001.getAutoVolume();          // pH volume (ml)
  modbusRegisters[HR_AT_011_AT_CV] = // XIC_001.getAutoInterval();        // ph volume dispensed every T (s) during auto
  modbusRegisters[HR_AT_011_MV_CV] = // XIC_001.getManualVolume();        // manual volume (ml)
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
    AT_002.retrieveCompensatedTemperature();

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
    AT_001.retrieveCompensatedTemperature();
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
    // XIC_001.setSP(((float)(modbusRegisters[HW_AT_001_SP]) / 10.0));

    EEPROMWriteUint16(modbusRegisters[HW_AT_001_SP],
                      EEPROM_PH_SETPOINT_ADDR);
    modbusRegisters[HW_AT_001_SP] = -1;
  }

  // set and persist EC setpoint
  if (modbusRegisters[HW_AT_002_SP] != 0)
  {
    // XIC_002.setSP(modbusRegisters[HW_AT_002_SP]);
    // XIC_003.setSP(modbusRegisters[HW_AT_002_SP]);
    // EEPROMWriteUint16(modbusRegisters[HW_AT_002_SP],
    //                 EEPROM_EC_SETPOINT_ADDR);
    modbusRegisters[HW_AT_002_SP] = -1;
  }

  // pH Auto Volume
  if (modbusRegisters[HW_AT_011_AV] != 0)
  {
    // XIC_001.setAutoVolume(modbusRegisters[HW_AT_011_AV]);

    EEPROMWriteUint16(modbusRegisters[HW_AT_011_AV],
                      EEPROM_PH_AUTO_VOL_ADDR);
    modbusRegisters[HW_AT_011_AV] = -1;
  }

  // pH Down Auto Interval
  if (modbusRegisters[HW_AT_011_AT] != 0)
  {
    // XIC_001.setAutoInterval(modbusRegisters[HW_AT_011_AT]);

    EEPROMWriteUint16(modbusRegisters[HW_AT_011_AT],
                      EEPROM_PH_AUTO_INTERVAL_ADDR);
    modbusRegisters[HW_AT_011_AT] = -1;
  }

  // pH Down Manual Volume
  if (modbusRegisters[HW_AT_011_MV] != 0)
  {
    // XIC_001.setManualVolume(modbusRegisters[HW_AT_011_MV]);

    EEPROMWriteUint16(modbusRegisters[HW_AT_011_MV],
                      EEPROM_PH_MANUAL_VOL_ADDR);
    modbusRegisters[HW_AT_011_MV] = -1;
  }

  // N1/N2 auto Volume
  if (modbusRegisters[HW_AT_002_AV] != 0)
  {
    //  XIC_002.setAutoVolume(modbusRegisters[HW_AT_002_AV]);
    //  XIC_003.setAutoVolume(modbusRegisters[HW_AT_002_AV]);
    //  EEPROMWriteUint16(modbusRegisters[HW_AT_002_AV],
    //                    EEPROM_N1N2_AUTO_VOL_ADDR);
    //  modbusRegisters[HW_AT_002_AV] = -1;
  }

  // N1/N2 auto Interval
  if (modbusRegisters[HW_AT_002_AT] != 0)
  {
    //  XIC_002.setAutoInterval(modbusRegisters[HW_AT_002_AT]);
    //  XIC_003.setAutoInterval(modbusRegisters[HW_AT_002_AT]);
    //  EEPROMWriteUint16(modbusRegisters[HW_AT_002_AT],
    //                    EEPROM_N1N2_AUTO_INTERVAL_ADDR);
    //  modbusRegisters[HW_AT_002_AT] = -1;
  }

  // N1/N2 Manual Volume
  if (modbusRegisters[HW_AT_002_MV] != 0)
  {
    //  XIC_002.setManualVolume(modbusRegisters[HW_AT_002_MV]);
    //  XIC_003.setManualVolume(modbusRegisters[HW_AT_002_MV]);

    //  EEPROMWriteUint16(modbusRegisters[HW_AT_002_MV],
    //                    EEPROM_N1N2_MANUAL_VOL_ADDR);
    modbusRegisters[HW_AT_002_MV] = -1;
  }

  // N1/N2 Manual HOA
  if (modbusRegisters[HW_HS_012_HOA_SP] != 0)
  {
    //  XIC_002.setControlMode(modbusRegisters[HW_HS_012_HOA_SP]);
    //  XIC_003.setControlMode(modbusRegisters[HW_HS_012_HOA_SP]);

    //  EEPROMWriteUint16(modbusRegisters[HW_HS_012_HOA_SP],
    //                    EEPROM_N1N2_MODE_ADDR);
    modbusRegisters[HW_HS_012_HOA_SP] = -1;
  }
}

void processMiscCommands()
{
#ifdef ENABLE_FLOWMETER
  if (getModbusCoilValue(COIL_STATUS_READ_WRITE_OFFSET, CW_KY_004))
    FT_002.resetStatistics();
#endif
}

#endif

/*******************************************************************************************************
** END Modbus related functions
********************************************************************************************************/

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

  MY_101.setActiveDuration(EEPROMReadUint16(EEPROM_FAN_ON_DURATION_ADDR));
  MY_101.setInactiveDuration(EEPROMReadUint16(EEPROM_FAN_OFF_DURATION_ADDR));

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
#define TIMER_ALARM_ON '1'               // Timer Alarm On A14:00:00->turn on  lights at 4AM
#define TIMER_ALARM_OFF '0'              // Timer Alarm On A023:00:00->turn off lighst at 11 PM
#define HELP_HEADER '?'
#define LIGHT_HEADER 'L'                  // Light Header tag
#define LIGHTS_ON '1'                     // Turn lights on L1
#define LIGHTS_OFF '0'                    // Turn lights off L0
#define CIRCULATION_PUMP_HEADER 'C'       // Circulation pump Header tag
#define DRAIN_PUMP_HEADER 'R'             // Drain punmp Header tag
#define FAN_HEADER 'F'                    // Light Header tag
#define TEMPERATURE_COMPENSATE_HEADER 'E' // Atlas Sensor Tempearture Compensate
#define CALIBRATE_EC_HEADER 'Y'
#define CALIBRATE_PH_HEADER 'Z'

#define LIGHTS_ON '1'  // Turn lights on L1
#define LIGHTS_OFF '0' // Turn lights off L0

// define LIGHTS_TOOGLE 't'         // toggle
#define LIGHTS_DUTY_CYCLE 'd'        // Ld60->60% dominant color default mostly red 60-90 percent allowed
#define LIGHTS_DUTY_CYCLE_PERIOD 'r' // Lr3600->change the duty cycle between  60-90 % every hour
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
#define SERIALIZE_FT_002 'm'

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
#if defined(ENABLE_AMBIENT_T_H)
    *debugOutputStream << "HT-101: "
                       << "Rel Humidity:" << AT_101H << " % Temperature:" << AT_101T;
    *debugOutputStream << " C Heat Index " << AT_101HI << endl;
#endif
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
    // refreshModbusRegisters();
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
  EEPROMWriteTimeEntry(*anEpoch, eepromAddr);
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
    // PY_002.setActiveDuration(duration);
    EEPROMWriteUint16(duration, EEPROM_DRAIN_PUMP_ON_DURATION_ADDR);
  }
  else
  {
    // PY_002.setInactiveDuration(duration);
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
  *debugOutputStream << F("Sm - Serialize FT-002") << endl;
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
    // XIC_001.serialize(debugOutputStream, true);

    break;
  case SERIALIZE_FT_002:
#if defined(ENABLE_FLOWMETER)
    FT_002.serialize(debugOutputStream, true);
#endif
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
        // HS_102AB.setRemoteState(x);
        EEPROMWriteUint16(x,
                          EEPROM_HS_102HOA_ADDR);
        *debugOutputStream << F("GC SOFT HOA:Writing") << endl;
        EEPROMWriteUint16(x,
                          EEPROM_HS_103HOA_ADDR);

        *debugOutputStream << "105 mem:" << EEPROMReadUint16(EEPROM_HS_105HOA_ADDR) << " address:" << EEPROM_HS_105HOA_ADDR << endl;

        *debugOutputStream << "101 mem:" << EEPROMReadUint16(EEPROM_HS_101HOA_ADDR) << " address:" << EEPROM_HS_101HOA_ADDR << endl;
        *debugOutputStream << "102 mem:" << EEPROMReadUint16(EEPROM_HS_102HOA_ADDR) << " address:" << EEPROM_HS_102HOA_ADDR << endl;
        *debugOutputStream << "103 mem:" << EEPROMReadUint16(EEPROM_HS_103HOA_ADDR) << " address:" << EEPROM_HS_103HOA_ADDR << endl;
#if defined(ENABLE_MODBUS_IP) || defined(ENABLE_MODBUS_SERIAL)
        *debugOutputStream << "modbusRegisters[HR_MY_101_ONP_CV]:" << modbusRegisters[HR_MY_101_ONP_CV] << endl;
#endif
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