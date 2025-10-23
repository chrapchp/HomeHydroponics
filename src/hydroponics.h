/**
 * @file        hydroponics.h
 * @version     0.1
 * @date        2022Mar18
 * @author      pjc

 *
 * @description
 *  Helpers for plant lighting and control using Modbus
 *
 * Using arduino modbus implementation @
 * https://github.com/smarmengol/Modbus-Master-Slave-for-Arduino
 */

#include <Ethernet.h>
#define DEFAULT_IP_ADDRESS 192, 168, 1, 177
#define DEFAULT_GATEWAY 192, 168, 1, 254
#define DEFAULT_SUBNET_MASK 255, 255, 255, 0
#define DEFAULT_MAC_ADDRESS 0xDE, 0xAD, 0xBE, 0x00, 0x01, 0xFC

IPAddress defaultIP(DEFAULT_IP_ADDRESS);
IPAddress defaultGateway(DEFAULT_GATEWAY);
IPAddress defaultSubnet(DEFAULT_SUBNET_MASK);

byte defaultMAC[] = {DEFAULT_MAC_ADDRESS};

#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ArduinoHA.h>

#define DEFAULT_MQTT_ADDRESS 192, 168, 1, 81
IPAddress defaultMQTT(DEFAULT_MQTT_ADDRESS);

#define PUBLISH_INTERVAL 5000 // 5 seconds
#define MQTT_MSG_IN_SZ 300
#define MQTT_STORE_SZ 550
#define MQTT_TX_SZ 500




// char mqttMsgOut[200];

StaticJsonDocument<MQTT_STORE_SZ> IOData;

const char mqttEUSendTopic[] = "Home/Hydroponic/Controller/EU";
const char mqttStatusSendTopic[] = "Home/Hydroponic/Controller/Status";
const char hostCommandTopic[] = "Home/Hydroponic/Controller/HostCommand";
const char deviceID[] = "Controller001";
const char connectedMsg[] = "Connected";


boolean mqttReconnect();
void initIOData();
void onMQTTMessage(char *topic, byte *payload, unsigned int length);
void doOnMQQTCheck();
void mqttSendDiscreteIO(const char *topic);
void mqttSendSetpointPV(const char *topic);
void mqttSendHOA(const char *topic);
void mqttSendEU(const char *topic);
void mqttSendCalibration( const char *topic);
void setCirculationPumpOnDuration(uint32_t aDuration);
void setCirculationPumpOffDuration(uint32_t aDuration);
void setFanOnDuration(uint32_t aDuration);
void setFanOffDuration(uint32_t aDuration);
void setHeatingPadOnTime(uint32_t anEpoch);
void setHeatingPadOffTime(uint32_t anEpoch);
void setGrowingChamberLightsOnTime(uint32_t anEpoch);
void setGrowingChamberLightsOffTime(uint32_t anEpoch);
void setSeedingAreaLightsOnTime(uint32_t anEpoch);
void setSeedingAreaLightsOffTime(uint32_t anEpoch);
void setCirculationPumpHOA(uint32_t aState);
void setGrowingChamberFanHOA(uint32_t aState);
void setSeedingLightHOA(uint32_t aState);
void setGrowingChamberLightHOA(uint32_t aState);
void setHeatingPadHOA(uint32_t aState);
void setBoardTime(uint32_t anEpoch);
void doPHCalibration(int8_t mode);
void doECCalibration(int8_t mode);

#define FAULT_HDLR 0
#define PY_001_OFP_SP_HDLR 1
#define PY_001_ONP_SP_HDLR 2
#define MY_101_OFP_SP_HDLR 3
#define MY_101_ONP_SP_HDLR 4
#define DY_104_OFT_SP_HDLR 5
#define DY_104_ONT_SP_HDLR 6
#define DY_103_OFT_SP_HDLR 7
#define DY_103_ONT_SP_HDLR 8
#define DY_102_OFT_SP_HDLR 9
#define DY_102_ONT_SP_HDLR 10
#define PY_001_HOA_HDLR 11
#define MY_101_HOA_HDLR 12
#define DY_102_HOA_HDLR 13
#define DY_103_HOA_HDLR 14
#define DY_104_HOA_HDLR 15
#define QT_001_SP_HDLR 16
// define HS_900_SP_HDLR 17
#define PH_CAL_HDLR 17
#define EC_CAL_HDLR 18


const int CALIBRATE_HIGH = 1;
const int CALIBRATE_MID = 2;
const int CALIBRATE_LOW = 3;
const int CALIBRATE_CLEAR = 4;
const int CALIBRATE_STATUS = 5;
const int CALIBRATE_DRY = 2;

// setHandlers[0] = setCirculationPumpOnDuration;

// enum _dataType
// {
//   DT_I16,
//   DT_U16,
//   DT_U32,
//   DT_FLOAT,
//   DT_BOOL
// };

// typedef enum _dataType DataType;

// typedef struct _datavalue
// {
//   uint16_t uiVal;
//   int16_t iVal;
//   uint32_t ulVal;
//   float fVal;
//   bool bVal;
// } DataValue;

// typedef struct _tagCommandEntry
// {
//   char tagName[15];
//   boolean isSetpoint;
//   boolean isDirty;
//   DataType dataType;
//   DataValue currentValue;
//   DataValue previousValue;
//   bool (*dataFetcher)() = NULL;
//   void (*onDirty)() = NULL;

// } HostCommandEntry;

// #define isNaanModbus(x)   \
//   if (isnan(x))           \
//     bfconvert.val = -127; \
//   else                    \
//     bfconvert.val = x;
