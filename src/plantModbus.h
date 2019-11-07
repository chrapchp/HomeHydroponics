/**
 * @file        plantModbus.h
 * @version     0.1
 * @date        2017May3
 * @author      pjc

 *
 * @description
 *  Helpers for plant lighting and control using Modbus
 *
 * Using arduino modbus implementation @
 * https://github.com/smarmengol/Modbus-Master-Slave-for-Arduino
 */


#include <SoftwareSerial.h>
#include <ModbusRtu.h>

#define COIL_STATUS_READ_WRITE_OFFSET 0
#define COIL_STATUS_WRITE_START_BIT     (16 * 5) // 80 coils/bits for reads then
                                                 // followed by write memory for
                                                 // coils



// #define HOLDING_REGISTER_READ_OFFSET 10		// start read holding
// regisers
// #define HOLDING_REGISTER_WRITE_OFFSET 30
#define MODBUS_REG_COUNT 150

// read coils
#define CS_HS_001   0          // Drain Pump Hand status : Start/Stop
#define CS_SSH_101   1         // Smoke Detector
#define CS_HS_002   2          // Inlet H20 Open/Close
#define CS_LSHH_002   3        // nutrient mixture hi-hi level switch
#define CS_HS_003C   4        // Local/Remote Control


// write coils
#define CW_VY_001A   81        // inlet H20 valve, active low 12VDC
#define CW_PY_002   82        // Drain Pump 12VDC
#define CW_KY_001   83        // Reset config to defaults
#define CW_SPARE1   84        // SPARE1
#define CW_SPARE2   85        // SPARE2
#define CW_SPARE3   86        // SPARE3
#define CW_SPARE4   87        // SPARE4
#define CW_DY_102   88        // Seeding Area LED 120 VAC
#define CW_DY_103   89        // Growing Chamber LED 120 VAC
#define CW_PY_001   90        // Circulation Pump 120VAC
#define CW_MY_101   91        // Fan, 60 on/60 off 120VAC
#define CW_AT_002_CL   92        // EC Calibrate Low
#define CW_AT_002_CD   93        // EC Calibrate Dry
#define CW_AT_002_CH   94        // EC Calibrate High
#define CW_AT_002_CR   95        // EC Calibrate Clear
#define CW_AT_002_CQ   96        // EC Calibrate Query
#define CW_AT_002_TC   97        // EC Temperature Compensate
#define CW_AT_001_CL   98        // pH Calibrate Low
#define CW_AT_001_CM   99        // pH Calibrate Mid
#define CW_AT_001_CH   100        // pH Calibrate High
#define CW_AT_001_CR   101        // pH Calibrate Clear
#define CW_AT_001_CQ   102        // pH Calibrate Query
#define CW_AT_001_TC   103        // pH Temperature Compensate
#define CW_XY_001   104        // Nutrient 1 Pump Run
#define CW_XY_002   105        // Nutrient 2 Pump Run
#define CW_XY_003   106        // pH Down Pump Run
#define CW_XY_004   107        // Hydrogen Peroxide Pump Run
#define CW_PY_002_SP   108        // Drain Pump 12VDC On/Off from HMI
#define CW_HS_002_SP   109        // Inlet H20 Open/Close from HMI
#define CW_LSHH_002_BYP   110        // nutrient mixture hi-hi level switch Bypass from HMI (1=bypass)
#define CW_DY_104   111        // Heating Pad on/off






// Read holding registers 16 bit
#define HR_HS_105_HOA_CV   30        // Circulation pump HOA current value (remote or local)
#define HR_HS_102_HOA_CV   31        // Seeding Area Light  HOA current value (remote or local)
#define HR_HS_103_HOA_CV   32        // Growing Chamber Light HOA current value (remote or local)
#define HR_SPARE4   33        // Spare

#define HR_KY_002   34        // Version
#define HR_AT_001_CC   35        // pH Calibrate Calibrated Count
#define HR_AT_001_CS   36        // pH Calibrate Command Status
#define HR_AT_001_PS   37        // pH Calibrate Poll  Status
#define HR_AT_002_CC   38        // EC Calibrate Calibrated Count
#define HR_AT_002_CS   39        // EC Calibrate Command Status
#define HR_AT_002_PS   40        // EC Calibrate Poll  Status
#define HR_MY_101_OFP_CV   41        // Fan  120VAC OFF Period in sec  Current Value
#define HR_MY_101_ONP_CV   42        // Fan  120VAC ON Period in sec  Current Value
#define HR_PY_001_OFP_CV   43        // Circulation Pump 120VAC OFF Period in sec  Current Value
#define HR_PY_001_ONP_CV   44        // Circulation Pump 120VAC ON Period in sec  Current Value
#define HR_SPARE1  45        // Nutrient 1 Pump HOA (H=2,O=1,A=3)
#define HR_SPARE2   46        // Nutrient 2 Pump  HOA (H=2,O=1,A=3)
#define HR_SPARE3  47        // pH Down Pump  HOA (H=2,O=1,A=3)
#define HR_AT_001_SP_CV   48        // pH Setpoint (pH * 10 )
#define HR_AT_002_SP_CV   49        // EC Setpoint
#define HR_AT_011_AV_CV   50        // pH volume (m) to dispense during auto for one  interval T
#define HR_AT_011_AT_CV   51        // ph volume dispensed every T (s) during auto
#define HR_AT_011_MV_CV   52        // pH volume (ml) to dispense during manual  (oneshot)
#define HR_AT_002_AV_CV   53        // N1/N2  volume (ml) to dispense during auto for one  interval T
#define HR_AT_002_AT_CV   54        // N1/N2 volume dispensed every T (s) during auto
#define HR_AT_002_MV_CV   55        // N1/N2 volume (ml) to dispense during manual (oneshot)
#define HR_MY_201_OFP_SP   56       // Tank Aerator Off Period in sec  current value
#define HR_MY_201_ONP_SP   57       // Tank Aerator On Period in sec  current value
#define HR_SPARE4 58       // Tank Aerator On Period
#define HA_SPARE5  59        // Spare



// Read Holding registers 32 bit
#define HR_AT_001   80        // Nutrient pH
#define HR_AT_001_TV   82        // pH Temperature Compensation Value
#define HR_AT_002   84        // Nutrient EC
#define HR_AT_002_TV   86        // EC Temperature Compensation Value
#define HR_AT_002SAL   88        // Nutrient Salinity
#define HR_AT_002SG   90        // Nutrient Specific Gravity
#define HR_AT_002TDS   92        // Nutrient TDS
#define HR_AT_101   94        // Ambient relative humidity
#define HR_AT_102   96        // CO2 Sensor PPM
#define HR_FT_002   98        // Inlet Flow
#define HR_FT_003   100        // Nutrient Flow
#define HR_LT_002   102        // Water Level present value in cm from top,  0-> undefined
#define HR_TT_001   104        // mixture temperature
#define HR_TT_101   106        // Abient Temperature


#define HR_DY_102_OFT_CV   120        // Seeding Area LED OFF Time  (UNIX EPOCH)  Current Value  UTC
#define HR_DY_102_ONT_CV   122        // Seeding Area Chamber LED ON Time (UNIX EPOCH) Current Value UTC
#define HR_DY_103_OFT_CV   124        // Growing Chamber LED OFF Time  (UNIX EPOCH)  Current Value UTC
#define HR_DY_103_ONT_CV   126        // Growing Chamber LED ON Time (UNIX EPOCH)  Current Value UTC
#define HR_DY_104_OFT_CV   128        // Seeding Area Heating Pad OFF Time  (UNIX EPOCH)  Current Value  UTC
#define HR_DY_104_ONT_CV   130        // Seeding Area Heating Pad LED ON Time (UNIX EPOCH) Current Value UTC




// Write holding registers 16 bits
#define HW_MY_101_OFP_SP   10        // Fan  120VAC OFF Period in sec  Setpoint
#define HW_MY_101_ONP_SP   11        // Fan  120VAC ON Period in sec  Setpoint
#define HW_PY_001_OFP_SP   12        // Circulation Pump 120VAC OFF Period in sec  Setpoint
#define HW_PY_001_ONP_SP   13        // Circulation Pump 120VAC ON Period in sec  Setpoint
#define HW_HS_011_HOA_SP   14        // pH Down Pump  HOA(H=1,O=2,A=3) from HMI
#define HW_HS_012_HOA_SP   15        // Nutrient 1 Pump  HOA(H=1,O=2,A=3) from HMI
#define HW_HS_013_HOA_SP   16        // Nutrient 2 Pump  HOA(H=1,O=2,A=3) from HMI
#define HW_AT_001_SP   17        // pH Setpoint (pH * 10 )
#define HW_AT_002_SP   18        // EC Setpoint
#define HW_AT_011_AV   19        // pH volume (m) to dispense during auto for one  interval T
#define HW_AT_011_AT   20        // ph volume dispensed every T (s) during auto
#define HW_AT_011_MV   21        // pH volume (ml) to dispense during manual  (oneshot)
#define HW_AT_002_AV   22        // N1/N2  volume (ml) to dispense during auto for one  interval T
#define HW_AT_002_AT   23        // N1/N2 volume dispensed every T (s) during auto
#define HW_AT_002_MV   24        // N1/N2 volume (ml) to dispense during manual (oneshot)
#define HW_HS_105HOA_SP   25        // Circulation Pump  HOA  (H=1, O=2, A=3) from HMI
#define HW_HS_101HOA_SP   26        // Fan HOA  (H=1, O=2, A=3)  from HMI
#define HW_HS_102HOA_SP   27        // Seeding Area  Light HOA  (H=1, O=2, A=3)  from HMI
#define HW_HS_103HOA_SP   28        // Growing Chamber  Light HOA  (H=1, O=2, A=3)  from HMI
#define HW_HS_104HOA_SP   29        // Seeding Area Heating Pad HOA  (H=1, O=2, A=3)  from HMI

// 16 be writes
#define HW_MY_201_OFP_SP   150        // Nutrutriant Tank Aerator Off Period in sec  Setpoint
#define HW_MY_201_ONP_SP   151        // Nutrutriant Tank Aerator On Period in sec  Setpoint
#define HW_MY_201_HOA_SP   151        // Nutrutriant Tank Aerator On Period in sec  Setpoint


//
// Write holding registers 32 bits
#define HW_DY_102_OFT_SP   60        // Seeding Area LED OFF Time  (UNIX EPOCH)  Setpoint  Local Time
#define HW_DY_102_ONT_SP   62        // Seeding Area Chamber LED ON Time (UNIX EPOCH) Setpoint  Local Time
#define HW_DY_103_OFT_SP   64        // Growing Chamber LED OFF Time  (UNIX EPOCH)  Setpoint  Local Time
#define HW_DY_103_ONT_SP   66        // Growing Chamber LED ON Time (UNIX EPOCH)  Setpoint  Local Time
#define HW_QT_001_SP   68        // Realt-time clock time (UNIX EPOCH)  Setpoint UTC
#define HW_DY_104_OFT_SP   70        // Seeding Area Heating Pad OFF Time  (UNIX EPOCH)  Setpoint Local Time
#define HW_DY_104_ONT_SP   72        // Seeding Area Heating Pad LED ON Time (UNIX EPOCH) Setpoint Local Time





#define MB_SLAVE_ID                     1
#define MB_SERIAL_PORT                  0  // restore to 0
#define MB_SERIAL_BAUD                  19200


union
{
        uint16_t regsf[2];
        float val;
}
bfconvert;

union
{
        uint16_t regsl[2];
        long val;
}
blconvert;

uint16_t modbusRegisters[MODBUS_REG_COUNT];

Modbus slave(MB_SLAVE_ID, MB_SERIAL_PORT);
