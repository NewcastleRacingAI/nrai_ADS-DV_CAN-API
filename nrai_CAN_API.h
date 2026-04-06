/*      Copyright 2026 Rowan Catterall,
 *                     Newcastle University Formula Student AI Team
 *
 *      Licensed under the Apache License, Version 2.0 (the "License");
 *      you may not use this file except in compliance with the License.
 *      You may obtain a copy of the License at
 *             http://www.apache.org/licenses/LICENSE-2.0
 *
 *      Unless required by applicable law or agreed to in writing, software
 *      distributed under the License is distributed on an "AS IS" BASIS,
 *      WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *      See the License for the specific language governing permissions and
 *      limitations under the License.
 */

/*
 * This is the Newcastle University implementation of the
 * HYPERMOTIVE ADS-DV SOFTWARE INTERFACE SPECIFICATION VERSION 4.0
 * AND is extended with functionality to also read the PCAN-GPS module over CAN
 * it aims to be as minimal as possible, facilitating only the bare minimum to
 * pack and unpack the can frames only implementing checks where required by
 * the API. All data sanity checks are expected to be implemented by the calling
 * program.
 */

#ifndef NRAI_CAN_API
#define NRAI_CAN_API

#include <stdint.h>

#include <linux/can.h>

#ifndef  NRAI_FT
#define  NRAI_FT
#endif //NRAI)FT

enum{
        NRAI_CAN_ID_NMT                         = 0,
        NRAI_CAN_ID_SYNC                        = 128,
        NRAI_CAN_ID_RES_EMCY                    = 129,
        NRAI_CAN_ID_UMB_EMCY                    = 132,
        NRAI_CAN_ID_OPUS2VCU_FB                 = 291,
        NRAI_CAN_ID_RES_TPDO                    = 385,
        NRAI_CAN_ID_UMB_PDO1                    = 388,
        NRAI_CAN_ID_UMB_PDO2                    = 644,
        NRAI_CAN_ID_VCU2OPUS_1                  = 768,
        NRAI_CAN_ID_VCU2OPUS_2                  = 769,
        NRAI_CAN_ID_VCU2OPUS_3                  = 770,
        NRAI_CAN_ID_VCU2LOG_Dynamics1           = 1280,
        NRAI_CAN_ID_AI2LOG_Dynamics2            = 1281,
        NRAI_CAN_ID_VCU2LOG_Status              = 1282,
        NRAI_CAN_ID_AI2VCU_Status               = 1296,
        NRAI_CAN_ID_AI2VCU_Drive_F              = 1297,
        NRAI_CAN_ID_AI2VCU_Drive_R              = 1298,
        NRAI_CAN_ID_AI2VCU_Steer                = 1299,
        NRAI_CAN_ID_AI2VCU_Brake                = 1300,
        NRAI_CAN_ID_VCU2AI_Status               = 1312,
        NRAI_CAN_ID_VCU2AI_Drive_F              = 1313,
        NRAI_CAN_ID_VCU2AI_Drive_R              = 1314,
        NRAI_CAN_ID_VCU2AI_Steer                = 1315,
        NRAI_CAN_ID_VCU2AI_Brake                = 1316,
        NRAI_CAN_ID_VCU2AI_Speeds               = 1317,
        NRAI_CAN_ID_VCU2AI_Wheel_counts         = 1318,
        NRAI_CAN_ID_BMC_Acceleration            = 1536,
        NRAI_CAN_ID_L3GD20_Rotation_A           = 1552,
        NRAI_CAN_ID_L3GD20_Rotation_B           = 1553,
        NRAI_CAN_ID_GPS_Status                  = 1568,
        NRAI_CAN_ID_GPS_CourseSpeed             = 1569,
        NRAI_CAN_ID_GPS_PositionLongitude       = 1570,
        NRAI_CAN_ID_GPS_PositionLatitude        = 1571,
        NRAI_CAN_ID_GPS_PositionAltitude        = 1572,
        NRAI_CAN_ID_GPS_Delusions_A             = 1573,
        NRAI_CAN_ID_GPS_Delusions_B             = 1574,
        NRAI_CAN_ID_GPS_DateTime                = 1575,
        NRAI_CAN_ID_BMC_MagneticField           = 1576,
        NRAI_CAN_ID_IO                          = 1584,
        NRAI_CAN_ID_RTC_DateTime                = 1600,
        NRAI_CAN_ID_Out_IO                      = 1616,
        NRAI_CAN_ID_Out_PowerOff                = 1617,
        NRAI_CAN_ID_Out_Gyro                    = 1618,
        NRAI_CAN_ID_Out_BMC_AccScale            = 1619,
        NRAI_CAN_ID_Out_SaveConfig              = 1620,
        NRAI_CAN_ID_Out_RTC_SetTime             = 1621,
        NRAI_CAN_ID_Out_RTC_TimeFromGPS         = 1622,
        NRAI_CAN_ID_Out_Acc_FastCalibration     = 1623,
        NRAI_CAN_ID_VECTOR__INDEPENDENT_SIG_MSG = 3221225472,
};

/* ----------------------- AI_READ_START ----------------------- */

//BO_ 1312 VCU2AI_Status: 8 VCU
// SG_ HANDSHAKE : 0|1@1- (1,0) [0|1] ""  LOG,AI,OPUS
// SG_ OP_STATE : 4|4@1+ (1,0) [0|15] ""  OPUS
// SG_ SHUTDOWN_REQUEST : 8|1@1- (1,0) [0|1] ""  LOG,AI,OPUS
// SG_ AS_SWITCH_STATUS : 9|1@1- (1,0) [0|1] ""  LOG,AI,OPUS
// SG_ TS_SWITCH_STATUS : 10|1@1- (1,0) [0|1] ""  LOG,AI,OPUS
// SG_ GO_SIGNAL : 11|1@1- (1,0) [0|1] ""  LOG,AI,OPUS
// SG_ STEERING_STATUS : 12|2@1+ (1,0) [0|3] ""  LOG,AI,OPUS
// SG_ AS_STATE : 16|4@1+ (1,0) [0|7] ""  LOG,AI,OPUS
// SG_ AMI_STATE : 20|4@1+ (1,0) [0|15] ""  LOG,AI,OPUS
// SG_ FAULT_STATUS : 24|1@1- (1,0) [0|1] ""  LOG,AI,OPUS
// SG_ WARNING_STATUS : 25|1@1- (1,0) [0|1] ""  LOG,AI,OPUS
// SG_ TS_STATE : 28|4@1+ (1,0) [0|15] ""  OPUS
// SG_ WARN_BATT_TEMP_HIGH : 32|1@1- (1,0) [0|1] ""  LOG,AI,OPUS
// SG_ WARN_BATT_SOC_LOW : 33|1@1- (1,0) [0|1] ""  LOG,AI,OPUS
// SG_ EBS_STATE : 36|4@1+ (1,0) [0|15] ""  OPUS
// SG_ AI_ESTOP_REQUEST : 40|1@1- (1,0) [0|1] "__F__"  LOG,AI,OPUS
// SG_ HVIL_OPEN_FAULT : 41|1@1- (1,0) [0|1] "__F__"  LOG,AI,OPUS
// SG_ HVIL_SHORT_FAULT : 42|1@1- (1,0) [0|1] "__F__"  LOG,AI,OPUS
// SG_ EBS_FAULT : 43|1@1- (1,0) [0|1] "__F__"  LOG,AI,OPUS
// SG_ OFFBOARD_CHARGER_FAULT : 44|1@1- (1,0) [0|1] "__F__"  LOG,AI,OPUS
// SG_ AI_COMMS_LOST : 45|1@1- (1,0) [0|1] "__F__"  LOG,AI,OPUS
// SG_ AUTONOMOUS_BRAKING_FAULT : 46|1@1- (1,0) [0|1] "__F__"  LOG,AI,OPUS
// SG_ MISSION_STATUS_FAULT : 47|1@1- (1,0) [0|1] "__F__"  LOG,AI,OPUS
// SG_ CHARGE_PROCEDURE_FAULT : 48|1@1- (1,0) [0|1] "__F__"  LOG,AI,OPUS
// SG_ BMS_FAULT : 49|1@1- (1,0) [0|1] "__F__"  LOG,AI,OPUS
// SG_ BRAKE_PLAUSIBILITY_FAULT : 50|1@1- (1,0) [0|1] "__F__"  LOG,AI,OPUS
// SG_ SHUTDOWN_CAUSE : 56|8@1+ (1,0) [0|255] ""  LOG,AI,OPUS

//BO_ 1315 VCU2AI_Steer: 6 VCU
// SG_ ANGLE : 0|16@1- (0.1,0) [-21|21] "deg"  LOG,AI,OPUS
// SG_ ANGLE_MAX : 16|16@1+ (0.1,0) [0|21] "deg"  LOG,AI,OPUS
// SG_ ANGLE_REQUEST : 32|16@1- (0.1,0) [-21|21] "deg"  LOG,AI,OPUS

//BO_ 1317 VCU2AI_Speeds: 8 VCU
// SG_ FL_WHEEL_SPEED : 0|16@1+ (1,0) [0|1250] "rpm"  LOG,AI,OPUS
// SG_ FR_WHEEL_SPEED : 16|16@1+ (1,0) [0|1250] "rpm"  LOG,AI,OPUS
// SG_ RL_WHEEL_SPEED : 32|16@1+ (1,0) [0|1250] "rpm"  LOG,AI,OPUS
// SG_ RR_WHEEL_SPEED : 48|16@1+ (1,0) [0|1250] "rpm"  LOG,AI,OPUS

//BO_ 1316 VCU2AI_Brake: 5 VCU
// SG_ HYD_PRESS_F_pct : 0|8@1+ (0.5,0) [0|100] "%"  LOG,AI,OPUS
// SG_ HYD_PRESS_F_REQ_pct : 8|8@1+ (0.5,0) [0|100] "%"  LOG,AI,OPUS
// SG_ HYD_PRESS_R_pct : 16|8@1+ (0.5,0) [0|100] "%"  LOG,AI,OPUS
// SG_ HYD_PRESS_R_REQ_pct : 24|8@1+ (0.5,0) [0|100] "%"  LOG,AI,OPUS
// SG_ STATUS_BRK : 32|4@1+ (1,0) [0|15] ""  LOG,AI,OPUS
// SG_ STATUS_EBS : 36|4@1+ (1,0) [0|15] ""  LOG,AI,OPUS

//BO_ 1313 VCU2AI_Drive_F: 6 VCU
// SG_ FRONT_AXLE_TRQ : 0|16@1- (0.1,0) [-195|195] "Nm"  OPUS,LOG,AI
// SG_ FRONT_AXLE_TRQ_REQUEST : 16|16@1+ (0.1,0) [0|195] "Nm"  OPUS,LOG,AI
// SG_ FRONT_AXLE_TRQ_MAX : 32|16@1+ (0.1,0) [0|195] "Nm"  OPUS,LOG,AI

//BO_ 1314 VCU2AI_Drive_R: 6 VCU
// SG_ REAR_AXLE_TRQ_Nm : 0|16@1- (0.1,0) [-195|195] "Nm"  LOG,AI,OPUS
// SG_ REAR_AXLE_TRQ_REQUEST_Nm : 16|16@1+ (0.1,0) [0|195] "Nm"  LOG,AI,OPUS
// SG_ REAR_AXLE_TRQ_MAX_Nm : 32|16@1+ (0.1,0) [0|195] "Nm"  LOG,AI,OPUS

//BO_ 1318 VCU2AI_Wheel_counts: 8 VCU
// SG_ FL_PULSE_COUNT : 0|16@1+ (1,0) [0|65535] ""  LOG,AI,OPUS
// SG_ FR_PULSE_COUNT : 16|16@1+ (1,0) [0|65535] ""  LOG,AI,OPUS
// SG_ RL_PULSE_COUNT : 32|16@1+ (1,0) [0|65535] ""  LOG,AI,OPUS
// SG_ RR_PULSE_COUNT : 48|16@1+ (1,0) [0|65535] ""  LOG,AI,OPUS

//BO_ 1536 BMC_Acceleration: 8 PCAN_GPS
// SG_ Acceleration_X : 0|16@1- (3.91,0) [-20000|20000] "mG"  OPUS,LOG,AI
// SG_ Acceleration_Y : 16|16@1- (3.91,0) [-20000|20000] "mG"  OPUS,LOG,AI
// SG_ Acceleration_Z : 32|16@1- (3.91,0) [-20000|20000] "mG"  OPUS,LOG,AI
// SG_ Temperature : 48|8@1- (0.5,24) [-40|87.5] "C"  OPUS,LOG,AI
// SG_ VerticalAxis : 56|2@1+ (1,0) [0|3] ""  OPUS,LOG,AI
// SG_ Orientation : 58|3@1+ (1,0) [0|7] ""  OPUS,LOG,AI

//BO_ 1576 BMC_MagneticField: 6 PCAN_GPS
// SG_ MagneticField_X : 0|16@1- (0.3,0) [-9830.4|9830.1] "T"  OPUS,LOG,AI
// SG_ MagneticField_Y : 16|16@1- (0.3,0) [-9830.4|9830.1] "T"  OPUS,LOG,AI
// SG_ MagneticField_Z : 32|16@1- (0.3,0) [-9830.4|9830.1] "T"  OPUS,LOG,AI

//BO_ 1552 L3GD20_Rotation_A: 8 PCAN_GPS
// SG_ Rotation_X : 0|32@1- (1,0) [-4000|4000] "/s"  OPUS,LOG,AI
// SG_ Rotation_Y : 32|32@1- (1,0) [-4000|4000] "/s"  OPUS,LOG,AI

//BO_ 1553 L3GD20_Rotation_B: 4 PCAN_GPS
// SG_ Rotation_Z : 0|32@1- (1,0) [-4000|4000] "/s"  OPUS,LOG,AI

//BO_ 1568 GPS_Status: 3 PCAN_GPS
// SG_ GPS_AntennaStatus : 0|8@1+ (1,0) [0|255] ""  OPUS,LOG,AI
// SG_ GPS_NumSatellites : 8|8@1+ (1,0) [0|255] ""  OPUS,LOG,AI
// SG_ GPS_NavigationMethod : 16|8@1+ (1,0) [0|255] ""  OPUS,LOG,AI

//BO_ 1569 GPS_CourseSpeed: 8 PCAN_GPS
// SG_ GPS_Course : 0|32@1- (1,0) [-3.4E+038|3.4E+038] ""  OPUS,LOG,AI
// SG_ GPS_Speed : 32|32@1- (1,0) [-3.4E+038|3.4E+038] "km/h"  OPUS,LOG,AI

//BO_ 1570 GPS_PositionLongitude: 7 PCAN_GPS
// SG_ GPS_Longitude_Minutes : 0|32@1- (1,0) [-3.4E+038|3.4E+038] "'"  OPUS,LOG,AI
// SG_ GPS_Longitude_Degree : 32|16@1+ (1,0) [0|359] ""  OPUS,LOG,AI
// SG_ GPS_IndicatorEW : 48|8@1+ (1,0) [0|255] ""  OPUS,LOG,AI

//BO_ 1571 GPS_PositionLatitude: 7 PCAN_GPS
// SG_ GPS_Latitude_Minutes : 0|32@1- (1,0) [-3.4E+038|3.4E+038] "'"  OPUS,LOG,AI
// SG_ GPS_Latitude_Degree : 32|16@1+ (1,0) [0|359] ""  OPUS,LOG,AI
// SG_ GPS_IndicatorNS : 48|8@1+ (1,0) [0|255] ""  OPUS,LOG,AI

//BO_ 1572 GPS_PositionAltitude: 4 PCAN_GPS
// SG_ GPS_Altitude : 0|32@1- (1,0) [-3.4E+038|3.4E+038] "m"  OPUS,LOG,AI

//BO_ 1573 GPS_Delusions_A: 8 PCAN_GPS
// SG_ GPS_PDOP : 0|32@1- (1,0) [-3.4E+038|3.4E+038] ""  OPUS,LOG,AI
// SG_ GPS_HDOP : 32|32@1- (1,0) [-3.4E+038|3.4E+038] ""  OPUS,LOG,AI

//BO_ 1574 GPS_Delusions_B: 4 PCAN_GPS
// SG_ GPS_VDOP : 0|32@1- (1,0) [-3.4E+038|3.4E+038] ""  OPUS,LOG,AI

//BO_ 1575 GPS_DateTime: 6 PCAN_GPS
// SG_ UTC_Year : 0|8@1+ (1,0) [0|255] ""  OPUS,LOG,AI
// SG_ UTC_Month : 8|8@1+ (1,0) [0|255] ""  OPUS,LOG,AI
// SG_ UTC_DayOfMonth : 16|8@1+ (1,0) [0|255] ""  OPUS,LOG,AI
// SG_ UTC_Hour : 24|8@1+ (1,0) [0|255] ""  OPUS,LOG,AI
// SG_ UTC_Minute : 32|8@1+ (1,0) [0|255] ""  OPUS,LOG,AI
// SG_ UTC_Second : 40|8@1+ (1,0) [0|255] ""  OPUS,LOG,AI

//BO_ 1584 IO: 1 PCAN_GPS
// SG_ Din1_Status : 0|1@1+ (1,0) [0|1] ""  OPUS,LOG,AI
// SG_ Din2_Status : 1|1@1+ (1,0) [0|1] ""  OPUS,LOG,AI
// SG_ Dout_Status : 2|1@1+ (1,0) [0|1] ""  OPUS,LOG,AI
// SG_ SD_Present : 3|1@1+ (1,0) [0|1] ""  OPUS,LOG,AI
// SG_ GPS_PowerStatus : 4|1@1+ (1,0) [0|1] ""  OPUS,LOG,AI
// SG_ Device_ID : 5|3@1+ (1,0) [0|7] ""  OPUS,LOG,AI

//BO_ 1600 RTC_DateTime: 8 PCAN_GPS
// SG_ RTC_Sec : 0|8@1+ (1,0) [0|255] ""  OPUS,LOG,AI
// SG_ RTC_Min : 8|8@1+ (1,0) [0|255] ""  OPUS,LOG,AI
// SG_ RTC_Hour : 16|8@1+ (1,0) [0|255] ""  OPUS,LOG,AI
// SG_ RTC_DayOfWeek : 24|8@1+ (1,0) [0|255] ""  OPUS,LOG,AI
// SG_ RTC_DayOfMonth : 32|8@1+ (1,0) [0|255] ""  OPUS,LOG,AI
// SG_ RTC_Month : 40|8@1+ (1,0) [0|255] ""  OPUS,LOG,AI
// SG_ RTC_Year : 48|16@1+ (1,0) [0|65535] ""  OPUS,LOG,AI

//BO_ 1616 Out_IO: 1 PCAN_GPS
// SG_ Dout_Set : 0|1@1+ (1,0) [0|1] ""  OPUS,LOG,AI
// SG_ GPS_SetPower : 1|1@1+ (1,0) [0|1] ""  OPUS,LOG,AI

//BO_ 1617 Out_PowerOff: 1 PCAN_GPS
// SG_ Device_PowerOff : 0|1@1+ (1,0) [0|1] ""  OPUS,LOG,AI

//BO_ 1618 Out_Gyro: 1 PCAN_GPS
// SG_ Gyro_SetScale : 0|2@1+ (1,0) [0|3] "/s"  OPUS,LOG,AI

//BO_ 1619 Out_BMC_AccScale: 1 PCAN_GPS
// SG_ Acc_SetScale : 0|3@1+ (1,0) [0|7] ""  OPUS,LOG,AI

//BO_ 1620 Out_SaveConfig: 1 PCAN_GPS
// SG_ Config_SaveToEEPROM : 0|1@1+ (1,0) [0|1] ""  OPUS,LOG,AI

//BO_ 1621 Out_RTC_SetTime: 8 PCAN_GPS
// SG_ RTC_SetSec : 0|8@1+ (1,0) [0|59] ""  OPUS,LOG,AI
// SG_ RTC_SetMin : 8|8@1+ (1,0) [0|59] ""  OPUS,LOG,AI
// SG_ RTC_SetHour : 16|8@1+ (1,0) [0|23] ""  OPUS,LOG,AI
// SG_ RTC_SetDayOfWeek : 24|8@1+ (1,0) [0|6] ""  OPUS,LOG,AI
// SG_ RTC_SetDayOfMonth : 32|8@1+ (1,0) [1|31] ""  OPUS,LOG,AI
// SG_ RTC_SetMonth : 40|8@1+ (1,0) [1|12] ""  OPUS,LOG,AI
// SG_ RTC_SetYear : 48|16@1+ (1,0) [1900|2099] ""  OPUS,LOG,AI

//BO_ 1622 Out_RTC_TimeFromGPS: 1 PCAN_GPS
// SG_ RTC_SetTimeFromGPS : 0|1@1+ (1,0) [0|1] ""  OPUS,LOG,AI

//BO_ 1623 Out_Acc_FastCalibration: 4 PCAN_GPS
// SG_ Acc_SetCalibTarget_X : 0|2@1+ (1,0) [0|3] ""  OPUS,LOG,AI
// SG_ Acc_SetCalibTarget_Y : 8|2@1+ (1,0) [0|3] ""  OPUS,LOG,AI
// SG_ Acc_SetCalibTarget_Z : 16|2@1+ (1,0) [0|3] ""  OPUS,LOG,AI
// SG_ Acc_StartFastCalib : 24|1@1+ (1,0) [0|1] ""  OPUS,LOG,AI

/* unsorted reference struct 
struct nrai_can_ai_read{
//1312 VCU2AI_Status: 8 VCU
        uint8_t  HANDSHAKE;                     // 0|1@1-  (1,0) [0|1]
        uint8_t  OP_STATE;                      // 4|4@1+  (1,0) [0|15]
        uint8_t  SHUTDOWN_REQUEST;              // 8|1@1-  (1,0) [0|1]
        uint8_t  AS_SWITCH_STATUS;              // 9|1@1-  (1,0) [0|1]
        uint8_t  TS_SWITCH_STATUS;              // 10|1@1- (1,0) [0|1]
        uint8_t  GO_SIGNAL;                     // 11|1@1- (1,0) [0|1]
        uint8_t  STEERING_STATUS;               // 12|2@1+ (1,0) [0|3]
        uint8_t  AS_STATE;                      // 16|4@1+ (1,0) [0|7]
        uint8_t  AMI_STATE;                     // 20|4@1+ (1,0) [0|15]
        uint8_t  FAULT_STATUS;                  // 24|1@1- (1,0) [0|1]
        uint8_t  WARNING_STATUS;                // 25|1@1- (1,0) [0|1]
        uint8_t  TS_STATE;                      // 28|4@1+ (1,0) [0|15]
        uint8_t  WARN_BATT_TEMP_HIGH;           // 32|1@1- (1,0) [0|1]
        uint8_t  WARN_BATT_SOC_LOW;             // 33|1@1- (1,0) [0|1]
        uint8_t  EBS_STATE;                     // 36|4@1+ (1,0) [0|15]
        uint8_t  AI_ESTOP_REQUEST;              // 40|1@1- (1,0) [0|1]
        uint8_t  HVIL_OPEN_FAULT;               // 41|1@1- (1,0) [0|1]
        uint8_t  HVIL_SHORT_FAULT;              // 42|1@1- (1,0) [0|1]
        uint8_t  EBS_FAULT;                     // 43|1@1- (1,0) [0|1]
        uint8_t  OFFBOARD_CHARGER_FAULT;        // 44|1@1- (1,0) [0|1]
        uint8_t  AI_COMMS_LOST;                 // 45|1@1- (1,0) [0|1]
        uint8_t  AUTONOMOUS_BRAKING_FAULT;      // 46|1@1- (1,0) [0|1]
        uint8_t  MISSION_STATUS_FAULT;          // 47|1@1- (1,0) [0|1]
        uint8_t  CHARGE_PROCEDURE_FAULT;        // 48|1@1- (1,0) [0|1]
        uint8_t  BMS_FAULT;                     // 49|1@1- (1,0) [0|1]
        uint8_t  BRAKE_PLAUSIBILITY_FAULT;      // 50|1@1- (1,0) [0|1]
        uint8_t  SHUTDOWN_CAUSE;                // 56|8@1+ (1,0) [0|255]

//1315 VCU2AI_Steer: 6 VCU
        int16_t  ANGLE;                         // 0|16@1- (0.1,0) [-21|21]
        uint16_t ANGLE_MAX;                     // 16|16@1+ (0.1,0) [0|21]
        int16_t  ANGLE_REQUEST;                 // 32|16@1- (0.1,0) [-21|21]

//1317 VCU2AI_Speeds: 8 VCU
        uint16_t FL_WHEEL_SPEED;                // 0|16@1+ (1,0) [0|1250]
        uint16_t FR_WHEEL_SPEED;                // 16|16@1+ (1,0) [0|1250]
        uint16_t RL_WHEEL_SPEED;                // 32|16@1+ (1,0) [0|1250]
        uint16_t RR_WHEEL_SPEED;                // 48|16@1+ (1,0) [0|1250]

//1316 VCU2AI_Brake: 5 VCU
        uint8_t  HYD_PRESS_F_pct;               // 0|8@1+ (0.5,0) [0|100]
        uint8_t  HYD_PRESS_F_REQ_pct;           // 8|8@1+ (0.5,0) [0|100]
        uint8_t  HYD_PRESS_R_pct;               // 16|8@1+ (0.5,0) [0|100]
        uint8_t  HYD_PRESS_R_REQ_pct;           // 24|8@1+ (0.5,0) [0|100]
        uint8_t  STATUS_BRK;                    // 32|4@1+ (1,0) [0|15]
        uint8_t  STATUS_EBS;                    // 36|4@1+ (1,0) [0|15]

//1313 VCU2AI_Drive_F: 6 VCU
        int16_t  FRONT_AXLE_TRQ;                // 0|16@1- (0.1,0) [-195|195]
        uint16_t FRONT_AXLE_TRQ_REQUEST;        // 16|16@1+ (0.1,0) [0|195]
        uint16_t FRONT_AXLE_TRQ_MAX;            // 32|16@1+ (0.1,0) [0|195]

//1314 VCU2AI_Drive_R: 6 VCU
        int16_t  REAR_AXLE_TRQ_Nm;              // 0|16@1- (0.1,0) [-195|195]
        uint16_t REAR_AXLE_TRQ_REQUEST_Nm;      // 16|16@1+ (0.1,0) [0|195]
        uint16_t REAR_AXLE_TRQ_MAX_Nm;          // 32|16@1+ (0.1,0) [0|195]

//1318 VCU2AI_Wheel_counts: 8 VCU
        uint16_t FL_PULSE_COUNT;                // 0|16@1+ (1,0) [0|65535]
        uint16_t FR_PULSE_COUNT;                // 16|16@1+ (1,0) [0|65535]
        uint16_t RL_PULSE_COUNT;                // 32|16@1+ (1,0) [0|65535]
        uint16_t RR_PULSE_COUNT;                // 48|16@1+ (1,0) [0|65535]

//1536 BMC_Acceleration: 8 PCAN_GPS
        int16_t  Acceleration_X;                // 0|16@1- (3.91,0) [-20000|20000]
        int16_t  Acceleration_Y;                // 16|16@1- (3.91,0) [-20000|20000]
        int16_t  Acceleration_Z;                // 32|16@1- (3.91,0) [-20000|20000]
        int8_t   Temperature;                   // 48|8@1- (0.5,24) [-40|87.5]
        uint8_t  VerticalAxis;                  // 56|2@1+ (1,0) [0|3]
        uint8_t  Orientation;                   // 58|3@1+ (1,0) [0|7]

//1576 BMC_MagneticField: 6 PCAN_GPS
        int16_t  MagneticField_X;               // 0|16@1- (0.3,0) [-9830.4|9830.1]
        int16_t  MagneticField_Y;               // 16|16@1- (0.3,0) [-9830.4|9830.1]
        int16_t  MagneticField_Z;               // 32|16@1- (0.3,0) [-9830.4|9830.1]

//1552 L3GD20_Rotation_A: 8 PCAN_GPS
        int32_t  Rotation_X;                    // 0|32@1- (1,0) [-4000|4000]
        int32_t  Rotation_Y;                    // 32|32@1- (1,0) [-4000|4000]

//1553 L3GD20_Rotation_B: 4 PCAN_GPS
        int32_t  Rotation_Z;                    // 0|32@1- (1,0) [-4000|4000]

//1568 GPS_Status: 3 PCAN_GPS
        uint8_t  GPS_AntennaStatus;             // 0|8@1+ (1,0) [0|255]
        uint8_t  GPS_NumSatellites;             // 8|8@1+ (1,0) [0|255]
        uint8_t  GPS_NavigationMethod;          // 16|8@1+ (1,0) [0|255]

//1569 GPS_CourseSpeed: 8 PCAN_GPS
        int32_t  GPS_Course;                    // 0|32@1- (1,0) [-3.4E+038|3.4E+038]
        int32_t  GPS_Speed;                     // 32|32@1- (1,0) [-3.4E+038|3.4E+038]

//1570 GPS_PositionLongitude: 7 PCAN_GPS
        int32_t  GPS_Longitude_Minutes;         // 0|32@1- (1,0) [-3.4E+038|3.4E+038]
        uint16_t GPS_Longitude_Degree;          // 32|16@1+ (1,0) [0|359]
        uint8_t  GPS_IndicatorEW;               // 48|8@1+ (1,0) [0|255]

//1571 GPS_PositionLatitude: 7 PCAN_GPS
        int32_t  GPS_Latitude_Minutes;          // 0|32@1- (1,0) [-3.4E+038|3.4E+038]
        uint16_t GPS_Latitude_Degree;           // 32|16@1+ (1,0) [0|359]
        uint8_t  GPS_IndicatorNS;               // 48|8@1+ (1,0) [0|255]

//1572 GPS_PositionAltitude: 4 PCAN_GPS
        int32_t  GPS_Altitude;                  // 0|32@1- (1,0) [-3.4E+038|3.4E+038]

//1573 GPS_Delusions_A: 8 PCAN_GPS
        int32_t  GPS_PDOP;                      // 0|32@1- (1,0) [-3.4E+038|3.4E+038]
        int32_t  GPS_HDOP;                      // 32|32@1- (1,0) [-3.4E+038|3.4E+038]

//1574 GPS_Delusions_B: 4 PCAN_GPS
        int32_t  GPS_VDOP;                      // 0|32@1- (1,0) [-3.4E+038|3.4E+038]

//1575 GPS_DateTime: 6 PCAN_GPS
        uint8_t  UTC_Year;                      // 0|8@1+ (1,0) [0|255]
        uint8_t  UTC_Month;                     // 8|8@1+ (1,0) [0|255]
        uint8_t  UTC_DayOfMonth;                // 16|8@1+ (1,0) [0|255]
        uint8_t  UTC_Hour;                      // 24|8@1+ (1,0) [0|255]
        uint8_t  UTC_Minute;                    // 32|8@1+ (1,0) [0|255]
        uint8_t  UTC_Second;                    // 40|8@1+ (1,0) [0|255]

//1584 IO: 1 PCAN_GPS
        uint8_t  Din1_Status;                   // 0|1@1+ (1,0) [0|1]
        uint8_t  Din2_Status;                   // 1|1@1+ (1,0) [0|1]
        uint8_t  Dout_Status;                   // 2|1@1+ (1,0) [0|1]
        uint8_t  SD_Present;                    // 3|1@1+ (1,0) [0|1]
        uint8_t  GPS_PowerStatus;               // 4|1@1+ (1,0) [0|1]
        uint8_t  Device_ID;                     // 5|3@1+ (1,0) [0|7]

//1600 RTC_DateTime: 8 PCAN_GPS
        uint8_t  RTC_Sec;                       // 0|8@1+ (1,0) [0|255]
        uint8_t  RTC_Min;                       // 8|8@1+ (1,0) [0|255]
        uint8_t  RTC_Hour;                      // 16|8@1+ (1,0) [0|255]
        uint8_t  RTC_DayOfWeek;                 // 24|8@1+ (1,0) [0|255]
        uint8_t  RTC_DayOfMonth;                // 32|8@1+ (1,0) [0|255]
        uint8_t  RTC_Month;                     // 40|8@1+ (1,0) [0|255]
        uint16_t RTC_Year;                      // 48|16@1+ (1,0) [0|65535]

//1616 Out_IO: 1 PCAN_GPS
        uint8_t  Dout_Set;                      // 0|1@1+ (1,0) [0|1]
        uint8_t  GPS_SetPower;                  // 1|1@1+ (1,0) [0|1]

//1617 Out_PowerOff: 1 PCAN_GPS
        uint8_t  Device_PowerOff;               // 0|1@1+ (1,0) [0|1]

//1618 Out_Gyro: 1 PCAN_GPS
        uint8_t  Gyro_SetScale;                 // 0|2@1+ (1,0) [0|3]

//1619 Out_BMC_AccScale: 1 PCAN_GPS
        uint8_t  Acc_SetScale;                  // 0|3@1+ (1,0) [0|7]

//1620 Out_SaveConfig: 1 PCAN_GPS
        uint8_t  Config_SaveToEEPROM;           // 0|1@1+ (1,0) [0|1]

//1621 Out_RTC_SetTime: 8 PCAN_GPS
        uint8_t  RTC_SetSec;                    // 0|8@1+ (1,0) [0|59]
        uint8_t  RTC_SetMin;                    // 8|8@1+ (1,0) [0|59]
        uint8_t  RTC_SetHour;                   // 16|8@1+ (1,0) [0|23]
        uint8_t  RTC_SetDayOfWeek;              // 24|8@1+ (1,0) [0|6]
        uint8_t  RTC_SetDayOfMonth;             // 32|8@1+ (1,0) [1|31]
        uint8_t  RTC_SetMonth;                  // 40|8@1+ (1,0) [1|12]
        uint16_t RTC_SetYear;                   // 48|16@1+ (1,0) [1900|2099]

//1622 Out_RTC_TimeFromGPS: 1 PCAN_GPS
        uint8_t  RTC_SetTimeFromGPS;            // 0|1@1+ (1,0) [0|1]

//1623 Out_Acc_FastCalibration: 4 PCAN_GPS
        uint8_t  Acc_SetCalibTarget_X;          // 0|2@1+ (1,0) [0|3]
        uint8_t  Acc_SetCalibTarget_Y;          // 8|2@1+ (1,0) [0|3]
        uint8_t  Acc_SetCalibTarget_Z;          // 16|2@1+ (1,0) [0|3]
        uint8_t  Acc_StartFastCalib;            // 24|1@1+ (1,0) [0|1]
};
*/

struct nrai_can_ai_read {
        int32_t  Rotation_X;                    // 0|32@1-  (1,0)    [-4000|4000]
        int32_t  Rotation_Y;                    // 32|32@1- (1,0)    [-4000|4000]
        int32_t  Rotation_Z;                    // 0|32@1-  (1,0)    [-4000|4000]
        int32_t  GPS_Course;                    // 0|32@1-  (1,0)    [-3.4E+038|3.4E+038]
        int32_t  GPS_Speed;                     // 32|32@1- (1,0)    [-3.4E+038|3.4E+038]
        int32_t  GPS_Longitude_Minutes;         // 0|32@1-  (1,0)    [-3.4E+038|3.4E+038]
        int32_t  GPS_Latitude_Minutes;          // 0|32@1-  (1,0)    [-3.4E+038|3.4E+038]
        int32_t  GPS_Altitude;                  // 0|32@1-  (1,0)    [-3.4E+038|3.4E+038]
        int32_t  GPS_PDOP;                      // 0|32@1-  (1,0)    [-3.4E+038|3.4E+038]
        int32_t  GPS_HDOP;                      // 32|32@1- (1,0)    [-3.4E+038|3.4E+038]
        int32_t  GPS_VDOP;                      // 0|32@1-  (1,0)    [-3.4E+038|3.4E+038]

        int16_t  ANGLE;                         // 0|16@1-  (0.1,0)  [-21|21]
        uint16_t ANGLE_MAX;                     // 16|16@1+ (0.1,0)  [0|21]
        int16_t  ANGLE_REQUEST;                 // 32|16@1- (0.1,0)  [-21|21]
        uint16_t FL_WHEEL_SPEED;                // 0|16@1+  (1,0)    [0|1250]
        uint16_t FR_WHEEL_SPEED;                // 16|16@1+ (1,0)    [0|1250]
        uint16_t RL_WHEEL_SPEED;                // 32|16@1+ (1,0)    [0|1250]
        uint16_t RR_WHEEL_SPEED;                // 48|16@1+ (1,0)    [0|1250]
        int16_t  FRONT_AXLE_TRQ;                // 0|16@1-  (0.1,0)  [-195|195]
        uint16_t FRONT_AXLE_TRQ_REQUEST;        // 16|16@1+ (0.1,0)  [0|195]
        uint16_t FRONT_AXLE_TRQ_MAX;            // 32|16@1+ (0.1,0)  [0|195]
        int16_t  REAR_AXLE_TRQ_Nm;              // 0|16@1-  (0.1,0)  [-195|195]
        uint16_t REAR_AXLE_TRQ_REQUEST_Nm;      // 16|16@1+ (0.1,0)  [0|195]
        uint16_t REAR_AXLE_TRQ_MAX_Nm;          // 32|16@1+ (0.1,0)  [0|195]
        uint16_t FL_PULSE_COUNT;                // 0|16@1+  (1,0)    [0|65535]
        uint16_t FR_PULSE_COUNT;                // 16|16@1+ (1,0)    [0|65535]
        uint16_t RL_PULSE_COUNT;                // 32|16@1+ (1,0)    [0|65535]
        uint16_t RR_PULSE_COUNT;                // 48|16@1+ (1,0)    [0|65535]
        int16_t  Acceleration_X;                // 0|16@1-  (3.91,0) [-20000|20000]
        int16_t  Acceleration_Y;                // 16|16@1- (3.91,0) [-20000|20000]
        int16_t  Acceleration_Z;                // 32|16@1- (3.91,0) [-20000|20000]
        int16_t  MagneticField_X;               // 0|16@1-  (0.3,0)  [-9830.4|9830.1]
        int16_t  MagneticField_Y;               // 16|16@1- (0.3,0)  [-9830.4|9830.1]
        int16_t  MagneticField_Z;               // 32|16@1- (0.3,0)  [-9830.4|9830.1]
        uint16_t GPS_Longitude_Degree;          // 32|16@1+ (1,0)    [0|359]
        uint16_t GPS_Latitude_Degree;           // 32|16@1+ (1,0)    [0|359]
        uint16_t RTC_Year;                      // 48|16@1+ (1,0)    [0|65535]
        uint16_t RTC_SetYear;                   // 48|16@1+ (1,0)    [1900|2099]

        uint8_t  HANDSHAKE;                     // 0|1@1-   (1,0)    [0|1]
        uint8_t  OP_STATE;                      // 4|4@1+   (1,0)    [0|15]
        uint8_t  SHUTDOWN_REQUEST;              // 8|1@1-   (1,0)    [0|1]
        uint8_t  AS_SWITCH_STATUS;              // 9|1@1-   (1,0)    [0|1]
        uint8_t  TS_SWITCH_STATUS;              // 10|1@1-  (1,0)    [0|1]
        uint8_t  GO_SIGNAL;                     // 11|1@1-  (1,0)    [0|1]
        uint8_t  STEERING_STATUS;               // 12|2@1+  (1,0)    [0|3]
        uint8_t  AS_STATE;                      // 16|4@1+  (1,0)    [0|7]
        uint8_t  AMI_STATE;                     // 20|4@1+  (1,0)    [0|15]
        uint8_t  FAULT_STATUS;                  // 24|1@1-  (1,0)    [0|1]
        uint8_t  WARNING_STATUS;                // 25|1@1-  (1,0)    [0|1]
        uint8_t  TS_STATE;                      // 28|4@1+  (1,0)    [0|15]
        uint8_t  WARN_BATT_TEMP_HIGH;           // 32|1@1-  (1,0)    [0|1]
        uint8_t  WARN_BATT_SOC_LOW;             // 33|1@1-  (1,0)    [0|1]
        uint8_t  EBS_STATE;                     // 36|4@1+  (1,0)    [0|15]
        uint8_t  AI_ESTOP_REQUEST;              // 40|1@1-  (1,0)    [0|1]
        uint8_t  HVIL_OPEN_FAULT;               // 41|1@1-  (1,0)    [0|1]
        uint8_t  HVIL_SHORT_FAULT;              // 42|1@1-  (1,0)    [0|1]
        uint8_t  EBS_FAULT;                     // 43|1@1-  (1,0)    [0|1]
        uint8_t  OFFBOARD_CHARGER_FAULT;        // 44|1@1-  (1,0)    [0|1]
        uint8_t  AI_COMMS_LOST;                 // 45|1@1-  (1,0)    [0|1]
        uint8_t  AUTONOMOUS_BRAKING_FAULT;      // 46|1@1-  (1,0)    [0|1]
        uint8_t  MISSION_STATUS_FAULT;          // 47|1@1-  (1,0)    [0|1]
        uint8_t  CHARGE_PROCEDURE_FAULT;        // 48|1@1-  (1,0)    [0|1]
        uint8_t  BMS_FAULT;                     // 49|1@1-  (1,0)    [0|1]
        uint8_t  BRAKE_PLAUSIBILITY_FAULT;      // 50|1@1-  (1,0)    [0|1]
        uint8_t  SHUTDOWN_CAUSE;                // 56|8@1+  (1,0)    [0|255]
        uint8_t  HYD_PRESS_F_pct;               // 0|8@1+   (0.5,0)  [0|100]
        uint8_t  HYD_PRESS_F_REQ_pct;           // 8|8@1+   (0.5,0)  [0|100]
        uint8_t  HYD_PRESS_R_pct;               // 16|8@1+  (0.5,0)  [0|100]
        uint8_t  HYD_PRESS_R_REQ_pct;           // 24|8@1+  (0.5,0)  [0|100]
        uint8_t  STATUS_BRK;                    // 32|4@1+  (1,0)    [0|15]
        uint8_t  STATUS_EBS;                    // 36|4@1+  (1,0)    [0|15]
        int8_t   Temperature;                   // 48|8@1-  (0.5,24) [-40|87.5]
        uint8_t  VerticalAxis;                  // 56|2@1+  (1,0)    [0|3]
        uint8_t  Orientation;                   // 58|3@1+  (1,0)    [0|7]
        uint8_t  GPS_AntennaStatus;             // 0|8@1+   (1,0)    [0|255]
        uint8_t  GPS_NumSatellites;             // 8|8@1+   (1,0)    [0|255]
        uint8_t  GPS_NavigationMethod;          // 16|8@1+  (1,0)    [0|255]
        uint8_t  GPS_IndicatorEW;               // 48|8@1+  (1,0)    [0|255]
        uint8_t  GPS_IndicatorNS;               // 48|8@1+  (1,0)    [0|255]
        uint8_t  UTC_Year;                      // 0|8@1+   (1,0)    [0|255]
        uint8_t  UTC_Month;                     // 8|8@1+   (1,0)    [0|255]
        uint8_t  UTC_DayOfMonth;                // 16|8@1+  (1,0)    [0|255]
        uint8_t  UTC_Hour;                      // 24|8@1+  (1,0)    [0|255]
        uint8_t  UTC_Minute;                    // 32|8@1+  (1,0)    [0|255]
        uint8_t  UTC_Second;                    // 40|8@1+  (1,0)    [0|255]
        uint8_t  Din1_Status;                   // 0|1@1+   (1,0)    [0|1]
        uint8_t  Din2_Status;                   // 1|1@1+   (1,0)    [0|1]
        uint8_t  Dout_Status;                   // 2|1@1+   (1,0)    [0|1]
        uint8_t  SD_Present;                    // 3|1@1+   (1,0)    [0|1]
        uint8_t  GPS_PowerStatus;               // 4|1@1+   (1,0)    [0|1]
        uint8_t  Device_ID;                     // 5|3@1+   (1,0)    [0|7]
        uint8_t  RTC_Sec;                       // 0|8@1+   (1,0)    [0|255]
        uint8_t  RTC_Min;                       // 8|8@1+   (1,0)    [0|255]
        uint8_t  RTC_Hour;                      // 16|8@1+  (1,0)    [0|255]
        uint8_t  RTC_DayOfWeek;                 // 24|8@1+  (1,0)    [0|255]
        uint8_t  RTC_DayOfMonth;                // 32|8@1+  (1,0)    [0|255]
        uint8_t  RTC_Month;                     // 40|8@1+  (1,0)    [0|255]
        uint8_t  Dout_Set;                      // 0|1@1+   (1,0)    [0|1]
        uint8_t  GPS_SetPower;                  // 1|1@1+   (1,0)    [0|1]
        uint8_t  Device_PowerOff;               // 0|1@1+   (1,0)    [0|1]
        uint8_t  Gyro_SetScale;                 // 0|2@1+   (1,0)    [0|3]
        uint8_t  Acc_SetScale;                  // 0|3@1+   (1,0)    [0|7]
        uint8_t  Config_SaveToEEPROM;           // 0|1@1+   (1,0)    [0|1]
        uint8_t  RTC_SetSec;                    // 0|8@1+   (1,0)    [0|59]
        uint8_t  RTC_SetMin;                    // 8|8@1+   (1,0)    [0|59]
        uint8_t  RTC_SetHour;                   // 16|8@1+  (1,0)    [0|23]
        uint8_t  RTC_SetDayOfWeek;              // 24|8@1+  (1,0)    [0|6]
        uint8_t  RTC_SetDayOfMonth;             // 32|8@1+  (1,0)    [1|31]
        uint8_t  RTC_SetMonth;                  // 40|8@1+  (1,0)    [1|12]
        uint8_t  RTC_SetTimeFromGPS;            // 0|1@1+   (1,0)    [0|1]
        uint8_t  Acc_SetCalibTarget_X;          // 0|2@1+   (1,0)    [0|3]
        uint8_t  Acc_SetCalibTarget_Y;          // 8|2@1+   (1,0)    [0|3]
        uint8_t  Acc_SetCalibTarget_Z;          // 16|2@1+  (1,0)    [0|3]
        uint8_t  Acc_StartFastCalib;            // 24|1@1+  (1,0)    [0|1]
};

/*
 * The following funcitons ALL unpack the data from the CAN frame corresponding 
 * to their name into the nrai_can_ai_read struct.
 * All functions return:
 *      -1 on error
 *       0 on success
 * On error the struct nrai_can_ai_read AND the can_frame are not modified.
 */
NRAI_FT int32_t nrai_can_unpack_VCU2AI_Status           (struct nrai_can_ai_read *, struct can_frame *);
NRAI_FT int32_t nrai_can_unpack_VCU2AI_Steer            (struct nrai_can_ai_read *, struct can_frame *);
NRAI_FT int32_t nrai_can_unpack_VCU2AI_Speeds           (struct nrai_can_ai_read *, struct can_frame *);
NRAI_FT int32_t nrai_can_unpack_VCU2AI_Brake            (struct nrai_can_ai_read *, struct can_frame *);
NRAI_FT int32_t nrai_can_unpack_VCU2AI_Drive_F          (struct nrai_can_ai_read *, struct can_frame *);
NRAI_FT int32_t nrai_can_unpack_VCU2AI_Drive_R          (struct nrai_can_ai_read *, struct can_frame *);
NRAI_FT int32_t nrai_can_unpack_VCU2AI_Wheel_counts     (struct nrai_can_ai_read *, struct can_frame *);

NRAI_FT int32_t nrai_can_unpack_BMC_Acceleration        (struct nrai_can_ai_read *, struct can_frame *);
NRAI_FT int32_t nrai_can_unpack_BMC_MagneticField       (struct nrai_can_ai_read *, struct can_frame *);
NRAI_FT int32_t nrai_can_unpack_L3GD20_Rotation_A       (struct nrai_can_ai_read *, struct can_frame *);
NRAI_FT int32_t nrai_can_unpack_L3GD20_Rotation_B       (struct nrai_can_ai_read *, struct can_frame *);

NRAI_FT int32_t nrai_can_unpack_GPS_Status              (struct nrai_can_ai_read *, struct can_frame *);
NRAI_FT int32_t nrai_can_unpack_GPS_CourseSpeed         (struct nrai_can_ai_read *, struct can_frame *);
NRAI_FT int32_t nrai_can_unpack_GPS_PositionLongitude   (struct nrai_can_ai_read *, struct can_frame *);
NRAI_FT int32_t nrai_can_unpack_GPS_PositionLatitude    (struct nrai_can_ai_read *, struct can_frame *);
NRAI_FT int32_t nrai_can_unpack_GPS_PositionAltitude    (struct nrai_can_ai_read *, struct can_frame *);
NRAI_FT int32_t nrai_can_unpack_GPS_Delusions_A         (struct nrai_can_ai_read *, struct can_frame *);
NRAI_FT int32_t nrai_can_unpack_GPS_Delusions_B         (struct nrai_can_ai_read *, struct can_frame *);
NRAI_FT int32_t nrai_can_unpack_GPS_DateTime            (struct nrai_can_ai_read *, struct can_frame *);
NRAI_FT int32_t nrai_can_unpack_IO                      (struct nrai_can_ai_read *, struct can_frame *);

NRAI_FT int32_t nrai_can_unpack_RTC_DateTime            (struct nrai_can_ai_read *, struct can_frame *);
NRAI_FT int32_t nrai_can_unpack_Out_IO                  (struct nrai_can_ai_read *, struct can_frame *);
NRAI_FT int32_t nrai_can_unpack_Out_PowerOff            (struct nrai_can_ai_read *, struct can_frame *);
NRAI_FT int32_t nrai_can_unpack_Out_Gyro                (struct nrai_can_ai_read *, struct can_frame *);
NRAI_FT int32_t nrai_can_unpack_Out_BMC_AccScale        (struct nrai_can_ai_read *, struct can_frame *);
NRAI_FT int32_t nrai_can_unpack_Out_SaveConfig          (struct nrai_can_ai_read *, struct can_frame *);
NRAI_FT int32_t nrai_can_unpack_Out_RTC_SetTime         (struct nrai_can_ai_read *, struct can_frame *);
NRAI_FT int32_t nrai_can_unpack_Out_RTC_TimeFromGPS     (struct nrai_can_ai_read *, struct can_frame *);
NRAI_FT int32_t nrai_can_unpack_Out_Acc_FastCalibration (struct nrai_can_ai_read *, struct can_frame *);

/* ----------------------- AI_READ_END ----------------------- */



/* ----------------------- AI_WRITE_START ----------------------- */

//BO_ 1296 AI2VCU_Status: 8 AI
// SG_ HANDSHAKE : 0|1@1- (1,0) [0|1] ""  LOG,OPUS,VCU
// SG_ ESTOP_REQUEST : 8|1@1- (1,0) [0|1] ""  LOG,OPUS,VCU
// SG_ MISSION_STATUS : 12|2@1+ (1,0) [0|3] ""  LOG,OPUS,VCU
// SG_ DIRECTION_REQUEST : 14|2@1+ (1,0) [0|3] ""  LOG,OPUS,VCU
// SG_ LAP_COUNTER : 16|4@1+ (1,0) [0|15] ""  LOG,OPUS,VCU
// SG_ CONES_COUNT_ACTUAL : 24|8@1+ (1,0) [0|255] ""  LOG,OPUS,VCU
// SG_ CONES_COUNT_ALL : 32|16@1+ (1,0) [0|65535] ""  LOG,OPUS,VCU
// SG_ VEH_SPEED_ACTUAL : 48|8@1+ (1,0) [0|255] "km/h"  LOG,OPUS,VCU
// SG_ VEH_SPEED_DEMAND : 56|8@1+ (1,0) [0|255] "km/h"  LOG,OPUS,VCU

//BO_ 1299 AI2VCU_Steer: 2 AI
// SG_ STEER_REQUEST : 0|16@1- (0.1,0) [-21|21] "deg"  LOG,OPUS,VCU

//BO_ 1300 AI2VCU_Brake: 2 AI
// SG_ HYD_PRESS_F_REQ_pct : 0|8@1+ (0.5,0) [0|100] "%"  LOG,OPUS,VCU
// SG_ HYD_PRESS_R_REQ_pct : 8|8@1+ (0.5,0) [0|100] "%"  LOG,OPUS,VCU

//BO_ 1297 AI2VCU_Drive_F: 4 AI
// SG_ FRONT_AXLE_TRQ_REQUEST : 0|16@1+ (0.1,0) [0|195] "Nm"  OPUS,LOG,VCU
// SG_ FRONT_MOTOR_SPEED_MAX : 16|16@1+ (1,0) [0|4000] "rpm"  OPUS,LOG,VCU

//BO_ 1298 AI2VCU_Drive_R: 4 AI
// SG_ REAR_AXLE_TRQ_REQUEST : 0|16@1+ (0.1,0) [0|195] "Nm"  LOG,OPUS,VCU
// SG_ REAR_MOTOR_SPEED_MAX : 16|16@1+ (1,0) [0|4000] "rpm"  LOG,OPUS,VCU

struct nrai_can_ai_write {
        uint16_t CONES_COUNT_ALL;               // 32|16@1+ (1,0)   [0|65535]
        int16_t  STEER_REQUEST;                 // 0|16@1-  (0.1,0) [-21|21]
        uint16_t FRONT_AXLE_TRQ_REQUEST;        // 0|16@1+  (0.1,0) [0|195]
        uint16_t FRONT_MOTOR_SPEED_MAX;         // 16|16@1+ (1,0)   [0|4000]
        uint16_t REAR_AXLE_TRQ_REQUEST;         // 0|16@1+  (0.1,0) [0|195]
        uint16_t REAR_MOTOR_SPEED_MAX;          // 16|16@1+ (1,0)   [0|4000]
        uint8_t  HANDSHAKE;                     // 0|1@1-   (1,0)   [0|1]
        uint8_t  ESTOP_REQUEST;                 // 8|1@1-   (1,0)   [0|1]
        uint8_t  MISSION_STATUS;                // 12|2@1+  (1,0)   [0|3]
        uint8_t  DIRECTION_REQUEST;             // 14|2@1+  (1,0)   [0|3]
        uint8_t  LAP_COUNTER;                   // 16|4@1+  (1,0)   [0|15]
        uint8_t  CONES_COUNT_ACTUAL;            // 24|8@1+  (1,0)   [0|255]
        uint8_t  VEH_SPEED_ACTUAL;              // 48|8@1+  (1,0)   [0|255]
        uint8_t  VEH_SPEED_DEMAND;              // 56|8@1+  (1,0)   [0|255]
        uint8_t  HYD_PRESS_F_REQ_pct;           // 0|8@1+   (0.5,0) [0|100]
        uint8_t  HYD_PRESS_R_REQ_pct;           // 8|8@1+   (0.5,0) [0|100]
};

/*
 * The following funcitons ALL create the can_frame corresponding to their
 * funciton name from the data contained in the nrai_can_ai_write struct.
 * All functions return:
 *     -1 on error
 *      0 on success
 * On error the resulting can_frame is undefined and not safe to use.
 * Values out of range will be clamped to their higher or lower bound, a stat
 * clamp is performed silently and not reported as an error.
 * */
NRAI_FT int32_t nrai_can_mkframe_AI2VCU_Status( struct nrai_can_ai_write *, struct can_frame *);
NRAI_FT int32_t nrai_can_mkframe_AI2VCU_Steer(  struct nrai_can_ai_write *, struct can_frame *);
NRAI_FT int32_t nrai_can_mkframe_AI2VCU_Brake(  struct nrai_can_ai_write *, struct can_frame *);
NRAI_FT int32_t nrai_can_mkframe_AI2VCU_Drive_F(struct nrai_can_ai_write *, struct can_frame *);
NRAI_FT int32_t nrai_can_mkframe_AI2VCU_Drive_R(struct nrai_can_ai_write *, struct can_frame *);

/* ----------------------- AI_WRITE_END ----------------------- */





/* ----------------------- EXTERNAL_START ----------------------- */

//BO_ 1280 VCU2LOG_Dynamics1: 8 VCU
// SG_ Speed_actual : 0|8@1+ (1,0) [0|255] "km/h"  LOG,OPUS
// SG_ Speed_target : 8|8@1+ (1,0) [0|255] "km/h"  LOG,OPUS
// SG_ Steer_actual : 16|8@1- (0.5,0) [-64|63.5] "deg"  LOG,OPUS
// SG_ Steer_target : 24|8@1- (0.5,0) [-64|63.5] "deg"  LOG,OPUS
// SG_ Brake_actual_pct : 32|8@1+ (1,0) [0|100] "%"  LOG,OPUS
// SG_ Brake_target_pct : 40|8@1+ (1,0) [0|100] "%"  LOG,OPUS
// SG_ Drive_trq_actual_pct : 48|8@1+ (1,0) [0|100] "%"  LOG,OPUS
// SG_ Drive_trq_target_pct : 56|8@1+ (1,0) [0|100] "%"  LOG,OPUS

//BO_ 1282 VCU2LOG_Status: 5 VCU
// SG_ State_ASSI : 0|3@1+ (1,0) [0|7] ""  LOG,OPUS
// SG_ State_EBS : 3|2@1+ (1,0) [0|3] ""  LOG,OPUS
// SG_ AMI_STATE : 5|3@1+ (1,0) [0|7] ""  LOG,OPUS
// SG_ State_steering : 8|1@1- (1,0) [0|1] ""  LOG,OPUS
// SG_ State_service_brake : 9|2@1+ (1,0) [0|3] ""  LOG,OPUS
// SG_ Lap_counter : 11|4@1+ (1,0) [0|15] ""  LOG,OPUS
// SG_ Cones_count_actual : 15|8@1+ (1,0) [0|255] ""  LOG,OPUS
// SG_ Cones_count_all : 23|17@1+ (1,0) [0|131071] ""  LOG,OPUS

//BO_ 1281 AI2LOG_Dynamics2: 6 PCAN_GPS
// SG_ Accel_longitudinal_mps2 : 0|16@1- (0.00195313,0) [-64|63.998] "m/s^2"  OPUS,LOG
// SG_ Accel_lateral_mps2 : 16|16@1- (0.00195313,0) [-64|63.998] "m/s^2"  OPUS,LOG
// SG_ Yaw_rate_degps : 32|16@1- (0.0078125,0) [-256|255.992] "deg/s"  OPUS,LOG

//BO_ 129 RES_EMCY: 8 RES
// SG_ RES_EEC : 0|16@1+ (1,0) [0|65535] ""  LOG,OPUS,VCU
// SG_ RES_ER : 16|8@1+ (1,0) [0|255] ""  LOG,OPUS,VCU
// SG_ RES_Fault_ID : 24|16@1+ (1,0) [0|65535] ""  LOG,OPUS,VCU

//BO_ 385 RES_TPDO: 8 RES
// SG_ ESTOP_STATUS : 0|1@1- (1,0) [0|1] ""  LOG,OPUS,VCU
// SG_ K2_SWITCH_GO_STATUS : 1|1@1- (1,0) [0|1] ""  LOG,OPUS,VCU
// SG_ K3_SWITCH_STATUS : 2|1@1- (1,0) [0|1] ""  LOG,OPUS,VCU
// SG_ RADIO_STRENGTH : 48|8@1+ (1,0) [0|255] "%"  LOG,OPUS,VCU

//BO_ 291 OPUS2VCU_FB: 1 OPUS
// SG_ AMI_STATE : 0|4@1+ (1,0) [0|15] ""  OPUS,LOG,VCU
// SG_ BRK_DISCH_REQ : 4|1@1- (1,0) [0|1] ""  OPUS,LOG,VCU
// SG_ CHARGE_REQ : 5|1@1- (1,0) [0|1] "" Vector__XXX

//BO_ 644 UMB_PDO2: 4 JOYSTICK
// SG_ YPOS : 0|16@1+ (1,0) [0|255] ""  LOG,OPUS,VCU
// SG_ XPOS : 16|16@1+ (1,0) [0|255] ""  LOG,OPUS,VCU

//BO_ 132 UMB_EMCY: 8 JOYSTICK
// SG_ UMB_EEC : 0|16@1+ (1,0) [0|65535] ""  LOG,OPUS,VCU
// SG_ UMB_ER : 16|8@1+ (1,0) [0|255] ""  LOG,OPUS,VCU
// SG_ UMB_Fault_ID : 24|16@1+ (1,0) [0|65535] ""  LOG,OPUS,VCU

//BO_ 128 SYNC: 0 VCU

//BO_ 388 UMB_PDO1: 2 JOYSTICK
// SG_ BTN_TOP : 0|1@1- (1,0) [0|1] ""  LOG,OPUS,VCU
// SG_ BTN_BTM : 3|1@1- (1,0) [0|1] ""  LOG,OPUS,VCU

//BO_ 769 VCU2OPUS_2: 8 VCU
// SG_ EBS_FRONT_ACC_PRESS : 0|16@1- (0.01,0) [-327.68|327.67] "Bar"  OPUS
// SG_ EBS_REAR_ACC_PRESS : 16|16@1- (0.01,0) [-327.68|327.67] "Bar"  OPUS
// SG_ EBS_FRONT_CALIPER_PRESS : 32|16@1- (0.01,0) [-327.68|327.67] "Bar"  OPUS
// SG_ EBS_REAR_CALIPER_PRESS : 48|16@1- (0.01,0) [-327.68|327.67] "Bar"  OPUS

//BO_ 768 VCU2OPUS_1: 8 VCU
// SG_ KL15_ADC : 0|16@1+ (0.001,0) [0|65.535] "V"  OPUS
// SG_ VBAT_ADC : 16|16@1+ (0.001,0) [0|65.535] "V"  OPUS
// SG_ AUX_HV : 32|16@1+ (0.001,0) [0|65.535] "V"  OPUS
// SG_ LEM_500A_ADC : 48|16@1+ (0.25,-625) [-625|15758.75] "A"  OPUS

//BO_ 0 NMT: 2 VCU
// SG_ CMD : 0|8@1+ (1,0) [0|255] ""  LOG,OPUS,RES,JOYSTICK
// SG_ NODE_ID : 8|8@1+ (1,0) [0|255] ""  LOG,OPUS,RES,JOYSTICK

/* ----------------------- EXTERNAL_END ----------------------- */

/* ----------------------- WTF?????_START ----------------------- */

//BO_ 3221225472 VECTOR__INDEPENDENT_SIG_MSG: 0 Vector__XXX
// SG_ AI2VCU_Drive_F_Count : 0|8@1+ (1,0) [0|255] "" Vector__XXX
// SG_ AI2VCU_Brake_Count : 0|8@1+ (1,0) [0|255] "" Vector__XXX
// SG_ AI2VCU_Steer_Count : 0|8@1+ (1,0) [0|255] "" Vector__XXX
// SG_ AI2VCU_Drive_R_Count : 0|8@1+ (1,0) [0|255] "" Vector__XXX
// SG_ AI2VCU_Status_Count : 0|8@1+ (1,0) [0|255] "" Vector__XXX

//BO_ 770 VCU2OPUS_3: 8 VCU
// SG_ EBS_TEST_BITFIELD : 0|16@1+ (1,0) [0|65535] "" Vector__XXX
// SG_ EBS_TEST_BITFIELD_LATCHED : 16|16@1+ (1,0) [0|65535] "" Vector__XXX

/* ----------------------- WTF?????_END ----------------------- */

#endif //NRAI_CAN_API
