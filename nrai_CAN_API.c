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

#include <stdint.h>
#include <stdlib.h>

#include "nrai_CAN_API.h"

#define NRAI_CAN_UNPACK(name, ...)                                                          \
  NRAI_FT int32_t nrai_can_unpack_##name(struct nrai_can_ai_read* s, struct can_frame* f) { \
    if ((s == NULL) || (f == NULL))                                                         \
        return -1;                                                                          \
    if (f->can_id != NRAI_CAN_ID_##name)                                                    \
        return -1;                                                                          \
                                                                                            \
    __VA_ARGS__                                                                             \
    return 0;                                                                               \
  }

#define NRAI_CAN_MKFRAME(name, ...)                                                           \
  NRAI_FT int32_t nrai_can_mkframe_##name(struct nrai_can_ai_write* s, struct can_frame* f) { \
    if ((s == NULL) || (f == NULL))                                                           \
        return -1;                                                                            \
    f->can_id = NRAI_CAN_ID_##name;                                                           \
                                                                                              \
    __VA_ARGS__                                                                               \
    return 0;                                                                                 \
  }

static inline int32_t
CLAMP(int32_t value, int32_t min, int32_t max)
{
        if (value < min){
                return min;
        } else if (value > max){
                return max;
        }
        return value;
}

NRAI_CAN_UNPACK(VCU2AI_Status, 
        s->HANDSHAKE                    = (f->data[0] & 0b00000001);
        s->OP_STATE                     = (f->data[0] & 0b11110000) >> 4;
        s->SHUTDOWN_REQUEST             = (f->data[1] & 0b00000001);
        s->AS_SWITCH_STATUS             = (f->data[1] & 0b00000010) >> 1;
        s->TS_SWITCH_STATUS             = (f->data[1] & 0b00000100) >> 2;
        s->GO_SIGNAL                    = (f->data[1] & 0b00001000) >> 3;
        s->STEERING_STATUS              = (f->data[1] & 0b00110000) >> 4;
        s->AS_STATE                     = (f->data[2] & 0b00001111);
        s->AMI_STATE                    = (f->data[2] & 0b11110000) >> 4;
        s->FAULT_STATUS                 = (f->data[3] & 0b00000001);
        s->WARNING_STATUS               = (f->data[3] & 0b00000010) >> 1;
        s->TS_STATE                     = (f->data[3] & 0b11110000) >> 4;
        s->WARN_BATT_TEMP_HIGH          = (f->data[4] & 0b00000001);
        s->WARN_BATT_SOC_LOW            = (f->data[4] & 0b00000010) >> 1;
        s->EBS_STATE                    = (f->data[4] & 0b11110000) >> 4;
        s->AI_ESTOP_REQUEST             = (f->data[5] & 0b00000001);
        s->HVIL_OPEN_FAULT              = (f->data[5] & 0b00000010) >> 1;
        s->HVIL_SHORT_FAULT             = (f->data[5] & 0b00000100) >> 2;
        s->EBS_FAULT                    = (f->data[5] & 0b00001000) >> 3;
        s->OFFBOARD_CHARGER_FAULT       = (f->data[5] & 0b00010000) >> 4;
        s->AI_COMMS_LOST                = (f->data[5] & 0b00100000) >> 5;
        s->AUTONOMOUS_BRAKING_FAULT     = (f->data[5] & 0b01000000) >> 6;
        s->MISSION_STATUS_FAULT         = (f->data[5] & 0b10000000) >> 7;
        s->CHARGE_PROCEDURE_FAULT       = (f->data[6] & 0b00000001);
        s->BMS_FAULT                    = (f->data[6] & 0b00000010) >> 1;
        s->BRAKE_PLAUSIBILITY_FAULT     = (f->data[6] & 0b00000100) >> 2;
        s->SHUTDOWN_CAUSE               = (f->data[7]);
)

NRAI_CAN_UNPACK(VCU2AI_Steer, 
        s->ANGLE                = ((uint16_t)f->data[1] << 8) | f->data[0];
        s->ANGLE_MAX            = ((uint16_t)f->data[3] << 8) | f->data[2];
        s->ANGLE_REQUEST        = ((uint16_t)f->data[5] << 8) | f->data[4];
)

NRAI_CAN_UNPACK(VCU2AI_Speeds, 
        s->FL_WHEEL_SPEED = ((uint16_t)f->data[1] << 8) | f->data[0];
        s->FR_WHEEL_SPEED = ((uint16_t)f->data[3] << 8) | f->data[2];
        s->RL_WHEEL_SPEED = ((uint16_t)f->data[5] << 8) | f->data[4];
        s->RR_WHEEL_SPEED = ((uint16_t)f->data[7] << 8) | f->data[6];
)

NRAI_CAN_UNPACK(VCU2AI_Brake, 
        s->HYD_PRESS_F_pct         = f->data[0];
        s->HYD_PRESS_F_REQ_pct     = f->data[1];
        s->HYD_PRESS_R_pct         = f->data[2];
        s->HYD_PRESS_R_REQ_pct     = f->data[3];
        s->STATUS_BRK              = (f->data[4] & 0b00001111);
        s->STATUS_EBS              = (f->data[4] & 0b11110000) >> 4;
)

NRAI_CAN_UNPACK(VCU2AI_Drive_F, 
        s->FRONT_AXLE_TRQ               = ((uint16_t)f->data[1] << 8) | f->data[0];
        s->FRONT_AXLE_TRQ_REQUEST       = ((uint16_t)f->data[3] << 8) | f->data[2];
        s->FRONT_AXLE_TRQ_MAX           = ((uint16_t)f->data[5] << 8) | f->data[4];
)

NRAI_CAN_UNPACK(VCU2AI_Drive_R, 
        s->REAR_AXLE_TRQ_Nm             = ((uint16_t)f->data[1] << 8) | f->data[0];
        s->REAR_AXLE_TRQ_REQUEST_Nm     = ((uint16_t)f->data[3] << 8) | f->data[2];
        s->REAR_AXLE_TRQ_MAX_Nm         = ((uint16_t)f->data[5] << 8) | f->data[4];
)

NRAI_CAN_UNPACK(VCU2AI_Wheel_counts, 
        s->FL_PULSE_COUNT = ((uint16_t)f->data[1] << 8) | f->data[0];
        s->FR_PULSE_COUNT = ((uint16_t)f->data[3] << 8) | f->data[2];
        s->RL_PULSE_COUNT = ((uint16_t)f->data[5] << 8) | f->data[4];
        s->RR_PULSE_COUNT = ((uint16_t)f->data[7] << 8) | f->data[6];
)


NRAI_CAN_UNPACK(BMC_Acceleration, 
        s->Acceleration_X       = ((uint16_t)f->data[1] << 8) | f->data[0];
        s->Acceleration_Y       = ((uint16_t)f->data[3] << 8) | f->data[2];
        s->Acceleration_Z       = ((uint16_t)f->data[5] << 8) | f->data[4];
        s->Temperature          = f->data[6];
        s->VerticalAxis         = (f->data[7] & 0b00000011);
        s->Orientation          = (f->data[7] & 0b00011100) >> 2;
)

NRAI_CAN_UNPACK(BMC_MagneticField, 
        s->MagneticField_X = ((uint16_t)f->data[1] << 8) | f->data[0];
        s->MagneticField_Y = ((uint16_t)f->data[3] << 8) | f->data[2];
        s->MagneticField_Z = ((uint16_t)f->data[5] << 8) | f->data[4];
)

NRAI_CAN_UNPACK(L3GD20_Rotation_A, 
        s->Rotation_X = ((uint32_t)f->data[3] << 24) |
                                       ((uint32_t)f->data[2] << 16) |
                                       ((uint16_t)f->data[1] << 8) | f->data[0];
        s->Rotation_Y = ((uint32_t)f->data[7] << 24) |
                                       ((uint32_t)f->data[6] << 16) |
                                       ((uint16_t)f->data[5] << 8) | f->data[4];
)

NRAI_CAN_UNPACK(L3GD20_Rotation_B, 
        s->Rotation_Z = ((uint32_t)f->data[3] << 24) |
                                       ((uint32_t)f->data[2] << 16) |
                                       ((uint16_t)f->data[1] << 8) | f->data[0];
)

NRAI_CAN_UNPACK(GPS_Status, 
        s->GPS_AntennaStatus    = f->data[0];
        s->GPS_NumSatellites    = f->data[1];
        s->GPS_NavigationMethod = f->data[2];
)

NRAI_CAN_UNPACK(GPS_CourseSpeed, 
        s->GPS_Course   = ((uint32_t)f->data[3] << 24) |
                                       ((uint32_t)f->data[2] << 16) |
                                       ((uint16_t)f->data[1] << 8) | f->data[0];

        s->GPS_Speed    = ((uint32_t)f->data[7] << 24) |
                                       ((uint32_t)f->data[6] << 16) |
                                       ((uint16_t)f->data[5] << 8) | f->data[4];

)

NRAI_CAN_UNPACK(GPS_PositionLongitude, 
        s->GPS_Longitude_Minutes = ((uint32_t)f->data[3] << 24) |
                                       ((uint32_t)f->data[2] << 16) |
                                       ((uint16_t)f->data[1] << 8) | f->data[0];
        s->GPS_Longitude_Degree  = ((uint16_t)f->data[5] << 8) | f->data[4];
        s->GPS_IndicatorEW       = f->data[6];
)

NRAI_CAN_UNPACK(GPS_PositionLatitude, 
        s->GPS_Latitude_Minutes = ((uint32_t)f->data[3] << 24) |
                                       ((uint32_t)f->data[2] << 16) |
                                       ((uint16_t)f->data[1] << 8) | f->data[0];
        s->GPS_Latitude_Degree  = ((uint16_t)f->data[5] << 8) | f->data[4];
        s->GPS_IndicatorNS      = f->data[6];
)

NRAI_CAN_UNPACK(GPS_PositionAltitude, 
        s->GPS_Altitude = ((uint32_t)f->data[3] << 24) |
                                       ((uint32_t)f->data[2] << 16) |
                                       ((uint16_t)f->data[1] << 8) | f->data[0];
)

NRAI_CAN_UNPACK(GPS_Delusions_A, 
        s->GPS_PDOP = ((uint32_t)f->data[3] << 24) |
                                       ((uint32_t)f->data[2] << 16) |
                                       ((uint16_t)f->data[1] << 8) | f->data[0];

        s->GPS_HDOP = ((uint32_t)f->data[7] << 24) |
                                       ((uint32_t)f->data[6] << 16) |
                                       ((uint16_t)f->data[5] << 8) | f->data[4];
)

NRAI_CAN_UNPACK(GPS_Delusions_B, 
        s->GPS_VDOP = ((uint32_t)f->data[3] << 24) |
                                       ((uint32_t)f->data[2] << 16) |
                                       ((uint16_t)f->data[1] << 8) | f->data[0];
)

NRAI_CAN_UNPACK(GPS_DateTime,
        s->UTC_Year       = f->data[0];
        s->UTC_Month      = f->data[1];
        s->UTC_DayOfMonth = f->data[2];
        s->UTC_Hour       = f->data[3];
        s->UTC_Minute     = f->data[4];
        s->UTC_Second     = f->data[5];
)

NRAI_CAN_UNPACK(IO, 
        s->Din1_Status     = (f->data[0] & 0b00000001);
        s->Din2_Status     = (f->data[0] & 0b00000010) >> 1;
        s->Dout_Status     = (f->data[0] & 0b00000100) >> 2;
        s->SD_Present      = (f->data[0] & 0b00001000) >> 3;
        s->GPS_PowerStatus = (f->data[0] & 0b00010000) >> 4;
        s->Device_ID       = (f->data[0] & 0b11100000) >> 5;
)

NRAI_CAN_UNPACK(RTC_DateTime, 
        s->RTC_Sec        = f->data[0];
        s->RTC_Min        = f->data[1];
        s->RTC_Hour       = f->data[2];
        s->RTC_DayOfWeek  = f->data[3];
        s->RTC_DayOfMonth = f->data[4];
        s->RTC_Month      = f->data[5];
        s->RTC_Year       = ((uint16_t)f->data[7] << 8) | f->data[6];
)

NRAI_CAN_UNPACK(Out_IO, 
        s->Dout_Set     = (f->data[0] & 0b00000001);
        s->GPS_SetPower = (f->data[0] & 0b00000010) >> 2;
)

NRAI_CAN_UNPACK(Out_PowerOff, 
        s->Device_PowerOff = (f->data[0] & 0b00000001);
)

NRAI_CAN_UNPACK(Out_Gyro, 
        s->Gyro_SetScale = (f->data[0] & 0b00000011);
)

NRAI_CAN_UNPACK(Out_BMC_AccScale, 
        s->Acc_SetScale = (f->data[0] & 0b00000111);
)

NRAI_CAN_UNPACK(Out_SaveConfig, 
        s->Config_SaveToEEPROM = (f->data[0] & 0b00000001);
)

NRAI_CAN_UNPACK(Out_RTC_SetTime, 
        s->RTC_SetSec        = f->data[0];
        s->RTC_SetMin        = f->data[1];
        s->RTC_SetHour       = f->data[2];
        s->RTC_SetDayOfWeek  = f->data[3];
        s->RTC_SetDayOfMonth = f->data[4];
        s->RTC_SetMonth      = f->data[5];
        s->RTC_SetYear       = ((uint16_t)f->data[7] << 8) | f->data[6];
)

NRAI_CAN_UNPACK(Out_RTC_TimeFromGPS, 
        s->RTC_SetTimeFromGPS = (f->data[0] & 0b00000001);
)

NRAI_CAN_UNPACK(Out_Acc_FastCalibration, 
        s->Acc_SetCalibTarget_X = (f->data[0] & 0b00000011);
        s->Acc_SetCalibTarget_Y = (f->data[1] & 0b00000011);
        s->Acc_SetCalibTarget_Z = (f->data[2] & 0b00000011);
        s->Acc_StartFastCalib   = (f->data[3] & 0b00000001);
)




NRAI_CAN_MKFRAME(AI2VCU_Status,
        f->len = 8;

        f->data[0] =  (s->HANDSHAKE         & 0b00000001);
        f->data[1] =  (s->ESTOP_REQUEST     & 0b00000001)       |
                     ((s->MISSION_STATUS    & 0b00000011) << 4) |
                     ((s->DIRECTION_REQUEST & 0b00000011) << 6);
        f->data[2] =  (s->LAP_COUNTER        & 0b00001111);
        f->data[3] =   s->CONES_COUNT_ACTUAL;
        f->data[4] =  (s->CONES_COUNT_ALL       & 0x00FF);
        f->data[5] = ((s->CONES_COUNT_ALL >> 8) & 0x00FF);
        f->data[6] =   s->VEH_SPEED_ACTUAL;
        f->data[7] =   s->VEH_SPEED_DEMAND;
)

NRAI_CAN_MKFRAME(AI2VCU_Steer,
        uint32_t tmp;
        f->len = 2;
        tmp = s->STEER_REQUEST;
        tmp = CLAMP(tmp,-210,210);

        f->data[0] =            (tmp       & 0x00FF);
        f->data[1] = (((uint16_t)tmp >> 8) & 0x00FF);
)

NRAI_CAN_MKFRAME(AI2VCU_Brake,
        uint32_t tmp;
        f->len = 2;

        tmp = s->HYD_PRESS_F_REQ_pct;
        tmp = CLAMP(tmp,0,200);
        f->data[0] = tmp;

        tmp = s->HYD_PRESS_R_REQ_pct;
        tmp = CLAMP(tmp,0,200);
        f->data[1] = tmp;
)

NRAI_CAN_MKFRAME(AI2VCU_Drive_F,
        uint32_t tmp;
        f->len = 4;

        tmp = s->FRONT_AXLE_TRQ_REQUEST;
        tmp = CLAMP(tmp,0,1950);
        f->data[0] =  (tmp       & 0xFF);
        f->data[1] = ((tmp >> 8) & 0xFF);

        tmp = s->FRONT_MOTOR_SPEED_MAX;
        tmp = CLAMP(tmp,0,4000);
        f->data[2] = (tmp        & 0xFF);
        f->data[3] = ((tmp >> 8) & 0xFF);
)

NRAI_CAN_MKFRAME(AI2VCU_Drive_R,
        uint32_t tmp;
        f->len = 4;

        tmp = s->REAR_AXLE_TRQ_REQUEST;
        tmp = CLAMP(tmp,0,1950);
        f->data[0] =  (tmp       & 0xFF);
        f->data[1] = ((tmp >> 8) & 0xFF);

        tmp = s->REAR_MOTOR_SPEED_MAX;
        tmp = CLAMP(tmp,0,4000);
        f->data[2] = (tmp        & 0xFF);
        f->data[3] = ((tmp >> 8) & 0xFF);
)
