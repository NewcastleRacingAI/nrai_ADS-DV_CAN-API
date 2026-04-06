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

#define CLAMP(_value,_min,_max)                                                \
        do {                                                                   \
                if (_value < _min){                                            \
                        _value = _min;                                         \
                } else if (_value > _max){                                     \
                        _value =_max;                                          \
                }                                                              \
        } while(0)

NRAI_FT int32_t
nrai_can_unpack_VCU2AI_Status(struct nrai_can_ai_read * s, struct can_frame * f)
{
        if ((s == NULL) || (f == NULL))
                return -1;
        if (f->can_id != NRAI_CAN_ID_VCU2AI_Status)
                return -1;

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
        return 0;
}

NRAI_FT int32_t
nrai_can_unpack_VCU2AI_Steer(struct nrai_can_ai_read * s, struct can_frame * f)
{
        if ((s == NULL) || (f == NULL))
                return -1;
        if (f->can_id != NRAI_CAN_ID_VCU2AI_Steer)
                return -1;

        s->ANGLE                = ((uint16_t)f->data[1] << 8) | f->data[0];
        s->ANGLE_MAX            = ((uint16_t)f->data[3] << 8) | f->data[2];
        s->ANGLE_REQUEST        = ((uint16_t)f->data[5] << 8) | f->data[4];
        return 0;
}

NRAI_FT int32_t
nrai_can_unpack_VCU2AI_Speeds(struct nrai_can_ai_read * s, struct can_frame * f)
{
        if ((s == NULL) || (f == NULL))
                return -1;
        if (f->can_id != NRAI_CAN_ID_VCU2AI_Speeds)
                return -1;

        s->FL_WHEEL_SPEED = ((uint16_t)f->data[1] << 8) | f->data[0];
        s->FR_WHEEL_SPEED = ((uint16_t)f->data[3] << 8) | f->data[2];
        s->RL_WHEEL_SPEED = ((uint16_t)f->data[5] << 8) | f->data[4];
        s->RR_WHEEL_SPEED = ((uint16_t)f->data[7] << 8) | f->data[6];
        return 0;
}

NRAI_FT int32_t
nrai_can_unpack_VCU2AI_Brake(struct nrai_can_ai_read * s, struct can_frame * f)
{
        if ((s == NULL) || (f == NULL))
                return -1;
        if (f->can_id != NRAI_CAN_ID_VCU2AI_Brake)
                return -1;

        s->HYD_PRESS_F_pct         = f->data[0];
        s->HYD_PRESS_F_REQ_pct     = f->data[1];
        s->HYD_PRESS_R_pct         = f->data[2];
        s->HYD_PRESS_R_REQ_pct     = f->data[3];
        s->STATUS_BRK              = (f->data[4] & 0b00001111);
        s->STATUS_EBS              = (f->data[4] & 0b11110000) >> 4;
        return 0;
}

NRAI_FT int32_t
nrai_can_unpack_VCU2AI_Drive_F(struct nrai_can_ai_read * s, struct can_frame * f)
{
        if ((s == NULL) || (f == NULL))
                return -1;
        if (f->can_id != NRAI_CAN_ID_VCU2AI_Drive_F)
                return -1;

        s->FRONT_AXLE_TRQ               = ((uint16_t)f->data[1] << 8) | f->data[0];
        s->FRONT_AXLE_TRQ_REQUEST       = ((uint16_t)f->data[3] << 8) | f->data[2];
        s->FRONT_AXLE_TRQ_MAX           = ((uint16_t)f->data[5] << 8) | f->data[4];
        return 0;
}

NRAI_FT int32_t
nrai_can_unpack_VCU2AI_Drive_R(struct nrai_can_ai_read * s, struct can_frame * f)
{
        if ((s == NULL) || (f == NULL))
                return -1;
        if (f->can_id != NRAI_CAN_ID_VCU2AI_Drive_R)
                return -1;

        s->REAR_AXLE_TRQ_Nm             = ((uint16_t)f->data[1] << 8) | f->data[0];
        s->REAR_AXLE_TRQ_REQUEST_Nm     = ((uint16_t)f->data[3] << 8) | f->data[2];
        s->REAR_AXLE_TRQ_MAX_Nm         = ((uint16_t)f->data[5] << 8) | f->data[4];
        return 0;
}

NRAI_FT int32_t
nrai_can_unpack_VCU2AI_Wheel_counts(struct nrai_can_ai_read * s, struct can_frame * f)
{
        if ((s == NULL) || (f == NULL))
                return -1;
        if (f->can_id != NRAI_CAN_ID_VCU2AI_Wheel_counts)
                return -1;

        s->FL_PULSE_COUNT = ((uint16_t)f->data[1] << 8) | f->data[0];
        s->FR_PULSE_COUNT = ((uint16_t)f->data[3] << 8) | f->data[2];
        s->RL_PULSE_COUNT = ((uint16_t)f->data[5] << 8) | f->data[4];
        s->RR_PULSE_COUNT = ((uint16_t)f->data[7] << 8) | f->data[6];
        return 0;
}


NRAI_FT int32_t
nrai_can_unpack_BMC_Acceleration(struct nrai_can_ai_read * s, struct can_frame * f)
{
        if ((s == NULL) || (f == NULL))
                return -1;
        if (f->can_id != NRAI_CAN_ID_BMC_Acceleration)
                return -1;

        s->Acceleration_X       = ((uint16_t)f->data[1] << 8) | f->data[0];
        s->Acceleration_Y       = ((uint16_t)f->data[3] << 8) | f->data[2];
        s->Acceleration_Z       = ((uint16_t)f->data[5] << 8) | f->data[4];
        s->Temperature          = f->data[6];
        s->VerticalAxis         = (f->data[7] & 0b00000011);
        s->Orientation          = (f->data[7] & 0b00011100) >> 2;
        return 0;
}

NRAI_FT int32_t
nrai_can_unpack_BMC_MagneticField(struct nrai_can_ai_read * s, struct can_frame * f)
{
        if ((s == NULL) || (f == NULL))
                return -1;
        if (f->can_id != NRAI_CAN_ID_BMC_MagneticField)
                return -1;

        s->MagneticField_X = ((uint16_t)f->data[1] << 8) | f->data[0];
        s->MagneticField_Y = ((uint16_t)f->data[3] << 8) | f->data[2];
        s->MagneticField_Z = ((uint16_t)f->data[5] << 8) | f->data[4];
        return 0;
}

NRAI_FT int32_t
nrai_can_unpack_L3GD20_Rotation_A(struct nrai_can_ai_read * s, struct can_frame * f)
{
        if ((s == NULL) || (f == NULL))
                return -1;
        if (f->can_id != NRAI_CAN_ID_L3GD20_Rotation_A)
                return -1;

        s->Rotation_X = ((uint32_t)f->data[3] << 24) | 
                                       ((uint32_t)f->data[2] << 16) |
                                       ((uint16_t)f->data[1] << 8) | f->data[0];
        s->Rotation_Y = ((uint32_t)f->data[7] << 24) | 
                                       ((uint32_t)f->data[6] << 16) |
                                       ((uint16_t)f->data[5] << 8) | f->data[4];
        return 0;
}

NRAI_FT int32_t
nrai_can_unpack_L3GD20_Rotation_B(struct nrai_can_ai_read * s, struct can_frame * f)
{
        if ((s == NULL) || (f == NULL))
                return -1;
        if (f->can_id != NRAI_CAN_ID_L3GD20_Rotation_A)
                return -1;

        s->Rotation_Z = ((uint32_t)f->data[3] << 24) | 
                                       ((uint32_t)f->data[2] << 16) |
                                       ((uint16_t)f->data[1] << 8) | f->data[0];
        return 0;
}

NRAI_FT int32_t
nrai_can_unpack_GPS_Status(struct nrai_can_ai_read * s, struct can_frame * f)
{
        return 0;
}

NRAI_FT int32_t
nrai_can_unpack_GPS_CourseSpeed(struct nrai_can_ai_read * s, struct can_frame * f)
{
        return 0;
}

NRAI_FT int32_t
nrai_can_unpack_GPS_PositionLongitude(struct nrai_can_ai_read * s, struct can_frame * f)
{
        return 0;
}

NRAI_FT int32_t
nrai_can_unpack_GPS_PositionLatitude(struct nrai_can_ai_read * s, struct can_frame * f)
{
        return 0;
}

NRAI_FT int32_t
nrai_can_unpack_GPS_PositionAltitude(struct nrai_can_ai_read * s, struct can_frame * f)
{
        return 0;
}

NRAI_FT int32_t
nrai_can_unpack_GPS_Delusions_A(struct nrai_can_ai_read * s, struct can_frame * f)
{
        return 0;
}

NRAI_FT int32_t
nrai_can_unpack_GPS_Delusions_B(struct nrai_can_ai_read * s, struct can_frame * f)
{
        return 0;
}

NRAI_FT int32_t
nrai_can_unpack_GPS_DateTime(struct nrai_can_ai_read * s, struct can_frame * f)
{
        return 0;
}

NRAI_FT int32_t
nrai_can_unpack_IO(struct nrai_can_ai_read * s, struct can_frame * f)
{
        return 0;
}

NRAI_FT int32_t
nrai_can_unpack_RTC_DateTime(struct nrai_can_ai_read * s, struct can_frame * f)
{
        return 0;
}

NRAI_FT int32_t
nrai_can_unpack_Out_IO(struct nrai_can_ai_read * s, struct can_frame * f)
{
        return 0;
}

NRAI_FT int32_t
nrai_can_unpack_Out_PowerOff(struct nrai_can_ai_read * s, struct can_frame * f)
{
        return 0;
}

NRAI_FT int32_t
nrai_can_unpack_Out_Gyro(struct nrai_can_ai_read * s, struct can_frame * f)
{
        return 0;
}

NRAI_FT int32_t
nrai_can_unpack_Out_BMC_AccScale(struct nrai_can_ai_read * s, struct can_frame * f)
{
        return 0;
}

NRAI_FT int32_t
nrai_can_unpack_Out_SaveConfig(struct nrai_can_ai_read * s, struct can_frame * f)
{
        return 0;
}

NRAI_FT int32_t
nrai_can_unpack_Out_RTC_SetTime(struct nrai_can_ai_read * s, struct can_frame * f)
{
        return 0;
}

NRAI_FT int32_t
nrai_can_unpack_Out_RTC_TimeFromGPS(struct nrai_can_ai_read * s, struct can_frame * f)
{
        return 0;
}

NRAI_FT int32_t
nrai_can_unpack_Out_Acc_FastCalibration(struct nrai_can_ai_read * s, struct can_frame * f)
{
        return 0;
}




NRAI_FT int32_t
nrai_can_mkframe_AI2VCU_Status(struct nrai_can_ai_write * s, struct can_frame * f)
{
        if ((s == NULL) || (f == NULL))
                return -1;

        f->can_id = NRAI_CAN_ID_AI2VCU_Status;
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
        return 0;
}

NRAI_FT int32_t
nrai_can_mkframe_AI2VCU_Steer(  struct nrai_can_ai_write * s, struct can_frame * f)
{
        int32_t tmp;
        if ((s == NULL) || (f == NULL))
                return -1;

        f->can_id = NRAI_CAN_ID_AI2VCU_Steer;
        f->len = 2;

        tmp = s->STEER_REQUEST;
        CLAMP(tmp,-210,210);

        f->data[0] =            (tmp       & 0x00FF);
        f->data[1] = (((uint16_t)tmp >> 8) & 0x00FF);
        return 0;
}

NRAI_FT int32_t
nrai_can_mkframe_AI2VCU_Brake(struct nrai_can_ai_write * s, struct can_frame * f)
{
        uint32_t tmp;
        if ((s == NULL) || (f == NULL))
                return -1;

        f->can_id = NRAI_CAN_ID_AI2VCU_Brake;
        f->len = 2;

        tmp = s->HYD_PRESS_F_REQ_pct;
        CLAMP(tmp,0,100);
        f->data[0] = tmp;

        tmp = s->HYD_PRESS_R_REQ_pct;
        CLAMP(tmp,0,100);
        f->data[1] = tmp;
        return 0;
}

NRAI_FT int32_t
nrai_can_mkframe_AI2VCU_Drive_F(struct nrai_can_ai_write * s, struct can_frame * f)
{
        uint32_t tmp;
        if ((s == NULL) || (f == NULL))
                return -1;

        f->can_id = NRAI_CAN_ID_AI2VCU_Drive_F;
        f->len = 4;

        tmp = s->FRONT_AXLE_TRQ_REQUEST;
        CLAMP(tmp,0,1950);
        f->data[0] =  (tmp       & 0xFF);
        f->data[1] = ((tmp >> 8) & 0xFF);

        tmp = s->FRONT_MOTOR_SPEED_MAX;
        CLAMP(tmp,0,4000);
        f->data[2] = (tmp        & 0xFF);
        f->data[3] = ((tmp >> 8) & 0xFF);
        return 0;
}

NRAI_FT int32_t
nrai_can_mkframe_AI2VCU_Drive_R(struct nrai_can_ai_write * s, struct can_frame * f)
{
        uint32_t tmp;
        if ((s == NULL) || (f == NULL))
                return -1;

        f->can_id = NRAI_CAN_ID_AI2VCU_Drive_R;
        f->len = 4;

        tmp = s->REAR_AXLE_TRQ_REQUEST;
        CLAMP(tmp,0,1950);
        f->data[0] =  (tmp       & 0xFF);
        f->data[1] = ((tmp >> 8) & 0xFF);

        tmp = s->REAR_MOTOR_SPEED_MAX;
        CLAMP(tmp,0,4000);
        f->data[2] = (tmp        & 0xFF);
        f->data[3] = ((tmp >> 8) & 0xFF);
        return 0;
}
