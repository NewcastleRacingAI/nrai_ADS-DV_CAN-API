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
        return 0;
}

NRAI_FT int32_t
nrai_can_unpack_VCU2AI_Steer(struct nrai_can_ai_read * s, struct can_frame * f)
{
        return 0;
}

NRAI_FT int32_t
nrai_can_unpack_VCU2AI_Speeds(struct nrai_can_ai_read * s, struct can_frame * f)
{
        return 0;
}

NRAI_FT int32_t
nrai_can_unpack_VCU2AI_Brake(struct nrai_can_ai_read * s, struct can_frame * f)
{
        return 0;
}

NRAI_FT int32_t
nrai_can_unpack_VCU2AI_Drive_F(struct nrai_can_ai_read * s, struct can_frame * f)
{
        return 0;
}

NRAI_FT int32_t
nrai_can_unpack_VCU2AI_Drive_R(struct nrai_can_ai_read * s, struct can_frame * f)
{
        return 0;
}

NRAI_FT int32_t
nrai_can_unpack_VCU2AI_Wheel_counts(struct nrai_can_ai_read * s, struct can_frame * f)
{
        return 0;
}


NRAI_FT int32_t
nrai_can_unpack_BMC_Acceleration(struct nrai_can_ai_read * s, struct can_frame * f)
{
        return 0;
}

NRAI_FT int32_t
nrai_can_unpack_BMC_MagneticField(struct nrai_can_ai_read * s, struct can_frame * f)
{
        return 0;
}

NRAI_FT int32_t
nrai_can_unpack_L3GD20_Rotation_A(struct nrai_can_ai_read * s, struct can_frame * f)
{
        return 0;
}

NRAI_FT int32_t
nrai_can_unpack_L3GD20_Rotation_B(struct nrai_can_ai_read * s, struct can_frame * f)
{
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
nrai_can_mkframe_AI2VCU_Brake(struct nrai_can_ai_write * const s, struct can_frame * f)
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
