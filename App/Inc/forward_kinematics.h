/**
 * @file forward_kinematics.h
 * @brief Public declarations for App layer.
 */

#ifndef __FORWARD_KINEMATICS_H
#define __FORWARD_KINEMATICS_H

#include "struct_typedef.h"
#include "arm.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C"{
#endif

#ifdef __cplusplus

#define FK_MAX_DOF ARM_DOF
#define FK_AS5600L_RAW_MAX 4096
#define FK_AS5600L_RAW_MASK 0x0FFFU

enum FK_Status_e
{
    FK_OK = 0,
    FK_ERR_AXIS_RANGE,
    FK_ERR_NOT_READY,
    FK_ERR_PARAM,
};

typedef struct
{
    fp32 zero_raw;
    fp32 scale;
    int8_t direction;
    fp32 q_min;
    fp32 q_max;
    uint8_t dof_index;
}FK_AxisConfig_t;

typedef struct
{
    uint16_t raw_last;
    int32_t turn_count;
    fp32 q_value;
    uint8_t initialized;
}FK_AxisState_t;

class forward_kinematics_t
{
    private:
        FK_AxisConfig_t axis_cfg[FK_MAX_DOF];
        FK_AxisState_t axis_state[FK_MAX_DOF];
        uint8_t axis_num;
        ArmKinematics6D *solver;

        fp32 ConvertRawToQ(uint8_t axis_id, uint16_t raw12);

    public:
        forward_kinematics_t();

        void Reset();
        FK_Status_e SetAxisNum(uint8_t input_axis_num);
        FK_Status_e ConfigureAxis(uint8_t axis_id, const FK_AxisConfig_t &cfg);
        FK_Status_e BindSolver(ArmKinematics6D *input_solver);
        FK_Status_e SetDH(const ArmDHParam_t dh[ARM_DOF]);

        FK_Status_e UpdateAxisRaw(uint8_t axis_id, uint16_t raw12);
        FK_Status_e GetJointQ(fp32 q_out[FK_MAX_DOF]) const;
        FK_Status_e SolvePose(ArmPose_t *pose_out) const;
};

#endif

#ifdef __cplusplus
}
#endif

#endif

