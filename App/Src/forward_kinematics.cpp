/**
 * @file forward_kinematics.cpp
 * @brief AS5600 raw-angle to joint-angle conversion and FK pose solving.
 */

#include "forward_kinematics.h"
#include <string.h>

forward_kinematics_t::forward_kinematics_t()
{
	solver = ArmGetSolver();
    Reset();
}

void forward_kinematics_t::Reset()
{
    axis_num = FK_MAX_DOF;
    memset(axis_cfg, 0, sizeof(axis_cfg));
    memset(axis_state, 0, sizeof(axis_state));

    for(uint8_t i = 0; i < FK_MAX_DOF; i++)
    {
        axis_cfg[i].direction = 1;
        axis_cfg[i].q_min = -1e6f;
        axis_cfg[i].q_max = 1e6f;
        axis_cfg[i].dof_index = i;
    }
}

FK_Status_e forward_kinematics_t::SetAxisNum(uint8_t input_axis_num)
{
    if(input_axis_num == 0 || input_axis_num > FK_MAX_DOF)
        return FK_ERR_PARAM;

    axis_num = input_axis_num;
    return FK_OK;
}

FK_Status_e forward_kinematics_t::ConfigureAxis(uint8_t axis_id, const FK_AxisConfig_t &cfg)
{
    if(axis_id >= axis_num)
        return FK_ERR_AXIS_RANGE;
    if(cfg.dof_index >= FK_MAX_DOF)
        return FK_ERR_PARAM;
    if(cfg.scale == 0.0f)
        return FK_ERR_PARAM;
    if(cfg.direction != 1 && cfg.direction != -1)
        return FK_ERR_PARAM;

    axis_cfg[axis_id] = cfg;
    axis_state[axis_id].initialized = 0;
    axis_state[axis_id].turn_count = 0;
    return FK_OK;
}

FK_Status_e forward_kinematics_t::BindSolver(ArmKinematics6D *input_solver)
{
    if(input_solver == NULL)
        return FK_ERR_NOT_READY;

    solver = input_solver;
    return FK_OK;
}

FK_Status_e forward_kinematics_t::SetDH(const ArmDHParam_t dh[ARM_DOF])
{
    if(dh == NULL)
        return FK_ERR_PARAM;
    if(solver == NULL)
        return FK_ERR_NOT_READY;

    solver->SetDH(dh);
    return FK_OK;
}

fp32 forward_kinematics_t::ConvertRawToQ(uint8_t axis_id, uint16_t raw12)
{
    FK_AxisState_t *state = &axis_state[axis_id];
    const FK_AxisConfig_t *cfg = &axis_cfg[axis_id];

    // Keep only 12-bit absolute angle and maintain multi-turn continuity.
    uint16_t raw = raw12 & FK_AS5600L_RAW_MASK;
    if(!state->initialized)
    {
        state->raw_last = raw;
        state->turn_count = 0;
        state->initialized = 1;
    }
    else
    {
        int32_t diff = (int32_t)raw - (int32_t)state->raw_last;
        if(diff > (FK_AS5600L_RAW_MAX / 2))
            state->turn_count--;
        else if(diff < -(FK_AS5600L_RAW_MAX / 2))
            state->turn_count++;

        state->raw_last = raw;
    }

    fp32 raw_unwrap = (fp32)(state->turn_count * FK_AS5600L_RAW_MAX + raw);
    fp32 q = (raw_unwrap - cfg->zero_raw) * cfg->scale * (fp32)cfg->direction;

    if(q > cfg->q_max)
        q = cfg->q_max;
    else if(q < cfg->q_min)
        q = cfg->q_min;

    state->q_value = q;
    return q;
}

FK_Status_e forward_kinematics_t::UpdateAxisRaw(uint8_t axis_id, uint16_t raw12)
{
    if(axis_id >= axis_num)
        return FK_ERR_AXIS_RANGE;

    ConvertRawToQ(axis_id, raw12);
    return FK_OK;
}

FK_Status_e forward_kinematics_t::GetJointQ(fp32 q_out[FK_MAX_DOF]) const
{
    if(q_out == NULL)
        return FK_ERR_PARAM;

    memset(q_out, 0, sizeof(fp32) * FK_MAX_DOF);
    for(uint8_t i = 0; i < axis_num; i++)
    {
        if(!axis_state[i].initialized)
            return FK_ERR_NOT_READY;

        uint8_t dof_id = axis_cfg[i].dof_index;
        q_out[dof_id] = axis_state[i].q_value;
    }

    return FK_OK;
}

FK_Status_e forward_kinematics_t::SolvePose(ArmPose_t *pose_out) const
{
    if(pose_out == NULL)
        return FK_ERR_PARAM;
    if(solver == NULL)
        return FK_ERR_NOT_READY;

    fp32 q[FK_MAX_DOF] = {0.0f};
    FK_Status_e q_ret = GetJointQ(q);
    if(q_ret != FK_OK)
        return q_ret;

    solver->ForwardKinematics(q, pose_out);

    return FK_OK;
}
