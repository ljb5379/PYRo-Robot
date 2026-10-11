#include "pyro_sentry_gimbal.h"
#include "gimbal_config.h"
namespace pyro
{

void sentry_gimbal_t::state_passive_t::enter(owner *owner)
{


    owner->_ctx.motor.motor_yaw->disable();
    owner->_ctx.motor.motor_pitch->disable();

    owner->_ctx.pid.pitch_pos_pid->clear();
    owner->_ctx.pid.pitch_spd_pid->clear();
    owner->_ctx.pid.yaw_pos_pid->clear();
    owner->_ctx.pid.yaw_spd_pid->clear();
}

void sentry_gimbal_t::state_passive_t::execute(owner *owner)
{
    //imu目标值设置
    owner->_ctx.data.imu_data.target_yaw_rad = owner->_ctx.data.imu_data.current_yaw_rad;
    owner->_ctx.data.imu_data.target_pitch_rad = owner->_ctx.data.imu_data.current_pitch_rad;
    //电机目标值设置
    owner->_ctx.data.target_data.pitch_pos = owner->_ctx.data.current_data.pitch_pos;
    owner->_ctx.data.target_data.yaw_pos = owner->_ctx.data.current_data.yaw_pos;


    owner->_ctx.motor.motor_yaw->send_torque(0);
    owner->_ctx.motor.motor_pitch->send_torque(0);


}

void sentry_gimbal_t::state_passive_t::exit(owner *owner)
{
}

} // namespace pyro