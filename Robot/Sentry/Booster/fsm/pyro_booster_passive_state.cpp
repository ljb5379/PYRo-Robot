#include "pyro_sentry_booster.h"
#include "booster_config.h"
namespace pyro
{

void sentry_booster_t::state_passive_t::enter(owner *owner)
{
    #if new_function2_enable ==0
    owner->_ctx.motor.fric_left->disable();
    owner->_ctx.motor.fric_right->disable();
    
    #elif new_function2_enable == 1 

    #endif


    owner->_ctx.motor.trigger->disable();

}

void sentry_booster_t::state_passive_t::execute(owner *owner)
{
    #if new_function2_enable ==0
    owner->_ctx.motor.fric_left->send_torque(0);
    owner->_ctx.motor.fric_right->send_torque(0);
    #elif new_function2_enable == 1 
    owner->_ctx.data.target_data.left_spd = 0;
    owner->_ctx.data.target_data.right_spd = 0;
    owner->_fric_control();
    owner->_send_fric_command();
    #endif

    owner->_ctx.motor.trigger->send_torque(0);

    owner->_ctx.data.current_fire_count = owner->_ctx.cmd->fire_count;
    owner->_ctx.data.target_data.trigger_spd = 0;
    owner->_ctx.data.target_data.trigger_equal_pos = owner->_ctx.data.current_data.trigger_equal_pos;
}

void sentry_booster_t::state_passive_t::exit(owner *owner)
{
}

} // namespace pyro