#include "pyro_sentry_booster.h"
#include "booster_config.h"

namespace pyro
{

void sentry_booster_t::fsm_active_t::fsm_ready_t::multi_shoot_t::enter(owner *owner)
{

}

void sentry_booster_t::fsm_active_t::fsm_ready_t::multi_shoot_t::execute(owner *owner)
{
    //防止摩擦轮没开就发射//可以改成摩擦轮速度达标标志
    if(owner->_ctx.cmd->fric_on == false ){
        return;
    }
    
    if(owner->_ctx.cmd->multi_shoot == false){
        request_switch(&owner->_state_active.state_ready._singgle_shoot);
        return;
    }



        owner->_trigger_control();
        owner->_send_trig_command();

}

void sentry_booster_t::fsm_active_t::fsm_ready_t::multi_shoot_t::exit(owner *owner)
{
    owner->_ctx.data.target_data.trigger_equal_pos = owner->_ctx.data.current_data.trigger_equal_pos;
    owner->_ctx.data.current_fire_count = owner->_ctx.cmd->fire_count;
}

} // namespace pyro