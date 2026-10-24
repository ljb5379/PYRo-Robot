#include "pyro_sentry_booster.h"
#include "booster_config.h"
#include "pyro_algo_common.h"

namespace pyro
{

void sentry_booster_t::fsm_active_t::fsm_ready_t::singgle_shoot_t::enter(owner *owner)
{
    //就近找到单发位置
float shots =   roundf((owner->_ctx.data.target_data.trigger_equal_pos 
                        - owner->_ctx.data.init_trigger_pos) / BOOSTER_ONE_SHOOT_ANGLE);
    owner->_ctx.data.target_data.trigger_equal_pos = 
                    loop_fp32_constrain(shots * BOOSTER_ONE_SHOOT_ANGLE 
                        + owner->_ctx.data.init_trigger_pos, -PI, PI);
}

void sentry_booster_t::fsm_active_t::fsm_ready_t::singgle_shoot_t::execute(owner *owner)
{   
    if(owner->_ctx.cmd->fric_on == false ){
        return;
    }
    
    if(owner->_ctx.cmd->multi_shoot){
        request_switch(&owner->_state_active.state_ready._multi_shoot);
        return;
    }
    

    // if(owner->_ctx.cmd->fire_count > owner->_ctx.data.current_fire_count){
        
    //     owner->_ctx.data.target_data.trigger_equal_pos += PI/4;
    //     owner->_ctx.data.current_fire_count++;
    //     //owner->_ctx.data.target_data.trigger_spd = 40.0f;
    // }
    // else{
    //     //owner->_ctx.data.target_data.trigger_spd = 0.0f;
    //     owner->_ctx.data.target_data.trigger_equal_pos = 
    //             (owner->_ctx.data.target_data.trigger_equal_pos/BOOSTER_ONE_SHOOT_ANGLE)
    //         *   BOOSTER_ONE_SHOOT_ANGLE +owner->_ctx.data.init_trigger_pos;
    // }
    
        owner->_trigger_control();
        owner->_send_trig_command();
        // if(fabsf(owner->_ctx.target_data.trigger_pos - owner->_ctx.current_data.trigger_pos)<0.03f)
        // owner->_ctx.cmd->singgle_shoot = false;
    
    //owner->_ctx.motor.trigger->send_torque(0);

}

void sentry_booster_t::fsm_active_t::fsm_ready_t::singgle_shoot_t::exit(owner *owner)
{
    //owner->_ctx.data.singgle_shoot = false;
    //owner->_ctx.target_data.trigger_spd = 0.0f;
    owner->_ctx.data.target_data.trigger_equal_pos = owner->_ctx.data.current_data.trigger_equal_pos;
    owner->_ctx.data.current_fire_count = owner->_ctx.cmd->fire_count;
}

} // namespace pyro