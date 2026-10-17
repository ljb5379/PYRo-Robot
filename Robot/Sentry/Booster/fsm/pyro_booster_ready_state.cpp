#include "pyro_sentry_booster.h"
#include "booster_config.h"
#include "pyro_algo_common.h"
#include "pyro_dwt_drv.h"


float llspd = 0.0f;
float lrspd =  0.0f;
namespace pyro
{

void sentry_booster_t::fsm_active_t::fsm_ready_t::on_enter(owner *owner)
{
    change_state(&_singgle_shoot);
}

void sentry_booster_t::fsm_active_t::fsm_ready_t::on_execute(owner *owner)
{
        if(owner->_ctx.cmd->multi_shoot){
            change_state(&_multi_shoot);
        }
        else{
            change_state(&_singgle_shoot);
        }
        // owner->_trigger_control();
        // owner->_send_trig_command();
        owner->_fric_control();
        owner->_send_fric_command();
}

void sentry_booster_t::fsm_active_t::fsm_ready_t::on_exit(owner *owner)
{
    
    owner->_ctx.data.target_data.trigger_spd = 0.0f;
    owner->_ctx.data.target_data.trigger_equal_pos = 
                (owner->_ctx.data.target_data.trigger_equal_pos/BOOSTER_ONE_SHOOT_ANGLE)
            *   BOOSTER_ONE_SHOOT_ANGLE +owner->_ctx.data.init_trigger_pos;
                
    owner->_ctx.data.current_fire_count = owner->_ctx.cmd->fire_count;
}

} // namespace pyro
