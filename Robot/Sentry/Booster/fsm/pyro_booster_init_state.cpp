#include "pyro_sentry_booster.h"
#include "booster_config.h"

namespace pyro
{

void sentry_booster_t::fsm_active_t::state_init_t::enter(owner *owner)
{
    owner->_ctx.data.init_finished_flag = 0 ;
    owner->_ctx.data.trigger_init_count = 0;
}

void sentry_booster_t::fsm_active_t::state_init_t::execute(owner *owner)
{
    float now_trigger_speed = owner->_ctx.data.current_data.trigger_spd;

    
    float trigger_init_speed = BOOSTER_TRIGGER_INIT_SPEED*BOOSTER_TRIGGER_RATE*BOOSTER_ONE_SHOOT_ANGLE;//以2颗/s 装弹
    owner->_ctx.data.out_data.trigger_torque = owner->_ctx.pid.trig_pos_pid
                        ->calculate(trigger_init_speed,
                                    now_trigger_speed);   //接在sendtouque前

        if(now_trigger_speed <= BOOSTER_TRIGGER_PLUG_SPEED*BOOSTER_TRIGGER_RATE*BOOSTER_ONE_SHOOT_ANGLE){
            owner->_ctx.data.trigger_init_count++;
            if(owner->_ctx.data.trigger_init_count >= BOOSTER_TRIGGER_INIT_COUNT){

                owner->_ctx.data.init_trigger_pos = owner->_ctx.data.current_data.trigger_equal_pos - 0.05f; //单发校准点位
                owner->_ctx.data.target_data.trigger_equal_pos =owner-> _ctx.data.init_trigger_pos;
                
                owner->_ctx.data.init_finished_flag = 1 ; //初始化事件完成标志 
            }
        }
        else{
            owner->_ctx.data.trigger_init_count = 0;
        }

    
        owner->_send_trig_command();    


}

void sentry_booster_t::fsm_active_t::state_init_t::exit(owner *owner)
{
    
}

} // namespace pyro


