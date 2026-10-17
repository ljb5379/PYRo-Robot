#include "pyro_sentry_booster.h"
#include "booster_config.h"

namespace pyro
{

void sentry_booster_t::fsm_active_t::state_stall_t::enter(owner *owner)
{
    owner->_ctx.data.trigger_stall_time++;
    owner->_ctx.data.init_finished_flag=0;
}

void sentry_booster_t::fsm_active_t::state_stall_t::execute(owner *owner)
{
    


}

void sentry_booster_t::fsm_active_t::state_stall_t::exit(owner *owner)
{
}

} // namespace pyro