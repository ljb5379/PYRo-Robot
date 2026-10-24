#include "pyro_sentry_booster.h"
#include "booster_config.h"

namespace pyro
{

void sentry_booster_t::fsm_active_t::state_stall_t::enter(owner *owner)
{
    owner->_ctx.data.trigger_stall_time++;
    
}

void sentry_booster_t::fsm_active_t::state_stall_t::execute(owner *owner)
{
    
    request_switch(&owner->_state_active.state_init);

}

void sentry_booster_t::fsm_active_t::state_stall_t::exit(owner *owner)
{
}

} // namespace pyro