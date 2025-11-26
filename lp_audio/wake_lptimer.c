#include "soc.h"
#include "lptimer.h"

void LPTIMER1_IRQHandler()
{
    LPTIMER_Type *lptimer = (LPTIMER_Type *) LPTIMER_BASE;
    lptimer_disable_counter(lptimer, 1);
    lptimer_clear_interrupt(lptimer, 1);
    lptimer_clear_interrupt(lptimer, 1);
    NVIC_DisableIRQ(61);
}

void lptimer_init()
{
    NVIC_DisableIRQ(61);
    uint32_t count = 327680 - 1;    // 10 seconds

    LPTIMER_Type *lptimer = (LPTIMER_Type *) LPTIMER_BASE;
    lptimer_disable_counter(lptimer, 1);
    lptimer_set_mode_userdefined(lptimer, 1);
    lptimer_load_count(lptimer, 1, &count);
    lptimer_enable_counter(lptimer, 1);
    lptimer_clear_interrupt(lptimer, 1);
    lptimer_clear_interrupt(lptimer, 1);

    NVIC_ClearPendingIRQ(61);
    NVIC_EnableIRQ(61);
}
