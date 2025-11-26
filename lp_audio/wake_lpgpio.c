#include "soc.h"
#include "gpio.h"
#include "pinconf.h"

#define TRIGGER_PIN         (PIN_0)
#define TRIGGER_PIN_MASK    (1U << TRIGGER_PIN)

/* Function for LPGPIO combined interrupt */
void LPGPIO_COMB_IRQHandler()
{
    gpio_disable_interrupt(LPGPIO, TRIGGER_PIN);
    gpio_interrupt_eoi(LPGPIO, TRIGGER_PIN);
    gpio_interrupt_eoi(LPGPIO, TRIGGER_PIN);        // LPGPIO interrupt clear

    NVIC_DisableIRQ(LPGPIO_COMB_IRQ_IRQn);
}

/* Function for initializing the LPGPIO peripheral */
void lpgpio_init()
{
    /* Disable the LPGPIO combined interrupt */
    NVIC_DisableIRQ(LPGPIO_COMB_IRQ_IRQn);

    uint32_t padconf = PADCTRL_READ_ENABLE | PADCTRL_DRIVER_DISABLED_PULL_UP;
    pinconf_set(PORT_15, TRIGGER_PIN, 0, padconf);

    gpio_set_value_low(LPGPIO, TRIGGER_PIN);
    gpio_set_direction_input(LPGPIO, TRIGGER_PIN);
    gpio_set_software_mode(LPGPIO, TRIGGER_PIN);

    gpio_enable_interrupt(LPGPIO, TRIGGER_PIN);
    gpio_interrupt_set_edge_trigger(LPGPIO, TRIGGER_PIN);
    gpio_interrupt_set_polarity_low(LPGPIO, TRIGGER_PIN);
    gpio_interrupt_eoi(LPGPIO, TRIGGER_PIN);
    gpio_interrupt_eoi(LPGPIO, TRIGGER_PIN);        // LPGPIO interrupt clear

    /* Enable the LPGPIO combined interrupt */
    NVIC_ClearPendingIRQ(LPGPIO_COMB_IRQ_IRQn);
    NVIC_EnableIRQ(LPGPIO_COMB_IRQ_IRQn);
}
