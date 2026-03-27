#include "hydrv_clock.hpp"
#include "hydrv_gpio_low.hpp"
#include "hydros_uart.hpp"

#define BUFFER_LENGTH 5 // TODO:make variable

constinit hydrv::GPIO::GPIOLow led_pin(hydrv::GPIO::GPIOLow::GPIOD_port, 15,
                                       hydrv::GPIO::GPIOLow::GPIO_Output);
constinit hydrv::GPIO::GPIOLow rx_pin(hydrv::GPIO::GPIOLow::GPIOB_port, 11,
                                      hydrv::GPIO::GPIOLow::GPIO_UART_RX);
constinit hydrv::GPIO::GPIOLow tx_pin(hydrv::GPIO::GPIOLow::GPIOB_port, 10,
                                      hydrv::GPIO::GPIOLow::GPIO_UART_TX);
constinit hydrv::GPIO::GPIOLow direction_pin(hydrv::GPIO::GPIOLow::GPIOC_port,
                                             13,
                                             hydrv::GPIO::GPIOLow::GPIO_Output);
constinit hydros::UART::RS485<255, 255>
    RS(hydrv::UART::UARTLow::USART3_115200_LOW, rx_pin, tx_pin, direction_pin,
       7);

uint8_t buffer[BUFFER_LENGTH];

int main(void)
{
    hydrv::clock::Clock::Init(hydrv::clock::Clock::HSI_DEFAULT);
    NVIC_SetPriorityGrouping(0);
    led_pin.Init();
    RS.Init();

    while (1)
    {
        unsigned rx_length = RS.GetRxLength();
        if (rx_length >= 5)
        {
            RS.Read(buffer, BUFFER_LENGTH);
            RS.Transmit(buffer, BUFFER_LENGTH);
        }
    }
}

extern "C"
{
    void SysTick_Handler(void) { hydrv::clock::Clock::SysTickHandler(); }
    void USART3_IRQHandler(void) { RS.IRQCallback(); }
}
