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
constinit hydros::UART::UART<255, 255>
    uart(hydrv::UART::UARTLow::USART3_115200_LOW, rx_pin, tx_pin, 7);

uint8_t buffer[BUFFER_LENGTH];

int main(void)
{
    hydrv::clock::Clock::Init(hydrv::clock::Clock::HSI_DEFAULT);
    NVIC_SetPriorityGrouping(0);
    led_pin.Init();
    uart.Init();

    while (1)
    {
        unsigned rx_length = uart.GetRxLength();
        if (rx_length >= 5)
        {
            uart.Read(buffer, BUFFER_LENGTH);
            uart.Transmit(buffer, BUFFER_LENGTH);
        }
    }
}

extern "C"
{
    void SysTick_Handler(void) { hydrv::clock::Clock::SysTickHandler(); }
    void USART3_IRQHandler(void) { uart.IRQCallback(); }
    void HardFault_Handler(void)
    {
        while (1)
        {
        }
    }
}
