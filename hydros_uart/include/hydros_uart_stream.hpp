#ifndef HYDROS_UART_STREAM_H_
#define HYDROS_UART_STREAM_H_

#include "cmsis_os.h"

#include "hydrv_uart.hpp"

namespace hydros
{
    using namespace hydrv::UART;

    template <int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY>
    class UARTStream : public UART<RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY>
    {
    private:
        using UARTBase = UART<RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY>;

    public:
        UARTStream(const hydrv::UART::UARTLow::UARTPreset &UART_preset,
                   hydrv::GPIO::GPIOLow &rx_pin,
                   hydrv::GPIO::GPIOLow &tx_pin,
                   uint32_t IRQ_priority,
                   UARTBase::ExternalIRQCallback rx_callback = nullptr);

    public:
        hydrolib_ReturnCode Push(const void *data, uint32_t data_length);

        hydrolib_ReturnCode Open();
        hydrolib_ReturnCode Close();

    private:
        osMutexId_t stream_mutex_;
    };

    template <int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY>
    UARTStream<RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY>::UARTStream(
        const UARTLow::UARTPreset &UART_preset,
        hydrv::GPIO::GPIOLow &rx_pin,
        hydrv::GPIO::GPIOLow &tx_pin,
        uint32_t IRQ_priority,
        UARTBase::ExternalIRQCallback rx_callback)
        : UARTBase(
              UART_preset,
              rx_pin, tx_pin,
              IRQ_priority,
              rx_callback)
    {
        stream_mutex_ = osMutexNew(nullptr);
    }

    template <int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY>
    hydrolib_ReturnCode UARTStream<RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY>::Push(
        const void *data, uint32_t data_length)
    {
        return UARTBase::Transmit(data, data_length);
    }

    template <int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY>
    hydrolib_ReturnCode UARTStream<RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY>::Open()
    {
        osMutexAcquire(stream_mutex_, 0);
        return HYDROLIB_RETURN_OK;
    }

    template <int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY>
    hydrolib_ReturnCode UARTStream<RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY>::Close()
    {
        osMutexRelease(stream_mutex_);
        return HYDROLIB_RETURN_OK;
    }
}

#endif
