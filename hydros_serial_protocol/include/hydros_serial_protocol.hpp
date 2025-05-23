#ifndef HYDROS_SERIAL_PROTOCOL_H_
#define HYDROS_SERIAL_PROTOCOL_H_

#include <cstdint>

#include "cmsis_os.h"

#include "hydrolib_serial_protocol_slave.hpp"
#include "hydrv_uart.hpp"

namespace hydros::serial_protocol
{
using namespace hydrv::UART;
using namespace hydrolib::serial_protocol;

template <PublicMemoryConcept Memory, logger::LogDistributorConcept Distributor>
void MainTask(void *argument);

template <PublicMemoryConcept Memory, logger::LogDistributorConcept Distributor>
class UARTSlave
{
private:
    class RxCallback_
    {
    public:
        constexpr RxCallback_(osSemaphoreId_t &rx_completed)
            : rx_completed_(rx_completed)
        {
        }

    public:
        void operator()() { osSemaphoreRelease(rx_completed_); }

    private:
        osSemaphoreId_t &rx_completed_;
    };

private:
    using UARTType = UART<1024, 1024, class RxCallback_>;
    using SlaveType = Slave<UARTType, Memory, Distributor>;

public:
    constexpr UARTSlave(const char *name, osPriority_t main_thread_priority,
                        const UARTLow::UARTPreset &UART_preset,
                        hydrv::GPIO::GPIOLow &rx_pin,
                        hydrv::GPIO::GPIOLow &tx_pin, uint32_t IRQ_priority,
                        uint8_t address, Memory &public_memory,
                        logger::Logger<Distributor> &logger,
                        uint8_t network = 0xA0);

public:
    void IRQCallback();

    void MainThreadHandler();

private:
    const char *name_;

    osSemaphoreId_t rx_completed_;
    osThreadId_t main_thread_handler_;

    class RxCallback_ rx_callback_;

    UARTType uart_;
    SlaveType slave_;
};

template <PublicMemoryConcept Memory, logger::LogDistributorConcept Distributor>
void MainTask(void *argument)
{
    UARTSlave<Memory, Distributor> *handler =
        static_cast<UARTSlave<Memory, Distributor> *>(argument);
    handler->MainThreadHandler();
}

template <PublicMemoryConcept Memory, logger::LogDistributorConcept Distributor>
constexpr UARTSlave<Memory, Distributor>::UARTSlave(
    const char *name, osPriority_t main_thread_priority,
    const UARTLow::UARTPreset &UART_preset, hydrv::GPIO::GPIOLow &rx_pin,
    hydrv::GPIO::GPIOLow &tx_pin, uint32_t IRQ_priority, uint8_t address,
    Memory &public_memory, logger::Logger<Distributor> &logger, uint8_t network)
    : name_(name),
      rx_completed_(osSemaphoreNew(1, 0, nullptr)),
      main_thread_handler_(nullptr),
      rx_callback_(rx_completed_),
      uart_(UART_preset, rx_pin, tx_pin, IRQ_priority, rx_callback_),
      slave_(address, uart_, public_memory, logger, network)
{
    osThreadAttr_t main_thread_attributes = {
        .name = "SerialProtocolMainThread",
        .stack_size = 128 * 16,
        .priority = main_thread_priority,
    };
    main_thread_handler_ = osThreadNew(MainTask<Memory, Distributor>, this,
                                       &main_thread_attributes);
}

template <PublicMemoryConcept Memory, logger::LogDistributorConcept Distributor>
void UARTSlave<Memory, Distributor>::IRQCallback()
{
    uart_.IRQcallback();
}

template <PublicMemoryConcept Memory, logger::LogDistributorConcept Distributor>
void UARTSlave<Memory, Distributor>::MainThreadHandler()
{
    while (1)
    {
        osStatus_t ret = osSemaphoreAcquire(rx_completed_, osWaitForever);
        if (!ret)
        {
            slave_.ProcessRx();
        }
    }
}

} // namespace hydros::serial_protocol

#endif
