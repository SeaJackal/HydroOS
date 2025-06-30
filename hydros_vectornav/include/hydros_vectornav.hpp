#ifndef HYDROS_SERIAL_PROTOCOL_H_
#define HYDROS_SERIAL_PROTOCOL_H_

#include <cstdint>

#include "cmsis_os.h"

#include "hydrolib_common.h"
#include "hydrolib_vectronav.hpp"
#include "hydrv_uart.hpp"

extern "C"
{
#include "hydrv_clock.h"
}

namespace hydros::vectornav
{
template <typename Logger>
class VectorNAVModule
{
private:
    class RxCallback_
    {
    public:
        constexpr RxCallback_(osSemaphoreId_t &rx_completed)
            : rx_completed_(rx_completed), counter1(0), counter2(0), flag(false)
        {
        }

    public:
        void operator()()
        {
            if (!flag)
            {
                flag = true;
                // trace1[counter1] = hydrv_Clock_GetSystemTime();
                // counter1++;
                osSemaphoreRelease(rx_completed_);
            }
        }

        void Release()
        {
            // trace2[counter2] = hydrv_Clock_GetSystemTime();
            // counter2++;
            flag = false;
        }

    private:
        osSemaphoreId_t &rx_completed_;
        unsigned trace1[1000];
        unsigned counter1;
        unsigned trace2[1000];
        unsigned counter2;
        bool flag;
    };

private:
    using UARTType = hydrv::UART::UART<1024, 1024, class RxCallback_ &>;
    using ParserType = hydrolib::VectorNAVParser<UARTType, Logger>;

public:
    VectorNAVModule(osPriority_t main_thread_priority,
                    const hydrv::UART::UARTLow::UARTPreset &UART_preset,
                    hydrv::GPIO::GPIOLow &rx_pin, hydrv::GPIO::GPIOLow &tx_pin,
                    uint32_t IRQ_priority, Logger &logger);

public:
    void IRQCallback();
    void MainThreadHandler();

    hydrolib_ReturnCode Read(void *buffer, uint32_t address, uint32_t length);
    hydrolib_ReturnCode Write(const void *buffer, uint32_t address,
                              uint32_t length);

    float GetYaw();
    float GetPitch();
    float GetRoll();

    unsigned GetWrongCRCCount() const;
    unsigned GetRubbishBytesCount() const;
    unsigned GetPackagesCount() const;

private:
    osSemaphoreId_t rx_completed_;
    osThreadId_t main_thread_handler_;

    osMutexId_t data_mutex_;

    class RxCallback_ rx_callback_;

    UARTType uart_;
    ParserType parser_;
};

template <typename Logger>
void MainTask(void *argument);

template <typename Logger>
VectorNAVModule<Logger>::VectorNAVModule(
    osPriority_t main_thread_priority,
    const hydrv::UART::UARTLow::UARTPreset &UART_preset,
    hydrv::GPIO::GPIOLow &rx_pin, hydrv::GPIO::GPIOLow &tx_pin,
    uint32_t IRQ_priority, Logger &logger)
    : rx_completed_(osSemaphoreNew(1, 0, nullptr)),
    // : rx_completed_(nullptr),
      main_thread_handler_(nullptr),
      data_mutex_(osMutexNew(nullptr)),
      rx_callback_(rx_completed_),
      uart_(UART_preset, rx_pin, tx_pin, IRQ_priority, rx_callback_),
      parser_(uart_, logger)
{
    osThreadAttr_t main_thread_attributes = {
        .name = "VectorNAVMainThread",
        .stack_size = 128 * 32,
        .priority = main_thread_priority,
    };
    main_thread_handler_ =
        osThreadNew(MainTask<Logger>, this, &main_thread_attributes);
}

template <typename Logger>
void VectorNAVModule<Logger>::IRQCallback()
{
    uart_.IRQcallback();
}

template <typename Logger>
void VectorNAVModule<Logger>::MainThreadHandler()
{

    parser_.Reset();
    hydrv_Clock_Delay(500);
    parser_.Init();

    uart_.StartRx();

    uart_.ClearRx();

    while (1)
    {
        osStatus_t ret = osSemaphoreAcquire(rx_completed_, osWaitForever);
        if (!ret)
        {
        rx_callback_.Release();
        osMutexAcquire(data_mutex_, osWaitForever);
        parser_.Process();
        osMutexRelease(data_mutex_);
        osDelay(5);
        }
    }
}

template <typename Logger>
hydrolib_ReturnCode
VectorNAVModule<Logger>::Read(void *buffer, uint32_t address, uint32_t length)
{
    osMutexAcquire(data_mutex_, osWaitForever);
    hydrolib_ReturnCode res = parser_.Read(buffer, address, length);
    osMutexRelease(data_mutex_);
    return res;
}

template <typename Logger>
hydrolib_ReturnCode VectorNAVModule<Logger>::Write(const void *buffer,
                                                   uint32_t address,
                                                   uint32_t length)
{
    return parser_.Write(buffer, address, length);
}

template <typename Logger>
float VectorNAVModule<Logger>::GetYaw()
{
    float res;
    Read(&res, ParserType::YAW_ADDRESS, sizeof(res));
    return res;
}

template <typename Logger>
float VectorNAVModule<Logger>::GetPitch()
{
    float res;
    Read(&res, ParserType::PITCH_ADDRESS, sizeof(res));
    return res;
}

template <typename Logger>
float VectorNAVModule<Logger>::GetRoll()
{
    float res;
    Read(&res, ParserType::ROLL_ADDRESS, sizeof(res));
    return res;
}

template <typename Logger>
unsigned VectorNAVModule<Logger>::GetWrongCRCCount() const
{
    return parser_.GetWrongCRCCount();
}

template <typename Logger>
unsigned VectorNAVModule<Logger>::GetRubbishBytesCount() const
{
    return parser_.GetRubbishBytesCount();
}

template <typename Logger>
unsigned VectorNAVModule<Logger>::GetPackagesCount() const
{
    return parser_.GetPackagesCount();
}

template <typename Logger>
void MainTask(void *argument)
{
    VectorNAVModule<Logger> *handler =
        static_cast<VectorNAVModule<Logger> *>(argument);
    handler->MainThreadHandler();
}

} // namespace hydros::vectornav

#endif
