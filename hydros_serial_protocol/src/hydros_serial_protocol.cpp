#include "hydros_serial_protocol.hpp"

#include <cstring>

extern "C"
{
    void MainTask(void *argument);
}

namespace hydros::serialProtocol
{
    SerialProtocolModule::SerialProtocolModule(
        const char *name,
        osPriority_t main_thread_priority,
        hydrv::serialProtocol::SerialProtocolDriver::UART &UART,
        uint8_t address,
        hydrolib::serialProtocol::MessageProcessor::PublicMemoryInterface &public_memory)
        : name_(name),
          driver_(address, public_memory, UART)
    {
        rx_completed_ = osSemaphoreNew(1, 0, nullptr);

        osThreadAttr_t main_thread_attributes = {
            .name = "SerialProtocolMainThread",
            .stack_size = 128 * 16,
            .priority = main_thread_priority,
        };
        main_thread_handler_ = osThreadNew(MainTask, this, &main_thread_attributes);
    }

    hydrolib_ReturnCode SerialProtocolModule::TransmitWrite(
        uint8_t device_address,
        uint32_t memory_address, uint32_t length,
        uint8_t *data)
    {
        driver_.TransmitWrite(device_address, memory_address, length, data);
    }

    hydrolib_ReturnCode SerialProtocolModule::TransmitRead(
        uint8_t device_address,
        uint32_t memory_address, uint32_t length,
        uint8_t *buffer)
    {
        driver_.TransmitRead(device_address, memory_address, length, buffer);
    }

    void SerialProtocolModule::RxInterruptCallback()
    {
        osSemaphoreRelease(rx_completed_);
    }

    void SerialProtocolModule::MainThreadHandler()
    {
        while (1)
        {
            osStatus_t ret = osSemaphoreAcquire(rx_completed_, osWaitForever);
            if (!ret)
            {
                driver_.ProcessRx();
            }
        }
    }
}
