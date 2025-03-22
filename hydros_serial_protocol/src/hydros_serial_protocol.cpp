#include "hydros_serial_protocol.hpp"

#include <cstring>

extern "C"
{
    void MainTask(void *argument);
}

namespace hydros::serialProtocol
{
    SerialProtocolModule::ThreadSecuredMemory_::ThreadSecuredMemory_(uint8_t *public_memory,
                                                                     uint32_t public_memory_capacity)
        : public_memory_(public_memory),
          public_memory_capacity_(public_memory_capacity)
    {
        memory_mutex_ = osMutexNew(nullptr);
    }

    hydrolib_ReturnCode SerialProtocolModule::ThreadSecuredMemory_::Read(void *buffer, uint32_t address,
                                                                         uint32_t length)
    {
        osMutexAcquire(memory_mutex_, osWaitForever);
        memcpy(buffer, public_memory_ + address, length);
        osMutexRelease(memory_mutex_);
        return HYDROLIB_RETURN_OK;
    }

    hydrolib_ReturnCode SerialProtocolModule::ThreadSecuredMemory_::Write(const void *buffer, uint32_t address,
                                                                          uint32_t length)
    {
        osMutexAcquire(memory_mutex_, osWaitForever);
        memcpy(public_memory_ + address, buffer, length);
        osMutexRelease(memory_mutex_);
        return HYDROLIB_RETURN_OK;
    }

    uint32_t SerialProtocolModule::ThreadSecuredMemory_::Size()
    {
        return public_memory_capacity_;
    }

    SerialProtocolModule::SerialProtocolModule(
        const char *name,
        osPriority_t main_thread_priority,
        USART_TypeDef *USARTx, uint8_t address,
        uint8_t *public_memory,
        uint32_t public_memory_capacity)
        : name_(name),
          public_memory_(public_memory, public_memory_capacity),
          driver_(USARTx, address, public_memory_),
          USARTx_(USARTx)
    {
        // hydrv_UART_Init(USARTx);

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

    void SerialProtocolModule::UARTinterruptHandler()
    {
        if (hydrv_UART_IsReceived(USARTx_))
        {
            driver_.ReceiveByteCallback();
            osSemaphoreRelease(rx_completed_);
        }
        if (hydrv_UART_IsTransmitted(USARTx_))
        {
            driver_.TransmitHandler();
        }
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
