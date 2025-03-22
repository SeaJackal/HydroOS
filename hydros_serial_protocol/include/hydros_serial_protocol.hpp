#ifndef HYDROS_SERIAL_PROTOCOL_H_
#define HYDROS_SERIAL_PROTOCOL_H_

#include <cstdint>

#include "cmsis_os.h"

#include "hydrv_serial_protocol.hpp"

namespace hydros::serialProtocol
{
    class SerialProtocolModule
    {
    private:
        class ThreadSecuredMemory_
            : public hydrolib::serialProtocol::MessageProcessor::PublicMemoryInterface
        {
        public:
            ThreadSecuredMemory_(uint8_t *public_memory,
                                 uint32_t public_memory_capacity);

        private:
            uint8_t *public_memory_;
            uint32_t public_memory_capacity_;

            osMutexId_t memory_mutex_;

        public:
            hydrolib_ReturnCode Read(void *buffer, uint32_t address,
                                     uint32_t length) override;
            hydrolib_ReturnCode Write(const void *buffer, uint32_t address,
                                      uint32_t length) override;
            uint32_t Size() override;
        };

    public:
        SerialProtocolModule(const char *name,
                             osPriority_t main_thread_priority,
                             USART_TypeDef *USARTx, uint8_t address,
                             uint8_t *public_memory,
                             uint32_t public_memory_capacity);

    private:
        const char *name_;

        osThreadId_t main_thread_handler_;
        osSemaphoreId_t rx_completed_;

        ThreadSecuredMemory_ public_memory_;

        hydrv::serialProtocol::SerialProtocolDriver driver_;
        USART_TypeDef *USARTx_;

    public:
        hydrolib_ReturnCode TransmitWrite(uint8_t device_address,
                                          uint32_t memory_address, uint32_t length,
                                          uint8_t *data);
        hydrolib_ReturnCode TransmitRead(uint8_t device_address,
                                         uint32_t memory_address, uint32_t length,
                                         uint8_t *buffer);

        void UARTinterruptHandler();

        void MainThreadHandler();
        void TxThreadHandler();
    };
}

#endif
