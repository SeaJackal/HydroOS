#ifndef HYDROS_SERIAL_PROTOCOL_H_
#define HYDROS_SERIAL_PROTOCOL_H_

#include <cstdint>

#include "cmsis_os.h"

#include "hydrv_serial_protocol.hpp"

namespace hydros::serialProtocol
{
    class SerialProtocolModule
    {
    public:
        SerialProtocolModule(
            const char *name,
            osPriority_t main_thread_priority,
            hydrv::serialProtocol::SerialProtocolDriver::UART &UART,
            uint8_t address,
            hydrolib::serialProtocol::MessageProcessor::PublicMemoryInterface &public_memory);

    private:
        const char *name_;

        osThreadId_t main_thread_handler_;
        osSemaphoreId_t rx_completed_;

        hydrv::serialProtocol::SerialProtocolDriver driver_;

    public:
        hydrolib_ReturnCode TransmitWrite(uint8_t device_address,
                                          uint32_t memory_address, uint32_t length,
                                          uint8_t *data);
        hydrolib_ReturnCode TransmitRead(uint8_t device_address,
                                         uint32_t memory_address, uint32_t length,
                                         uint8_t *buffer);

        void RxInterruptCallback();

        void MainThreadHandler();
    };
}

#endif
