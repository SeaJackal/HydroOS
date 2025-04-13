#ifndef HYDROS_LOGGER_H_
#define HYDROS_LOGGER_H_

#include <cstdint>

#include "cmsis_os.h"

#include "hydrolib_logger.hpp"

#include "hydrv_uart.hpp"

namespace hydros::logger
{
    using namespace hydrolib::Logger;

    class LoggerModule : public LogDistributor
    {
    public:
        class UARTloggerStream : public LogDistributor::SubscriberQueueInterface
        {
        public:
            UARTloggerStream(USART_TypeDef *USARTx, osPriority_t thread_priority);

        public:
            hydrolib_ReturnCode Push(Logger::Log &log) override;
            void ThreadHandler();
            hydrolib_ReturnCode TransmitByte();

        private:
            USART_TypeDef *USARTx_;
            LogTranslator translator_;

            osMessageQueueId_t queue_;
            osSemaphoreId_t tx_completed_;
            osThreadId_t thread_handler_;
        };

    public:
        LoggerModule();
    };
}

#endif
