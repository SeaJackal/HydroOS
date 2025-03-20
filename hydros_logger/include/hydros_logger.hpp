#ifndef HYDROS_LOGGER_H_
#define HYDROS_LOGGER_H_

#include <cstdint>

#include "cmsis_os.h"

#include "hydrolib_logger.hpp"
#include "hydrv_uart.h"

namespace hydros::logger
{
    using namespace hydrolib::Logger;

    class LoggerModule
    {
    public:
        class UARTloggerQueue : public LogDistributor::SubscriberQueueInterface
        {
        public:
            UARTloggerQueue(USART_TypeDef *USARTx, osPriority_t thread_priority);

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
        LoggerModule(USART_TypeDef *USARTx, osPriority_t thread_priority);

    public:
        LogDistributor *GetDistributor();

    private:
        UARTloggerQueue queue_;
        LogDistributor distributor_;
    };
}

#endif
