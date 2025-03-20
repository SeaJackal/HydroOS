#include "hydros_logger.hpp"

extern "C"
{
    void UARTloggerTask(void *argument);
}

namespace hydros::logger
{
    LoggerModule::UARTloggerQueue::UARTloggerQueue(USART_TypeDef *USARTx,
                                                   osPriority_t thread_priority)
        : USARTx_(USARTx),
          translator_()
    {
        queue_ = osMessageQueueNew(10, sizeof(Logger::Log), nullptr);
        translator_.SetFormatString("[%s] [%l] %m\n");
        tx_completed_ = osSemaphoreNew(1, 1, nullptr);

        osThreadAttr_t thread_attributes = {
            .name = "SerialProtocolMainThread",
            .stack_size = 128 * 16,
            .priority = thread_priority,
        };
        thread_handler_ = osThreadNew(UARTloggerTask, this, &thread_attributes);
    }

    void LoggerModule::UARTloggerQueue::ThreadHandler()
    {
        Logger::Log translated_log;
        osMessageQueueGet(queue_, &translated_log, nullptr, osWaitForever);
        translator_.StartTranslatingToBytes(translated_log);
        hydrv_UART_enableTxInterruption(USARTx_);
        osStatus_t ret = osSemaphoreAcquire(tx_completed_, osWaitForever);
    }

    hydrolib_ReturnCode LoggerModule::UARTloggerQueue::TransmitByte()
    {
        uint8_t byte;
        int32_t res = translator_.DoTranslation(reinterpret_cast<char *>(&byte), 1);
        if (res == 0)
        {
            hydrv_UART_disableTxInterruption(USARTx_);
            osSemaphoreRelease(tx_completed_);
        }
        hydrv_UART_Transmit(USARTx_, byte);
    }

    LoggerModule::LoggerModule(USART_TypeDef *USARTx, osPriority_t thread_priority)
        : queue_(USARTx, thread_priority),
          distributor_()
    {
        distributor_.AddSubscriber(queue_, LogLevel::DEBUG, nullptr);
    }

    LogDistributor *LoggerModule::GetDistributor()
    {
        return &distributor_;
    }
}
