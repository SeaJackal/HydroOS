#include "hydros_logger.hpp"

extern "C"
{
    void UARTloggerTask(void *argument);
}

namespace hydros::logger
{
    LoggerModule::UARTloggerStream::UARTloggerStream(USART_TypeDef *USARTx,
                                                     osPriority_t thread_priority)
        : USARTx_(USARTx),
          translator_()
    {

        queue_ = osMessageQueueNew(10, sizeof(Logger::Log), nullptr);
        translator_.SetFormatString("[%s] [%l] %m\n\r");
        tx_completed_ = osSemaphoreNew(1, 0, nullptr);

        osThreadAttr_t thread_attributes = {
            .name = "SerialProtocolMainThread",
            .stack_size = 128 * 16,
            .priority = thread_priority,
        };
        thread_handler_ = osThreadNew(UARTloggerTask, this, &thread_attributes);
    }

    hydrolib_ReturnCode LoggerModule::UARTloggerStream::Push(Logger::Log &log)
    {
        osMessageQueuePut(queue_, &log, 0, osWaitForever);
        return HYDROLIB_RETURN_OK;
    }

    void LoggerModule::UARTloggerStream::ThreadHandler()
    {
        while (1)
        {
            Logger::Log translated_log;
            osMessageQueueGet(queue_, &translated_log, nullptr, osWaitForever);
            translator_.StartTranslatingToBytes(translated_log);
            hydrv_UART_enableTxInterruption(USARTx_);
            osStatus_t ret = osSemaphoreAcquire(tx_completed_, osWaitForever);
        }
    }

    hydrolib_ReturnCode LoggerModule::UARTloggerStream::TransmitByte()
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

    LoggerModule::LoggerModule()
        : distributor_()
    {
    }

    LogDistributor &LoggerModule::GetDistributor()
    {
        return distributor_;
    }

    void LoggerModule::AddUARTstreams(UARTloggerStream &UART_stream)
    {
        distributor_.AddSubscriber(UART_stream, LogLevel::DEBUG, nullptr);
    }
}
