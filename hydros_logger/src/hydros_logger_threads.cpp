#include "hydros_logger.hpp"

using namespace hydros::logger;

extern "C"
{
    void UARTloggerTask(void *argument)
    {
        LoggerModule::UARTloggerQueue *handler =
            static_cast<LoggerModule::UARTloggerQueue *>(argument);
        handler->ThreadHandler();
    }
}
