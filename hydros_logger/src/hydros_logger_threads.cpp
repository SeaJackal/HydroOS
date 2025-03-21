#include "hydros_logger.hpp"

using namespace hydros::logger;

extern "C"
{
    void UARTloggerTask(void *argument)
    {
        LoggerModule::UARTloggerStream *handler =
            static_cast<LoggerModule::UARTloggerStream *>(argument);
        handler->ThreadHandler();
    }
}
