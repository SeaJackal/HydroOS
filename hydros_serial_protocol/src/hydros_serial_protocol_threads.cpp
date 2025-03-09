#include "hydros_serial_protocol.hpp"

using namespace hydros::serialProtocol;

extern "C"
{
    void MainTask(void *argument)
    {
        SerialProtocolModule *handler = static_cast<SerialProtocolModule *>(argument);
        handler->MainThreadHandler();
    }
}
