#include <string.h>

#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32f407xx.h"
#include "stm32f4xx.h"

#include "hydrv_clock.h"
#include "hydrv_common.h"

#ifdef __cplusplus
}
#endif

#include "hydros_serial_protocol.hpp"

#define BUFFER_LENGTH 10

class ThreadSecuredMemory
    : public hydrolib::serialProtocol::MessageProcessor::PublicMemoryInterface
{
public:
  ThreadSecuredMemory()
  {
    memory_mutex_ = osMutexNew(nullptr);
  }

private:
  uint8_t public_memory_[BUFFER_LENGTH];

  osMutexId_t memory_mutex_;

public:
  hydrolib_ReturnCode Read(void *buffer, uint32_t address,
                           uint32_t length) override
  {
    osMutexAcquire(memory_mutex_, osWaitForever);
    memcpy(buffer, public_memory_ + address, length);
    osMutexRelease(memory_mutex_);
    return HYDROLIB_RETURN_OK;
  }
  hydrolib_ReturnCode Write(const void *buffer, uint32_t address,
                            uint32_t length) override
  {
    osMutexAcquire(memory_mutex_, osWaitForever);
    memcpy(public_memory_ + address, buffer, length);
    osMutexRelease(memory_mutex_);
    return HYDROLIB_RETURN_OK;
  }
  uint32_t Size() override
  {
    return BUFFER_LENGTH;
  }
};

// hydrv::UART::UART<100, 100> uart(hydrv::UART::UART<100, 100>::HYDRV_USART3,
//                                  GPIOC, HYDRV_GPIO_PIN_11, GPIOC, HYDRV_GPIO_PIN_10, 7);

void SerialProtocolRxCallback();
extern "C"
{
  void StartTask02(void *argument);
}

osThreadId_t myTask02Handle;
const osThreadAttr_t myTask02_attributes = {
    .name = "myTask02",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};

ThreadSecuredMemory secure_buffer;

hydrv::GPIO::GPIOLow rx_pin(hydrv::GPIO::GPIOLow::GPIOC_group, 11);
hydrv::GPIO::GPIOLow tx_pin(hydrv::GPIO::GPIOLow::GPIOC_group, 10);
hydrv::GPIO::GPIOLow led_pin(hydrv::GPIO::GPIOLow::GPIOD_group, 15);

hydrv::serialProtocol::SerialProtocolDriver::UART
    uart(hydrv::UART::UARTLow::USART3_LOW, rx_pin, tx_pin, 7,
         SerialProtocolRxCallback);

hydros::serialProtocol::SerialProtocolModule module("SerialProtocol", osPriorityAboveNormal,
                                                    uart, 2, secure_buffer);

int main(void)
{
  hydrv_Clock_ConfigureHSI();
  NVIC_SetPriorityGrouping(0);

  led_pin.InitAsOutput();

  osKernelInitialize();

  myTask02Handle = osThreadNew(StartTask02, NULL, &myTask02_attributes);

  osKernelStart();

  while (1)
  {

    // uart.Transmit(buffer, 6);

    // hydrv_Clock_Delay(500);

    // hydrv_GPIO_Set(GPIOD, HYDRV_GPIO_PIN_15);
    // hydrv_Clock_Delay(500);
    // hydrv_GPIO_Reset(GPIOD, HYDRV_GPIO_PIN_15);
    // hydrv_Clock_Delay(500);
  }
}

void SerialProtocolRxCallback()
{
  module.RxInterruptCallback();
}

extern "C"
{
  void UART3IRQHandler()
  {
    uart.IRQcallback();
  }
}

extern "C"
{
  void StartTask02(void *argument)
  {
    /* USER CODE BEGIN StartTask02 */
    /* Infinite loop */
    for (;;)
    {
      uint8_t byte;
      secure_buffer.Read(&byte, 0, 1);

      if (byte == 'a')
      {
        led_pin.Set();
      }
      else if (byte == 'b')
      {
        led_pin.Reset();
      }
    }
    /* USER CODE END StartTask02 */
  }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
