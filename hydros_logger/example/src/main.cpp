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

#include "hydrolib_logger.hpp"
#include "hydrolib_log_distributor.hpp"
#include "hydros_uart_stream.hpp"

extern "C"
{
  void StartTask01(void *argument);
  void StartTask02(void *argument);
}

osThreadId_t myTask01Handle;
const osThreadAttr_t myTask01_attributes = {
    .name = "myTask01",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};

osThreadId_t myTask02Handle;
const osThreadAttr_t myTask02_attributes = {
    .name = "myTask02",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityHigh,
};

hydrv::GPIO::GPIOLow rx_pin(hydrv::GPIO::GPIOLow::GPIOC_group, 11);
hydrv::GPIO::GPIOLow tx_pin(hydrv::GPIO::GPIOLow::GPIOC_group, 10);
hydros::UARTStream<255, 255> uart(hydrv::UART::UARTLow::USART3_LOW,
                                  rx_pin, tx_pin, 7);

hydrolib::Logger::LogDistributor distributor("[%s] [%l] %m\n\r", uart);
hydrolib::Logger::Logger logger1("First logger", 0, distributor);
hydrolib::Logger::Logger logger2("Second logger", 1, distributor);

int main(void)
{
  hydrv_Clock_ConfigureHSI();
  NVIC_SetPriorityGrouping(0);

  distributor.SetAllFilters(0, hydrolib::Logger::LogLevel::DEBUG);
  distributor.SetAllFilters(1, hydrolib::Logger::LogLevel::DEBUG);

  osKernelInitialize();

  myTask01Handle = osThreadNew(StartTask01, NULL, &myTask01_attributes);
  myTask02Handle = osThreadNew(StartTask02, NULL, &myTask02_attributes);

  osKernelStart();
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
  void StartTask01(void *argument)
  {
    while (1)
    {
      osDelay(1000);
      logger1.WriteLog(hydrolib::Logger::LogLevel::INFO, "I'm first process, current time: {}", hydrv_Clock_GetSystemTime());
    }
  }

  void StartTask02(void *argument)
  {
    while (1)
    {
      osDelay(1500);
      logger2.WriteLog(hydrolib::Logger::LogLevel::DEBUG, "I'm second process, current time: {}", hydrv_Clock_GetSystemTime());
    }
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
