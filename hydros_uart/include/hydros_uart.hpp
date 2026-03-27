#pragma once

#include <concepts>

#include "cmsis_os.h"
#include "hydrv_rs_485.hpp"
#include "hydrv_uart.hpp"

namespace hydros::UART {
template <typename BaseStream, int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY>
class OsStream : public BaseStream {
 public:
  consteval OsStream(const hydrv::UART::UARTLow::UARTPreset &UART_preset,
                     hydrv::GPIO::GPIOLow &rx_pin, hydrv::GPIO::GPIOLow &tx_pin,
                     int irq_priority)
    requires std::same_as<
        BaseStream, hydrv::UART::UART<RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY>>;

  consteval OsStream(const hydrv::UART::UARTLow::UARTPreset &RS485_preset,
                     hydrv::GPIO::GPIOLow &rx_pin, hydrv::GPIO::GPIOLow &tx_pin,
                     hydrv::GPIO::GPIOLow &direction_pin, int irq_priority)
    requires std::same_as<
        BaseStream, hydrv::RS485::RS485<RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY>>
  ;

  using BaseStream::ClearRx;
  using BaseStream::GetRxLength;
  using BaseStream::GetTxLength;
  using BaseStream::IRQCallback;
  using BaseStream::Read;

  void Init();
  int Transmit(const void *data, unsigned data_length);

 private:
  osMutexId_t write_mutex_ = nullptr;
};

template <int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY>
using UART = OsStream<hydrv::UART::UART<RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY>,
                      RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY>;

template <int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY>
using RS485 =
    OsStream<hydrv::RS485::RS485<RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY>,
             RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY>;

template <typename BaseStream, int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY>
consteval OsStream<BaseStream, RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY>::
    OsStream(const hydrv::UART::UARTLow::UARTPreset &UART_preset,
             hydrv::GPIO::GPIOLow &rx_pin, hydrv::GPIO::GPIOLow &tx_pin,
             int irq_priority)
  requires std::same_as<
      BaseStream, hydrv::UART::UART<RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY>>
    : BaseStream(UART_preset, rx_pin, tx_pin, irq_priority) {}

template <typename BaseStream, int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY>
consteval OsStream<BaseStream, RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY>::
    OsStream(const hydrv::UART::UARTLow::UARTPreset &RS485_preset,
             hydrv::GPIO::GPIOLow &rx_pin, hydrv::GPIO::GPIOLow &tx_pin,
             hydrv::GPIO::GPIOLow &direction_pin, int irq_priority)
  requires std::same_as<
      BaseStream, hydrv::RS485::RS485<RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY>>
    : BaseStream(RS485_preset, rx_pin, tx_pin, direction_pin, irq_priority) {}

template <typename BaseStream, int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY>
void OsStream<BaseStream, RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY>::Init() {
  BaseStream::Init();
  write_mutex_ = osMutexNew(nullptr);
}

template <typename BaseStream, int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY>
int OsStream<BaseStream, RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY>::Transmit(
    const void *data, unsigned data_length) {
  osMutexAcquire(write_mutex_, osWaitForever);
  int result = BaseStream::Transmit(data, data_length);
  osMutexRelease(write_mutex_);
  return result;
}
}  // namespace hydros::UART
