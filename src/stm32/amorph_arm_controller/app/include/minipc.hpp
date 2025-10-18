
#pragma once

#include <cstdint>

#include <etl/span.h>
#include <etl/circular_buffer.h>

#include "usbd_cdc_if.h"

class CdcMessenger {
 public:
  void Transmit() {

  }
 private:
  static constexpr size_t kMaxQueuedMsgs = 256;
  etl::circular_buffer<etl::span<uint8_t>, kMaxQueuedMsgs> tx_queue_;
};