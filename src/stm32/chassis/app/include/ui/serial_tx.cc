
#include "serial_tx.hpp"

#include <atomic>
#include <cstring>

#include <etl/queue.h>

#include "shared_resources.hpp"

struct Frame0x301 {
  Frame0x301() = default;
  Frame0x301(const uint8_t *pData, uint16_t Size) : size(Size) {
    if (Size > 128) {
      // warn!
    }
    std::memcpy(&data, pData, std::min(Size, static_cast<uint16_t>(128)));
  }
  uint8_t data[128]{};
  uint16_t size{};
};

etl::queue<Frame0x301, 32> ui_buffer;

extern "C" {
void UiEnqueue(const uint8_t *pData, uint16_t Size) {
  if (ui_buffer.full()) {
    return;
  }
  ui_buffer.emplace(pData, Size);
}
}

void UiProcessTx() {
  auto &shared = SharedResources::GetInstance().no_dtcm;
  static Frame0x301 tx_this_thing;
  // static std::atomic<bool> tx_done{false};

  // if (!tx_done.load()) {
  //   return;
  // }
  if (ui_buffer.empty()) {
    return;
  }
  tx_this_thing = ui_buffer.front();
  ui_buffer.pop();

  // tx_done.store(false);
  shared->pm_link_serial.Write(tx_this_thing.data, tx_this_thing.size, 1);
}
