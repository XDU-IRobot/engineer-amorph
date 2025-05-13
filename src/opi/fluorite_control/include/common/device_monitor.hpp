
#ifndef COMMON_DEVICE_MONITOR_HPP
#define COMMON_DEVICE_MONITOR_HPP

#include <vector>
#include <chrono>
#include <string>

namespace fluorite {

class Device {
  friend class DeviceMonitor;

 public:
  explicit Device(std::string name, std::chrono::duration<double> offline_timeout = std::chrono::milliseconds(600));
  Device() = default;
  ~Device() = default;

  /**
   * @brief call me if you are still alive
   */
  void IAmAlive();

 private:
  std::chrono::time_point<std::chrono::steady_clock> last_seen_{};
  const std::chrono::duration<double> offline_timeout_{};
  std::string name_{};
  bool is_alive_{false};
};

class DeviceMonitor {
 public:
  DeviceMonitor() = default;
  ~DeviceMonitor() = default;

  template <typename... Devices>
  void Register(Devices... device) {
    (devices_.push_back(&device), ...);
  }
  void Update();

  DeviceMonitor &operator<<(Device &device);

  bool all_device_ok() const;

 private:
  std::vector<Device *> devices_;
};

}  // namespace fluorite

#endif