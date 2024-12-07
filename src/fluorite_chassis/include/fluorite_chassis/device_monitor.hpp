
#ifndef DEVICE_MONITOR_HPP
#define DEVICE_MONITOR_HPP

#include <vector>
#include <chrono>
#include <string>

class Device {
  friend class DeviceMonitor;

 public:
  explicit Device(std::string name);
  Device() = default;
  ~Device() = default;

  /**
   * @brief call me if you are still alive
   */
  void IAmAlive();

 private:
  static constexpr std::chrono::duration<double> kOfflineTimeout = std::chrono::milliseconds(600);

  std::chrono::time_point<std::chrono::steady_clock> last_seen_{};
  std::string name_{};
  bool is_alive_{false};
};

class DeviceMonitor {
 public:
  DeviceMonitor() = default;
  ~DeviceMonitor() = default;
  void Register(Device &device);
  void Update();

  DeviceMonitor &operator<<(Device &device);

  bool all_device_ok() const;

 private:
  std::vector<Device *> devices_;
};

#endif