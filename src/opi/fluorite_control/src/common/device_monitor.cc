
#include "common/device_monitor.hpp"

#include <chrono>

#include "common/logger.hpp"

Device::Device(std::string name, std::chrono::duration<double> offline_timeout)
    : name_{name}, offline_timeout_{offline_timeout} {}

void Device::IAmAlive() { last_seen_ = std::chrono::steady_clock::now(); }

void DeviceMonitor::Update() {
  for (auto &device : devices_) {
    bool previous_state = device->is_alive_;
    if (std::chrono::steady_clock::now() - device->last_seen_ > device->offline_timeout_) {
      device->is_alive_ = false;
    } else {
      device->is_alive_ = true;
    }
    if (previous_state == false && device->is_alive_ == true) {
      info("device is now online");
    }
    if (previous_state == true && device->is_alive_ == false) {
      warn("device is now offline");
    }
  }
}

DeviceMonitor &DeviceMonitor::operator<<(Device &device) {
  Register(device);
  return *this;
}

bool DeviceMonitor::all_device_ok() const {
  for (auto &device : devices_) {
    if (!device->is_alive_) {
      return false;
    }
  }
  return true;
}