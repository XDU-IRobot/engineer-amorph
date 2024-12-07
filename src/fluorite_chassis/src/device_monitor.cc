
#include "fluorite_chassis/device_monitor.hpp"

#include <chrono>

#include "fluorite_chassis/logger.hpp"

Device::Device(std::string name) : name_{name} {}

void Device::IAmAlive() {
  last_seen_ = std::chrono::steady_clock::now();
  is_alive_ = true;
}

void DeviceMonitor::Register(Device &device) { devices_.push_back(&device); }

void DeviceMonitor::Update() {
  for (auto &device : devices_) {
    bool previous_state = device->is_alive_;
    if (std::chrono::steady_clock::now() - device->last_seen_ > Device::kOfflineTimeout) {
      device->is_alive_ = false;
    }
    if (previous_state == false && device->is_alive_ == true) {
      info("device \"%s\" is now online", device->name_.c_str());
    }
    if (previous_state == true && device->is_alive_ == false) {
      warn("device \"%s\" is now offline", device->name_.c_str());
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