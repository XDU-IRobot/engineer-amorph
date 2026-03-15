
#include <iostream>
#include <chrono>
#include <cstring>
#include <thread>

#include <spdlog/spdlog.h>
#include <spdlog/fmt/bundled/ranges.h>

#include "packet_builder.hpp"
#include "serial.hpp"
#include "t265.hpp"

using namespace std::chrono_literals;

int main(int argc, char *argv[]) {
  while (true) {
    try {
      PacketBuilder packet_builder{};
      Serial ser{"/dev/ttyAMA3", 115200};
      T265 t265{};

      spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%l] %v");

      while (true) {
        const auto this_loop_start_time = std::chrono::steady_clock::now();
        const auto pose = t265.GetPoseData();
        const auto tx_data = [&pose] {
          std::vector<uint8_t> tx_data(30);
          std::memcpy(tx_data.data(), &pose.translation.x, sizeof(float));
          std::memcpy(tx_data.data() + 4, &pose.translation.y, sizeof(float));
          std::memcpy(tx_data.data() + 8, &pose.translation.z, sizeof(float));
          std::memcpy(tx_data.data() + 12, &pose.rotation.w, sizeof(float));
          std::memcpy(tx_data.data() + 16, &pose.rotation.x, sizeof(float));
          std::memcpy(tx_data.data() + 20, &pose.rotation.y, sizeof(float));
          std::memcpy(tx_data.data() + 24, &pose.rotation.z, sizeof(float));
          return tx_data;
        }();
        const auto tx_packet = packet_builder.Build(tx_data);
        spdlog::info("{}", fmt::join(*tx_packet, ", "));
        ser.Write(tx_packet);
        std::this_thread::sleep_until(this_loop_start_time + 40ms);
      }

    } catch (const rs2::error &e) {
      spdlog::error("RealSense error calling {}({}):\n    {}", e.get_failed_function(), e.get_failed_args(), e.what());
    } catch (const std::exception &e) {
      spdlog::error("Exception: {}", e.what());
    }
  }
}
