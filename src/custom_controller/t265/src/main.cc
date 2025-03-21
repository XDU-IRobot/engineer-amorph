
#include <cstring>

#include "packet_builder.hpp"
#include "serial.hpp"
#include "t265.hpp"

int main(int argc, char *argv[]) {
  while (true) {
    try {
      PacketBuilder packet_builder{};
      Serial ser{"/dev/ttyAMA3", 115200};
      T265 t265{};

      while (true) {
        const auto pose = t265.GetPoseData();
        const auto tx_data = [&pose] {
          std::vector<uint8_t> tx_data{30};
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
        ser.Write(tx_packet);
      }

    } catch (const rs2::error &e) {
      std::cerr << "RealSense error calling " << e.get_failed_function() << "("
                << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    } catch (const std::exception &e) {
      std::cerr << e.what() << std::endl;
    }
  }
}
