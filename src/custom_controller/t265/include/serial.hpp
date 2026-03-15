
#ifndef SERIAL_HPP
#define SERIAL_HPP

#include <vector>
#include <string>
#include <memory>

#include <boost/asio.hpp>

class Serial {
 public:
  Serial(const std::string &port, size_t baud_rate) {
    serial_port_.open(port);
    serial_port_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
  }

  ~Serial() { serial_port_.close(); }

  void Write(std::shared_ptr<std::vector<uint8_t>> data) {
    boost::asio::write(serial_port_, boost::asio::buffer(*data));
  }

 private:
  boost::asio::io_service io_service_;
  boost::asio::serial_port serial_port_{io_service_};
};

#endif