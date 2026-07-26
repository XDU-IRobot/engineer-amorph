#pragma once

#include <librm.hpp>

class Ahrs {
 public:
  enum Installation {
    PXPYPZ,  ///< 前左上+X+Y+Z
    PYPXNZ,  ///<
    PYPXPZ,
  };

  explicit Ahrs(Installation installation) : installation_{installation} {}
  void Update() {
    onboard_imu_.Update();
    switch (installation_) {
      case PYPXNZ: {
        imu_data_ = {
            .gx = -onboard_imu_.gyro_y(),
            .gy = -onboard_imu_.gyro_x(),
            .gz = onboard_imu_.gyro_z(),
            .ax = -onboard_imu_.accel_y(),
            .ay = -onboard_imu_.accel_x(),
            .az = onboard_imu_.accel_z(),
        };
        break;
      }
      case PYPXPZ: {
        imu_data_ = {
          .gx = -onboard_imu_.gyro_y(),
          .gy = -onboard_imu_.gyro_x(),
          .gz = -onboard_imu_.gyro_z(),
          .ax = -onboard_imu_.accel_y(),
          .ay = -onboard_imu_.accel_x(),
          .az = -onboard_imu_.accel_z(),
      };
        break;
      }
      case PXPYPZ:
      default: {
        imu_data_ = {
            .gx = onboard_imu_.gyro_x(),
            .gy = onboard_imu_.gyro_y(),
            .gz = onboard_imu_.gyro_z(),
            .ax = onboard_imu_.accel_x(),
            .ay = onboard_imu_.accel_y(),
            .az = onboard_imu_.accel_z(),
        };
        break;
      }
    }

    mahony_.Update(imu_data_);
    pose_euler_ = mahony_.euler_angle();
  }

  auto imu_data() const { return imu_data_; }
  auto pose_euler() const { return pose_euler_; }
  auto pose_quat() const { return mahony_.quaternion(); }

 private:
  Installation installation_;

  rm::modules::ImuData6Dof imu_data_{};
  rm::modules::EulerAngle pose_euler_{};

  rm::modules::MahonyAhrs mahony_{1000.f};
  rm::device::BMI088 onboard_imu_{hspi2, ACC_CS_GPIO_Port, ACC_CS_Pin, GYRO_CS_GPIO_Port, GYRO_CS_Pin};
};