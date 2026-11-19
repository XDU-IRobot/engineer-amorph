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
    const Eigen::Vector3f gyro(onboard_imu_.gyro_x(), onboard_imu_.gyro_y(), onboard_imu_.gyro_z());
    const Eigen::Vector3f accel(onboard_imu_.accel_x(), onboard_imu_.accel_y(), onboard_imu_.accel_z());
    // 目标坐标系为 ROS REP-105：x 向前、y 向左、z 向上
    switch (installation_) {
      case PYPXNZ: {
        imu_data_ = {Eigen::Vector3f(-gyro.y(), -gyro.x(), gyro.z()),
                     Eigen::Vector3f(-accel.y(), -accel.x(), accel.z())};
        break;
      }
      case PYPXPZ: {
        imu_data_ = {Eigen::Vector3f(-gyro.y(), gyro.x(), -gyro.z()),
                     Eigen::Vector3f(-accel.y(), accel.x(), -accel.z())};
        break;
      }
      case PXPYPZ:
      default: {
        imu_data_ = {gyro, accel};
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
  Eigen::Vector3f pose_euler_{Eigen::Vector3f::Zero()};

  rm::modules::MahonyAhrs mahony_{1000.f};
  rm::device::BMI088 onboard_imu_{hspi2, ACC_CS_GPIO_Port, ACC_CS_Pin, GYRO_CS_GPIO_Port, GYRO_CS_Pin};
};