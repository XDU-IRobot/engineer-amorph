
#pragma once

#include <array>
#include <cmath>
#include <utility>

#include "arm_math.h"

struct Point {
  int x;
  int y;
  Point(int x = 0, int y = 0) : x(x), y(y) {}
};

struct Point3D {
  double x, y, z;
  Point3D(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {}
};

class ArmRenderer {
 private:
  // 机械臂参数
  const int BASE_HEIGHT = 35;
  const int ARM1_LENGTH = 85;
  const int ARM2_LENGTH = 85;

  // 相机参数
  const double CAM_ANGLE = 30.0;      // 俯角（度）
  const double CAM_DISTANCE = 250.0;  // 相机距离
  const int SCREEN_WIDTH = 200;
  const int SCREEN_HEIGHT = 240;

  // 将三维坐标转换为屏幕坐标
  Point Project(const Point3D& p) const {
    const double theta = CAM_ANGLE * M_PI / 180.0;

    // 应用俯角旋转（绕X轴）
    const double y_rot = p.y * arm_cos_f32(theta) - p.z * arm_sin_f32(theta);
    const double z_rot = p.y * arm_sin_f32(theta) + p.z * arm_cos_f32(theta);

    // 透视投影计算
    const double scale = CAM_DISTANCE / (CAM_DISTANCE + z_rot);
    return {static_cast<int>(p.x * scale + SCREEN_WIDTH / 2), static_cast<int>(SCREEN_HEIGHT / 2 - y_rot * scale)};
  }

 public:
  // 输入关节角度，输出三条线段的屏幕坐标
  std::array<std::pair<Point, Point>, 3> Draw(double yaw, double pitch1, double pitch2) const {
    // 转换为弧度
    const double rad_yaw = yaw;
    const double rad_p1 = pitch1;
    const double rad_p2 = (pitch1 + pitch2);

    // 计算各关节三维坐标
    const Point3D base(0, 0, 0);
    const Point3D joint1(0, BASE_HEIGHT, 0);

    const Point3D joint2(ARM1_LENGTH * arm_sin_f32(rad_yaw) * arm_cos_f32(rad_p1),
                         BASE_HEIGHT + ARM1_LENGTH * arm_sin_f32(rad_p1),
                         ARM1_LENGTH * arm_cos_f32(rad_yaw) * arm_cos_f32(rad_p1));

    const Point3D joint3(joint2.x + ARM2_LENGTH * arm_sin_f32(rad_yaw) * arm_cos_f32(rad_p2),
                         joint2.y + ARM2_LENGTH * arm_sin_f32(rad_p2),
                         joint2.z + ARM2_LENGTH * arm_cos_f32(rad_yaw) * arm_cos_f32(rad_p2));

    // 投影到屏幕坐标
    const Point p0 = Project(base);
    const Point p1 = Project(joint1);
    const Point p2 = Project(joint2);
    const Point p3 = Project(joint3);

    // 返回三条线段：立柱、后臂、前臂
    return {{
        {p0, p1},  // 立柱（基座到关节1）
        {p1, p2},  // 后臂（关节1到关节2）
        {p2, p3}   // 前臂（关节2到关节3）
    }};
  }
};
