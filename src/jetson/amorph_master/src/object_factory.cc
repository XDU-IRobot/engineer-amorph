
#include "object_factory.hpp"

#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>
#include <geometry_msgs/msg/pose.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <shape_msgs/msg/mesh.h>

#include "moveit_helper.hpp"

namespace amorph {

namespace object_factory {
moveit_msgs::msg::CollisionObject Table(          //
    const std::string& table_name,                //
    const std::string& table_reference_frame,     //
    const std::vector<double>& table_dimensions,  //
    geometry_msgs::msg::Pose table_pose           //
) {
  moveit_msgs::msg::CollisionObject object;
  object.id = table_name;
  object.header.frame_id = table_reference_frame;
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  object.primitives[0].dimensions = {table_dimensions.at(0), table_dimensions.at(1), table_dimensions.at(2)};
  table_pose.position.z -= 0.5 * table_dimensions[2];  // align surface with world
  object.primitive_poses.push_back(table_pose);
  return object;
}

moveit_msgs::msg::CollisionObject Table(          //
    const std::string& table_name,                //
    const std::string& table_reference_frame,     //
    const std::vector<double>& table_dimensions,  //
    const std::vector<double>& table_pose         //
) {
  return Table(table_name, table_reference_frame, table_dimensions, MoveitHelper::VectorToPose(table_pose));
}

moveit_msgs::msg::CollisionObject EnergyUnit(   //
    const std::string& object_name,             //
    const std::string& object_reference_frame,  //
    const geometry_msgs::msg::Pose& pose        //
) {
  moveit_msgs::msg::CollisionObject object;
  object.id = object_name;
  object.header.frame_id = object_reference_frame;

  // 加载 STL 文件
  const std::string mesh_resource = "package://amorph_master/assets/energy_unit.STL";

  shapes::Mesh* mesh = shapes::createMeshFromResource(mesh_resource);
  if (mesh) {
    shape_msgs::msg::Mesh mesh_msg;
    shapes::ShapeMsg mesh_shape_msg;
    shapes::constructMsgFromShape(mesh, mesh_shape_msg);
    mesh_msg = boost::get<shape_msgs::msg::Mesh>(mesh_shape_msg);

    object.meshes.push_back(mesh_msg);
    object.mesh_poses.push_back(pose);

    delete mesh;  // 清理内存
  } else {
    throw std::runtime_error("Failed to load mesh from " + mesh_resource);
    // RCLCPP_WARN(kLogger, "Failed to load mesh from %s, using cylinder instead", mesh_resource.c_str());
    // object.primitives.resize(1);
    // object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    // object.primitives[0].dimensions = {object_dimensions.at(0), object_dimensions.at(1)};
    // pose.position.z += 0.5 * object_dimensions[0];
    // object.primitive_poses.push_back(pose);
  }

  return object;
}

moveit_msgs::msg::CollisionObject EnergyUnit(   //
    const std::string& object_name,             //
    const std::string& object_reference_frame,  //
    const std::vector<double>& pose             //
) {
  return EnergyUnit(object_name, object_reference_frame, MoveitHelper::VectorToPose(pose));
}

}  // namespace object_factory

}  // namespace amorph