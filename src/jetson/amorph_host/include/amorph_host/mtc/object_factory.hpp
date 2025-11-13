
#pragma once

#include <moveit_msgs/msg/collision_object.hpp>

namespace object_factory {

moveit_msgs::msg::CollisionObject CreateTable(    //
    const std::string& table_name,                //
    const std::string& table_reference_frame,     //
    const std::vector<double>& table_dimensions,  //
    const std::vector<double>& table_pose         //
);

moveit_msgs::msg::CollisionObject CreateEnergyUnit(           //
    const std::string& object_name,                           //
    const std::string& object_reference_frame,                //
    const std::vector<double>& pose = std::vector<double>(6)  //
);

}  // namespace object_factory