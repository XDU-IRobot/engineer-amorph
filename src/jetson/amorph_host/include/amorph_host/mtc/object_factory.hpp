
#pragma once

#include <moveit_msgs/msg/collision_object.hpp>

namespace amorph {

namespace object_factory {

moveit_msgs::msg::CollisionObject Table(          //
    const std::string& table_name,                //
    const std::string& table_reference_frame,     //
    const std::vector<double>& table_dimensions,  //
    geometry_msgs::msg::Pose table_pose           //
);

moveit_msgs::msg::CollisionObject Table(          //
    const std::string& table_name,                //
    const std::string& table_reference_frame,     //
    const std::vector<double>& table_dimensions,  //
    const std::vector<double>& table_pose         //
);

moveit_msgs::msg::CollisionObject EnergyUnit(   //
    const std::string& object_name,             //
    const std::string& object_reference_frame,  //
    const geometry_msgs::msg::Pose& pose        //
);

moveit_msgs::msg::CollisionObject EnergyUnit(   //
    const std::string& object_name,             //
    const std::string& object_reference_frame,  //
    const std::vector<double>& pose             //
);

}  // namespace object_factory

}  // namespace amorph