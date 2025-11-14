
// ROS
#include <rclcpp/rclcpp.hpp>

// MTC pick/place demo implementation
#include "pick_place_task/pick_place_task.hpp"

static const rclcpp::Logger kLogger = rclcpp::get_logger("moveit_task_constructor_demo");

int main(int argc, char** argv) {
	rclcpp::init(argc, argv);
	rclcpp::NodeOptions node_options;
	node_options.automatically_declare_parameters_from_overrides(true);
	auto node = rclcpp::Node::make_shared("moveit_task_constructor_demo", node_options);
	std::thread spinning_thread([node] { rclcpp::spin(node); });

	// Construct and run pick/place task
	PickPlaceTask pick_place_task("pick_place_task");
	if (!pick_place_task.init(node)) {
		RCLCPP_INFO(kLogger, "Initialization failed");
		return 1;
	}

	if (pick_place_task.plan(10)) {
    pick_place_task.task()->publishAllSolutions();
		RCLCPP_INFO(kLogger, "Planning succeded");
	} else {
		RCLCPP_INFO(kLogger, "Planning failed");
	}

	// Keep introspection alive
	spinning_thread.join();
	return 0;
}
