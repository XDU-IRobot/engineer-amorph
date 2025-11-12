// 使用tinyfsm之后，就没办法再使用oop的各种编程范式来写代码了，因为状态机类没有对象，实际执行逻辑和ROS节点割裂了，没法用依赖注入的方式传递数据
// 如果ROS节点和FSM要相互传递数据，那么就必须用全局变量（定义都在fsm.cc里）
// 如果以后找到了更好的实现方式，可以考虑重构

#include "chassis/ros_params.hpp"

namespace fluorite::chassis {

/**
 * @brief   本节点的所有ROS参数，程序启动时从yaml文件中读取一次，之后不再更新
 */
extern RosParams ros_params;

}  // namespace fluorite::chassis