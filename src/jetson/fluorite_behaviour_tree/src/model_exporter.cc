
/**
 * @file    model_exporter.cc
 * @brief   导出行为树模型到models.xml文件，供Groot加载使用
 * @date    2025-02-08
 */

#include <fstream>

#include "behaviortree_cpp_v3/xml_parsing.h"

#include "bt_node_register.hpp"

int main() {
  BT::BehaviorTreeFactory factory;
  bt_nodes::BtNodeRegisterArray::Register(factory);

  const auto xml_models = BT::writeTreeNodesModelXML(factory);
  try {
    std::ofstream file("bt_models.xml");
    file << xml_models;
    file.close();
    std::cout << "Models exported to models.xml" << std::endl;
  } catch (std::exception& e) {
    std::cerr << e.what() << std::endl;
    return 1;
  }
  return 0;
}