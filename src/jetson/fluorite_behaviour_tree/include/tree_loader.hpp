/**
 * @file    tree_loader.hpp
 * @brief   遍历xml路径下的所有xml文件，加载行为树
 */

#ifndef TREE_LOADER_HPP
#define TREE_LOADER_HPP

#include <filesystem>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "behaviortree_cpp_v3/bt_factory.h"

#include "bt_node_register.hpp"

class TreeLoader {
 public:
  TreeLoader() = delete;
  explicit TreeLoader(BT::BehaviorTreeFactory& factory) : factory_(factory) {
    for (const auto& entry : std::filesystem::directory_iterator(kTreePath)) {
      if (entry.path().extension() == ".xml") {
        xml_files_.push_back(entry.path().stem().string());
      }
    }
  }

  auto Load(const std::string& name) {
    if (std::find(xml_files_.begin(), xml_files_.end(), name) != xml_files_.end()) {
      return factory_.createTreeFromFile(kTreePath + name + ".xml");
    } else {
      throw std::runtime_error("Tree not found: " + name);
    }
  }

 private:
  const std::string kTreePath = ament_index_cpp::get_package_share_directory("fluorite_behaviour_tree") + "/xml/trees/";
  BT::BehaviorTreeFactory& factory_;
  std::vector<std::string> xml_files_;
};

#endif