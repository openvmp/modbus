/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-09-25
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "ros2_modbus/factory.hpp"

#include <exception>

#include "ros2_modbus/interface_remote.hpp"

namespace ros2_modbus {

std::shared_ptr<Interface> Factory::New(rclcpp::Node *node) {
  rclcpp::Parameter use_remote;
  if (!node->has_parameter("use_remote")) {
    node->declare_parameter("use_remote", true);
  }
  node->get_parameter("use_remote", use_remote);

  rclcpp::Parameter is_remote;
  node->declare_parameter("modbus_is_remote", use_remote.as_bool());
  node->get_parameter("modbus_is_remote", is_remote);

  if (is_remote.as_bool()) {
    return std::make_shared<RemoteInterface>(node);
  } else {
    throw std::invalid_argument(
        "Link with Modbus RTU/TCP/ASCII or set modbus_is_remote");
  }
}

}  // namespace ros2_modbus
