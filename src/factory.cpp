/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-09-25
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "modbus/factory.hpp"

#include <exception>

#include "modbus/implementation.hpp"
#include "modbus/interface_remote.hpp"

namespace modbus {

std::shared_ptr<Interface> Factory::New(rclcpp::Node *node) {
  rclcpp::Parameter is_remote;
  node->declare_parameter("modbus_is_remote", true);
  node->get_parameter("modbus_is_remote", is_remote);

  if (is_remote.as_bool()) {
    return std::make_shared<RemoteInterface>(node);
  } else {
    throw std::invalid_argument(
        "Link with Modbus RTU/TCP/ASCII or set modbus_is_remote");
  }
}

}  // namespace modbus