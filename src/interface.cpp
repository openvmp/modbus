/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-09-25
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "ros2_modbus/interface.hpp"

#include <functional>

#include "ros2_modbus/config.hpp"
#include "ros2_modbus/protocol.hpp"
#include "ros2_modbus/srv/configured_holding_register_read.hpp"
#include "ros2_modbus/srv/configured_holding_register_write.hpp"
#include "yaml-cpp/yaml.h"

namespace ros2_modbus {

Interface::Interface(rclcpp::Node *node) : node_{node} {
  callback_group_ =
      node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  node->declare_parameter("modbus_prefix",
                          "/modbus/" + std::string(node_->get_name()));
  node->get_parameter("modbus_prefix", interface_prefix_);
  node->declare_parameter("modbus_leaf_id", 1);
  node->get_parameter("modbus_leaf_id", leaf_id_);
}

std::string Interface::get_prefix_() {
  std::string prefix = std::string(node_->get_namespace());
  if (prefix.length() > 0 && prefix[prefix.length() - 1] == '/') {
    prefix = prefix.substr(0, prefix.length() - 1);
  }
  prefix += interface_prefix_.as_string();
  return prefix;
}

void Interface::generate_modbus_mappings(const std::string &prefix,
                                         const std::string &config_filename) {
  RCLCPP_DEBUG(node_->get_logger(),
               "modbus::Interface::generate_modbus_mappings(%s, %s): started",
               prefix.c_str(), config_filename.c_str());

  YAML::Node config = YAML::LoadFile(config_filename);

  if (!config || config.IsNull() || !config.IsMap()) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to parse: %s",
                 config_filename.c_str());
    return;
  }

  auto holding_registers = config[MODBUS_CONFIG_CHAPTER_HOLDING_REGISTERS];
  for (auto it : holding_registers) {
    RCLCPP_DEBUG(node_->get_logger(),
                 "Found a holding register entry in the config");
    auto hr = it;
    // auto hr = it.second;
    if (!hr["name"] || !hr["addr"]) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Incorrect syntax of the configured holding register");
      continue;
    }
    auto reg = std::make_shared<ConfiguredHoldingRegister>();
    reg->name = hr["name"].as<std::string>();
    RCLCPP_DEBUG(node_->get_logger(),
                 "Found a holding register entry in the config: %s",
                 reg->name.c_str());

    // TODO(clairbee): handle 'alt_names', 'comment' and 'min_delay_ms'
    reg->addr = hr["addr"].as<uint16_t>();
    if (hr["value"]) {
      auto value = hr["value"];

      auto min_value = value["min"];
      if (min_value) {
        reg->value.min_set = true;
        reg->value.min = min_value.as<uint16_t>();
      }

      auto max_value = value["max"];
      if (max_value) {
        reg->value.max_set = true;
        reg->value.max = max_value.as<uint16_t>();
      }
    }

    auto service_get =
        node_->create_service<srv::ConfiguredHoldingRegisterRead>(
            prefix + "/modbus/get_" + reg->name,
            std::bind(&Interface::configured_holding_register_read_handler_,
                      this, reg, std::placeholders::_1, std::placeholders::_2),
            ::rmw_qos_profile_default, callback_group_);
    configured_get.insert(
        {reg->name,
         std::pair<
             std::shared_ptr<ConfiguredHoldingRegister>,
             rclcpp::Service<srv::ConfiguredHoldingRegisterRead>::SharedPtr>(
             {reg, service_get})});

    if (!hr["read_only"]) {
      auto service_set =
          node_->create_service<srv::ConfiguredHoldingRegisterWrite>(
              prefix + "/modbus/set_" + reg->name,
              std::bind(&Interface::configured_holding_register_write_handler_,
                        this, reg, std::placeholders::_1,
                        std::placeholders::_2),
              ::rmw_qos_profile_default, callback_group_);
      configured_set.insert(
          {reg->name,
           std::pair<
               std::shared_ptr<ConfiguredHoldingRegister>,
               rclcpp::Service<srv::ConfiguredHoldingRegisterWrite>::SharedPtr>(
               {reg, service_set})});
    }
  }

  // TODO(clairbee): generate services for other types of registers

  RCLCPP_DEBUG(node_->get_logger(),
               "modbus::Interface::generate_modbus_mappings(%s, %s): ended",
               prefix.c_str(), config_filename.c_str());
}

void Interface::configured_holding_register_read_handler_(
    std::shared_ptr<ConfiguredHoldingRegister> reg,
    const std::shared_ptr<srv::ConfiguredHoldingRegisterRead::Request> request,
    std::shared_ptr<srv::ConfiguredHoldingRegisterRead::Response> response) {
  (void)request;  // no fields in the request
  configured_holding_register_read(reg, response->exception_code,
                                   response->value);
}

void Interface::configured_holding_register_read(
    std::shared_ptr<ConfiguredHoldingRegister> reg, uint8_t &exception_code,
    uint16_t &value) {
  auto req = std::make_shared<srv::HoldingRegisterRead::Request>();
  auto resp = std::make_shared<srv::HoldingRegisterRead::Response>();

  uint16_t leaf_id = leaf_id_.as_int();
  RCLCPP_DEBUG(node_->get_logger(),
               "Reading a configured holding register '%s' for leaf id: %d",
               reg->name.c_str(), leaf_id);
  req->leaf_id = leaf_id;
  req->addr = reg->addr;
  req->count = 1;

  holding_register_read(req, resp);
  RCLCPP_DEBUG(
      node_->get_logger(),
      "Interface::configured_holding_register_read(): response received");

  exception_code = resp->exception_code;
  if (resp->values.size() > 0) {
    value = resp->values[0];

    if (reg->value.min_set) {
      if (reg->value.min > value) {
        RCLCPP_ERROR(node_->get_logger(),
                     "Leaf: %d, Holding Register: %04X, Min: %04X, Value: %04X "
                     "(< min!)",
                     leaf_id, reg->addr, reg->value.min, value);
      }
    }

    if (reg->value.max_set) {
      if (reg->value.max < value) {
        RCLCPP_ERROR(node_->get_logger(),
                     "Leaf: %d, Holding Register: %04X, Max: %04X, Value: %04X "
                     "(> max!)",
                     leaf_id, reg->addr, reg->value.max, value);
      }
    }
  }
}

void Interface::configured_holding_register_write_handler_(
    std::shared_ptr<ConfiguredHoldingRegister> reg,
    const std::shared_ptr<srv::ConfiguredHoldingRegisterWrite::Request> request,
    std::shared_ptr<srv::ConfiguredHoldingRegisterWrite::Response> response) {
  configured_holding_register_write(reg, request->value,
                                    response->exception_code, response->value);
}

void Interface::configured_holding_register_write(
    std::shared_ptr<ConfiguredHoldingRegister> reg, uint16_t value,
    uint8_t &exception_code, uint16_t &response_value) {
  auto req = std::make_shared<srv::HoldingRegisterWrite::Request>();
  auto resp = std::make_shared<srv::HoldingRegisterWrite::Response>();

  uint16_t leaf_id = leaf_id_.as_int();
  RCLCPP_DEBUG(
      node_->get_logger(),
      "Writing '%u' to a configured holding register '%s' for leaf id: %d",
      (unsigned int)value, reg->name.c_str(), leaf_id);
  req->leaf_id = leaf_id;
  req->addr = reg->addr;
  req->value = value;

  holding_register_write(req, resp);
  RCLCPP_DEBUG(
      node_->get_logger(),
      "Interface::configured_holding_register_read(): response received");

  exception_code = resp->exception_code;
  response_value = resp->value;
}

}  // namespace ros2_modbus
