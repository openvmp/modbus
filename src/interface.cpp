/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-09-24
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "modbus/interface.hpp"

#include <functional>

#include "modbus/config.hpp"
#include "modbus/protocol.hpp"
#include "modbus/srv/configured_holding_register_read.hpp"
#include "modbus/srv/configured_holding_register_write.hpp"
#include "yaml-cpp/yaml.h"

namespace modbus {

ModbusInterface::ModbusInterface(rclcpp::Node *node) : node_{node} {
  node->declare_parameter("modbus_prefix", "/modbus/default");
  node->get_parameter("modbus_prefix", interface_prefix_);

#ifndef MODBUS_STATS_DISABLE
  status_leafs_seen = node->create_publisher<std_msgs::msg::UInt8>(
      interface_prefix_.as_string() + "/status/leafs_seen", 10);
#endif  // MODBUS_STATS_DISABLE
  status_last_leaf = node->create_publisher<std_msgs::msg::UInt8>(
      interface_prefix_.as_string() + "/status/last_leaf", 10);
  status_last_seen = node->create_publisher<std_msgs::msg::UInt64>(
      interface_prefix_.as_string() + "/status/last_seen", 10);

  status_last_error_leaf = node->create_publisher<std_msgs::msg::UInt8>(
      interface_prefix_.as_string() + "/status/last_error_leaf", 10);
  status_last_exception_code = node->create_publisher<std_msgs::msg::UInt8>(
      interface_prefix_.as_string() + "/status/last_exception_code", 10);
  status_last_error_seen = node->create_publisher<std_msgs::msg::UInt64>(
      interface_prefix_.as_string() + "/status/last_error_seen", 10);

  holding_register_read =
      node_->create_service<modbus::srv::HoldingRegisterRead>(
          interface_prefix_.as_string() + "/holding_register_read",
          std::bind(&ModbusInterface::holding_register_read_handler_, this,
                    std::placeholders::_1, std::placeholders::_2));
  holding_register_write =
      node_->create_service<modbus::srv::HoldingRegisterWrite>(
          interface_prefix_.as_string() + "/holding_register_write",
          std::bind(&ModbusInterface::holding_register_write_handler_, this,
                    std::placeholders::_1, std::placeholders::_2));
  holding_register_write_multiple =
      node_->create_service<modbus::srv::HoldingRegisterWriteMultiple>(
          interface_prefix_.as_string() + "/holding_register_write_multiple",
          std::bind(&ModbusInterface::holding_register_write_multiple_handler_,
                    this, std::placeholders::_1, std::placeholders::_2));

  get_com_event_log = node_->create_service<modbus::srv::GetComEventLog>(
      interface_prefix_.as_string() + "/get_com_event_log",
      std::bind(&ModbusInterface::get_com_event_log_handler_, this,
                std::placeholders::_1, std::placeholders::_2));
}

void ModbusInterface::generate_modbus_mappings(
    uint8_t leaf_id, const std::string &prefix,
    const std::string &config_filename) {
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
        node_->create_service<modbus::srv::ConfiguredHoldingRegisterRead>(
            prefix + "/modbus/get_" + reg->name,
            std::bind(
                &ModbusInterface::configured_holding_register_read_handler_,
                this, leaf_id, reg, std::placeholders::_1,
                std::placeholders::_2));
    configured_get.insert(
        {reg->name,
         std::pair<std::shared_ptr<ConfiguredHoldingRegister>,
                   rclcpp::Service<
                       modbus::srv::ConfiguredHoldingRegisterRead>::SharedPtr>(
             {reg, service_get})});
    // TODO(clairbee): prepare setters
  }

  // TODO(clairbee): parse other types of registers
}

ModbusInterface::LeafInterface::LeafInterface(
    rclcpp::Node *node, const std::string &interface_prefix, uint8_t leaf_id) {
  last_seen = node->create_publisher<std_msgs::msg::UInt64>(
      interface_prefix + "/id" + std::to_string(leaf_id) + "/last_seen", 10);
  last_function_code = node->create_publisher<std_msgs::msg::UInt8>(
      interface_prefix + "/id" + std::to_string(leaf_id) +
          "/last_function_code",
      10);
  last_exception_code = node->create_publisher<std_msgs::msg::UInt8>(
      interface_prefix + "/id" + std::to_string(leaf_id) +
          "/last_exception_code",
      10);
  last_error_seen = node->create_publisher<std_msgs::msg::UInt64>(
      interface_prefix + "/id" + std::to_string(leaf_id) + "/last_error_seen",
      10);
}

void ModbusInterface::leafs_seen_bitmap_update_(uint8_t leaf_id) {
#ifndef MODBUS_STATS_DISABLE
  uint16_t index = leaf_id >> 3;
  uint16_t offset = leaf_id & 7;

  leafs_seen_bitmap_mutex_.lock();
  bool seen = leafs_seen_bitmap_[index] & (1 << offset);
  if (!seen) {
    leafs_seen_bitmap_[index] |= (1 << offset);
    leafs_seen_++;

    leafs.emplace(leaf_id, std::shared_ptr<LeafInterface>(new LeafInterface(
                               node_, interface_prefix_.as_string(), leaf_id)));

    std_msgs::msg::UInt8 _;
    _.data = leafs_seen_;
    status_leafs_seen->publish(_);
  }
  leafs_seen_bitmap_mutex_.unlock();
#endif  // MODBUS_STATS_DISABLE
}

void ModbusInterface::update_on_response_(uint8_t leaf_id, uint8_t fc,
                                          uint8_t exception_code,
                                          rclcpp::FutureReturnCode ret) {
  std_msgs::msg::UInt8 msg_leaf_id;
  std_msgs::msg::UInt64 msg_now;
  std_msgs::msg::UInt8 msg_zero;
  std_msgs::msg::UInt8 msg_fc;

  msg_leaf_id.data = leaf_id;
  msg_fc.data = fc;

  auto now = rclcpp::Time();
  msg_now.data = now.nanoseconds();

  leafs_seen_bitmap_update_(leaf_id);
  if (ret == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_DEBUG(node_->get_logger(), "Update stats on request completion");
    // after this we can be sure that it's present in this->leafs
    status_last_leaf->publish(msg_leaf_id);
    status_last_seen->publish(msg_now);

    leafs_seen_bitmap_mutex_.lock();
    leafs[leaf_id]->last_seen->publish(msg_now);
    leafs[leaf_id]->last_function_code->publish(msg_fc);
    leafs_seen_bitmap_mutex_.unlock();

    if (exception_code) {
      RCLCPP_DEBUG(node_->get_logger(), "Update stats on exception");
      std_msgs::msg::UInt8 msg_exception_code;
      msg_exception_code.data = exception_code;

      status_last_error_leaf->publish(msg_leaf_id);
      status_last_exception_code->publish(msg_exception_code);
      status_last_error_seen->publish(msg_now);

      leafs_seen_bitmap_mutex_.lock();
      leafs[leaf_id]->last_exception_code->publish(msg_exception_code);
      leafs[leaf_id]->last_error_seen->publish(msg_now);
      leafs_seen_bitmap_mutex_.unlock();
    }
  } else {
    RCLCPP_DEBUG(node_->get_logger(), "Update stats on request failure");
    status_last_error_leaf->publish(msg_leaf_id);
    status_last_exception_code->publish(msg_zero);
    status_last_error_seen->publish(msg_now);

    leafs_seen_bitmap_mutex_.lock();
    leafs[leaf_id]->last_exception_code->publish(msg_zero);
    leafs[leaf_id]->last_error_seen->publish(msg_now);
    leafs_seen_bitmap_mutex_.unlock();
  }
}

}  // namespace modbus
