/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-09-24
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "remote_modbus/implementation.hpp"

#include <functional>

#include "remote_modbus/config.hpp"
#include "remote_modbus/protocol.hpp"
#include "remote_modbus/srv/configured_holding_register_read.hpp"
#include "remote_modbus/srv/configured_holding_register_write.hpp"
#include "yaml-cpp/yaml.h"

namespace remote_modbus {

Implementation::Implementation(rclcpp::Node *node) : Interface(node) {
  auto prefix = get_prefix_();

#ifndef MODBUS_STATS_DISABLE
  status_leafs_seen = node->create_publisher<std_msgs::msg::UInt8>(
      prefix + "/status/leafs_seen", 10);
#endif  // MODBUS_STATS_DISABLE
  status_last_leaf = node->create_publisher<std_msgs::msg::UInt8>(
      prefix + "/status/last_leaf", 10);
  status_last_seen = node->create_publisher<std_msgs::msg::UInt64>(
      prefix + "/status/last_seen", 10);

  status_last_error_leaf = node->create_publisher<std_msgs::msg::UInt8>(
      prefix + "/status/last_error_leaf", 10);
  status_last_exception_code = node->create_publisher<std_msgs::msg::UInt8>(
      prefix + "/status/last_exception_code", 10);
  status_last_error_seen = node->create_publisher<std_msgs::msg::UInt64>(
      prefix + "/status/last_error_seen", 10);
}

void Implementation::init_modbus_() {
  auto prefix = get_prefix_();

  srv_holding_register_read = node_->create_service<srv::HoldingRegisterRead>(
      prefix + MODBUS_SERVICE_HOLDING_REGISTER_READ,
      std::bind(&Interface::holding_register_read, this, std::placeholders::_1,
                std::placeholders::_2),
      ::rmw_qos_profile_default, callback_group_);
  srv_holding_register_write = node_->create_service<srv::HoldingRegisterWrite>(
      prefix + MODBUS_SERVICE_HOLDING_REGISTER_WRITE,
      std::bind(&Interface::holding_register_write, this, std::placeholders::_1,
                std::placeholders::_2),
      ::rmw_qos_profile_default, callback_group_);
  srv_holding_register_write_multiple =
      node_->create_service<srv::HoldingRegisterWriteMultiple>(
          prefix + MODBUS_SERVICE_HOLDING_REGISTER_WRITE_MULTIPLE,
          std::bind(&Interface::holding_register_write_multiple, this,
                    std::placeholders::_1, std::placeholders::_2),
          ::rmw_qos_profile_default, callback_group_);

  srv_get_com_event_log = node_->create_service<srv::GetComEventLog>(
      prefix + MODBUS_SERVICE_GET_COM_VENT_LOG,
      std::bind(&Interface::get_com_event_log, this, std::placeholders::_1,
                std::placeholders::_2),
      ::rmw_qos_profile_default, callback_group_);
  srv_read_device_id = node_->create_service<srv::ReadDeviceId>(
      prefix + MODBUS_SERVICE_READ_DEBICE_ID,
      std::bind(&Interface::read_device_id, this, std::placeholders::_1,
                std::placeholders::_2),
      ::rmw_qos_profile_default, callback_group_);

  srv_coil_read = node_->create_service<srv::CoilRead>(
      prefix + MODBUS_SERVICE_COIL_READ,
      std::bind(&Interface::coil_read, this, std::placeholders::_1,
                std::placeholders::_2),
      ::rmw_qos_profile_default, callback_group_);

  srv_coil_write = node_->create_service<srv::CoilWrite>(
      prefix + MODBUS_SERVICE_COIL_WRITE,
      std::bind(&Interface::coil_write, this, std::placeholders::_1,
                std::placeholders::_2),
      ::rmw_qos_profile_default, callback_group_);

  srv_coil_continuous_write = node_->create_service<srv::CoilContinuousWrite>(
      prefix + MODBUS_SERVICE_COIL_CONTINUOUS_WRITE,
      std::bind(&Interface::coil_continuous_write, this, std::placeholders::_1,
                std::placeholders::_2),
      ::rmw_qos_profile_default, callback_group_);
}

Implementation::LeafInterface::LeafInterface(
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

void Implementation::leafs_seen_bitmap_update_(uint8_t leaf_id) {
#ifndef MODBUS_STATS_DISABLE
  uint16_t index = leaf_id >> 3;
  uint16_t offset = leaf_id & 7;

  leafs_seen_bitmap_mutex_.lock();
  bool seen = leafs_seen_bitmap_[index] & (1 << offset);
  if (!seen) {
    leafs_seen_bitmap_[index] |= (1 << offset);
    leafs_seen_++;

    // FIXME(clairbee): rclcpp::exceptions::RCLError: could not create
    // publisher: rcl node's context is invalid
    //
    // leafs.emplace(leaf_id, std::make_shared<LeafInterface>(
    //                            node_, get_prefix_(),
    //                            leaf_id));

    std_msgs::msg::UInt8 _;
    _.data = leafs_seen_;
    status_leafs_seen->publish(_);
  }
  leafs_seen_bitmap_mutex_.unlock();
#endif  // MODBUS_STATS_DISABLE
}

void Implementation::update_on_response_(uint8_t leaf_id, uint8_t fc,
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
    if (leafs.find(leaf_id) != leafs.end()) {
      leafs[leaf_id]->last_seen->publish(msg_now);
      leafs[leaf_id]->last_function_code->publish(msg_fc);
    }
    leafs_seen_bitmap_mutex_.unlock();

    if (exception_code) {
      RCLCPP_DEBUG(node_->get_logger(), "Update stats on exception");
      std_msgs::msg::UInt8 msg_exception_code;
      msg_exception_code.data = exception_code;

      status_last_error_leaf->publish(msg_leaf_id);
      status_last_exception_code->publish(msg_exception_code);
      status_last_error_seen->publish(msg_now);

      leafs_seen_bitmap_mutex_.lock();
      if (leafs.find(leaf_id) != leafs.end()) {
        leafs[leaf_id]->last_exception_code->publish(msg_exception_code);
        leafs[leaf_id]->last_error_seen->publish(msg_now);
      }
      leafs_seen_bitmap_mutex_.unlock();
    }
  } else {
    RCLCPP_DEBUG(node_->get_logger(), "Update stats on request failure");
    status_last_error_leaf->publish(msg_leaf_id);
    status_last_exception_code->publish(msg_zero);
    status_last_error_seen->publish(msg_now);

    leafs_seen_bitmap_mutex_.lock();
    if (leafs.find(leaf_id) != leafs.end()) {
      leafs[leaf_id]->last_exception_code->publish(msg_zero);
      leafs[leaf_id]->last_error_seen->publish(msg_now);
    }
    leafs_seen_bitmap_mutex_.unlock();
  }
}

}  // namespace remote_modbus
