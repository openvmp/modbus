/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-09-24
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "modbus/implementation.hpp"

#include <functional>

#include "modbus/config.hpp"
#include "modbus/protocol.hpp"
#include "modbus/srv/configured_holding_register_read.hpp"
#include "modbus/srv/configured_holding_register_write.hpp"
#include "yaml-cpp/yaml.h"

namespace modbus {

Implementation::Implementation(rclcpp::Node *node) : Interface(node) {
  auto prefix = get_prefix_();

  rmw_qos_profile_t rmw = {
      .history = rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST,
      .depth = 1,
      .reliability =
          rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
      .durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
      .deadline = {0, 50000000},
      .lifespan = {0, 50000000},
      .liveliness = RMW_QOS_POLICY_LIVELINESS_AUTOMATIC,
      .liveliness_lease_duration = {0, 0},
      .avoid_ros_namespace_conventions = false,
  };
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw), rmw);

#ifndef MODBUS_STATS_DISABLE
  status_leafs_seen = node->create_publisher<std_msgs::msg::UInt8>(
      prefix + "/status/leafs_seen", qos);
#endif  // MODBUS_STATS_DISABLE
  status_last_leaf = node->create_publisher<std_msgs::msg::UInt8>(
      prefix + "/status/last_leaf", qos);
  status_last_seen = node->create_publisher<std_msgs::msg::UInt64>(
      prefix + "/status/last_seen", qos);

  status_last_error_leaf = node->create_publisher<std_msgs::msg::UInt8>(
      prefix + "/status/last_error_leaf", qos);
  status_last_exception_code = node->create_publisher<std_msgs::msg::UInt8>(
      prefix + "/status/last_exception_code", qos);
  status_last_error_seen = node->create_publisher<std_msgs::msg::UInt64>(
      prefix + "/status/last_error_seen", qos);

  srv_holding_register_read =
      node_->create_service<modbus::srv::HoldingRegisterRead>(
          prefix + MODBUS_SERVICE_HOLDING_REGISTER_READ,
          std::bind(&Interface::holding_register_read, this,
                    std::placeholders::_1, std::placeholders::_2),
          ::rmw_qos_profile_default, callback_group_);
  srv_holding_register_write =
      node_->create_service<modbus::srv::HoldingRegisterWrite>(
          prefix + MODBUS_SERVICE_HOLDING_REGISTER_WRITE,
          std::bind(&Interface::holding_register_write, this,
                    std::placeholders::_1, std::placeholders::_2),
          ::rmw_qos_profile_default, callback_group_);
  srv_holding_register_write_multiple =
      node_->create_service<modbus::srv::HoldingRegisterWriteMultiple>(
          prefix + MODBUS_SERVICE_HOLDING_REGISTER_WRITE_MULTIPLE,
          std::bind(&Interface::holding_register_write_multiple, this,
                    std::placeholders::_1, std::placeholders::_2),
          ::rmw_qos_profile_default, callback_group_);

  srv_get_com_event_log = node_->create_service<modbus::srv::GetComEventLog>(
      prefix + MODBUS_SERVICE_GET_COM_VENT_LOG,
      std::bind(&Interface::get_com_event_log, this, std::placeholders::_1,
                std::placeholders::_2),
      ::rmw_qos_profile_default, callback_group_);
  srv_read_device_id = node_->create_service<modbus::srv::ReadDeviceId>(
      prefix + MODBUS_SERVICE_READ_DEBICE_ID,
      std::bind(&Interface::read_device_id, this, std::placeholders::_1,
                std::placeholders::_2),
      ::rmw_qos_profile_default, callback_group_);
}

Implementation::LeafInterface::LeafInterface(
    rclcpp::Node *node, const std::string &interface_prefix, uint8_t leaf_id) {
  rmw_qos_profile_t rmw = {
      .history = rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST,
      .depth = 1,
      .reliability =
          rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
      .durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
      .deadline = {0, 50000000},
      .lifespan = {0, 50000000},
      .liveliness = RMW_QOS_POLICY_LIVELINESS_AUTOMATIC,
      .liveliness_lease_duration = {0, 0},
      .avoid_ros_namespace_conventions = false,
  };
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw), rmw);

  last_seen = node->create_publisher<std_msgs::msg::UInt64>(
      interface_prefix + "/id" + std::to_string(leaf_id) + "/last_seen", qos);
  last_function_code = node->create_publisher<std_msgs::msg::UInt8>(
      interface_prefix + "/id" + std::to_string(leaf_id) +
          "/last_function_code",
      qos);
  last_exception_code = node->create_publisher<std_msgs::msg::UInt8>(
      interface_prefix + "/id" + std::to_string(leaf_id) +
          "/last_exception_code",
      qos);
  last_error_seen = node->create_publisher<std_msgs::msg::UInt64>(
      interface_prefix + "/id" + std::to_string(leaf_id) + "/last_error_seen",
      qos);
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

}  // namespace modbus
