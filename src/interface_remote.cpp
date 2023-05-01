/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-09-25
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "remote_modbus/interface_remote.hpp"

#include <functional>

#include "remote_modbus/config.hpp"
#include "remote_modbus/protocol.hpp"
#include "yaml-cpp/yaml.h"

namespace remote_modbus {

RemoteInterface::RemoteInterface(rclcpp::Node *node) : Interface(node) {
  auto prefix = get_prefix_();

  RCLCPP_DEBUG(node_->get_logger(),
               "modbus::RemoteInterface::RemoteInterface(): Connecting to the "
               "remote interface: %s",
               prefix.c_str());

  clnt_holding_register_read = node->create_client<srv::HoldingRegisterRead>(
      prefix + MODBUS_SERVICE_HOLDING_REGISTER_READ, ::rmw_qos_profile_default,
      callback_group_);
  clnt_holding_register_write = node->create_client<srv::HoldingRegisterWrite>(
      prefix + MODBUS_SERVICE_HOLDING_REGISTER_WRITE, ::rmw_qos_profile_default,
      callback_group_);
  clnt_holding_register_write_multiple =
      node->create_client<srv::HoldingRegisterWriteMultiple>(
          prefix + MODBUS_SERVICE_HOLDING_REGISTER_WRITE_MULTIPLE,
          ::rmw_qos_profile_default, callback_group_);
  clnt_get_com_event_log = node->create_client<srv::GetComEventLog>(
      prefix + MODBUS_SERVICE_GET_COM_VENT_LOG, ::rmw_qos_profile_default,
      callback_group_);
  clnt_read_device_id = node->create_client<srv::ReadDeviceId>(
      prefix + MODBUS_SERVICE_READ_DEBICE_ID, ::rmw_qos_profile_default,
      callback_group_);

  clnt_holding_register_read->wait_for_service();
  clnt_holding_register_write->wait_for_service();
  clnt_holding_register_write_multiple->wait_for_service();
  clnt_get_com_event_log->wait_for_service();
  clnt_read_device_id->wait_for_service();

  RCLCPP_DEBUG(node_->get_logger(), "Connected to the remote interface: %s",
               prefix.c_str());
}

void RemoteInterface::holding_register_read(
    const std::shared_ptr<srv::HoldingRegisterRead::Request> request,
    std::shared_ptr<srv::HoldingRegisterRead::Response> response) {
  if (!request->leaf_id) {
    request->leaf_id = leaf_id_.as_int();
  }
  auto f = clnt_holding_register_read->async_send_request(request);
  f.wait();

  // if (rclcpp::spin_until_future_complete(node_->get_node_base_interface(), f)
  // !=
  //     rclcpp::FutureReturnCode::SUCCESS) {
  //   response->exception_code = 0;
  //   return;
  // }
  RCLCPP_DEBUG(node_->get_logger(),
               "RemoteInterface::holding_register_read(): response received");

  *response = *f.get();
}

void RemoteInterface::holding_register_write(
    const std::shared_ptr<srv::HoldingRegisterWrite::Request> request,
    std::shared_ptr<srv::HoldingRegisterWrite::Response> response) {
  if (!request->leaf_id) {
    request->leaf_id = leaf_id_.as_int();
  }
  auto f = clnt_holding_register_write->async_send_request(request);
  f.wait();
  *response = *f.get();
}

void RemoteInterface::holding_register_write_multiple(
    const std::shared_ptr<srv::HoldingRegisterWriteMultiple::Request> request,
    std::shared_ptr<srv::HoldingRegisterWriteMultiple::Response> response) {
  if (!request->leaf_id) {
    request->leaf_id = leaf_id_.as_int();
  }
  auto f = clnt_holding_register_write_multiple->async_send_request(request);
  f.wait();
  *response = *f.get();
}

void RemoteInterface::get_com_event_log(
    const std::shared_ptr<srv::GetComEventLog::Request> request,
    std::shared_ptr<srv::GetComEventLog::Response> response) {
  if (!request->leaf_id) {
    request->leaf_id = leaf_id_.as_int();
  }
  auto f = clnt_get_com_event_log->async_send_request(request);
  f.wait();
  *response = *f.get();
}

void RemoteInterface::read_device_id(
    const std::shared_ptr<srv::ReadDeviceId::Request> request,
    std::shared_ptr<srv::ReadDeviceId::Response> response) {
  if (!request->leaf_id) {
    request->leaf_id = leaf_id_.as_int();
  }
  auto f = clnt_read_device_id->async_send_request(request);
  f.wait();
  *response = *f.get();
}

}  // namespace remote_modbus
