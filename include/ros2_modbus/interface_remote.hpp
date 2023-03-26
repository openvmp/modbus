/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-09-25
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_MODBUS_INTERFACE_REMOTE_H
#define OPENVMP_MODBUS_INTERFACE_REMOTE_H

#include <future>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "ros2_modbus/interface.hpp"

namespace ros2_modbus {

class RemoteInterface final : public Interface {
 public:
  RemoteInterface(rclcpp::Node *node);
  virtual ~RemoteInterface() {}

  virtual void holding_register_read(
      const std::shared_ptr<srv::HoldingRegisterRead::Request> request,
      std::shared_ptr<srv::HoldingRegisterRead::Response> response) override;
  virtual void holding_register_write(
      const std::shared_ptr<srv::HoldingRegisterWrite::Request> request,
      std::shared_ptr<srv::HoldingRegisterWrite::Response> response) override;
  virtual void holding_register_write_multiple(
      const std::shared_ptr<srv::HoldingRegisterWriteMultiple::Request>,
      std::shared_ptr<srv::HoldingRegisterWriteMultiple::Response> response)
      override;
  virtual void get_com_event_log(
      const std::shared_ptr<srv::GetComEventLog::Request> request,
      std::shared_ptr<srv::GetComEventLog::Response> response) override;
  virtual void read_device_id(
      const std::shared_ptr<srv::ReadDeviceId::Request> request,
      std::shared_ptr<srv::ReadDeviceId::Response> response) override;

 private:
  rclcpp::Client<srv::HoldingRegisterRead>::SharedPtr
      clnt_holding_register_read;
  rclcpp::Client<srv::HoldingRegisterWrite>::SharedPtr
      clnt_holding_register_write;
  rclcpp::Client<srv::HoldingRegisterWriteMultiple>::SharedPtr
      clnt_holding_register_write_multiple;
  rclcpp::Client<srv::GetComEventLog>::SharedPtr clnt_get_com_event_log;
  rclcpp::Client<srv::ReadDeviceId>::SharedPtr clnt_read_device_id;
};

}  // namespace ros2_modbus

#endif  // OPENVMP_MODBUS_INTERFACE_REMOTE_H
