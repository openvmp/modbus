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

#include "modbus/interface.hpp"
#include "rclcpp/rclcpp.hpp"

namespace modbus {

class RemoteInterface final : public Interface {
 public:
  RemoteInterface(rclcpp::Node *node);
  virtual ~RemoteInterface() {}

  virtual void holding_register_read(
      const std::shared_ptr<modbus::srv::HoldingRegisterRead::Request> request,
      std::shared_ptr<modbus::srv::HoldingRegisterRead::Response> response)
      override;
  virtual void holding_register_write(
      const std::shared_ptr<modbus::srv::HoldingRegisterWrite::Request> request,
      std::shared_ptr<modbus::srv::HoldingRegisterWrite::Response> response)
      override;
  virtual void holding_register_write_multiple(
      const std::shared_ptr<modbus::srv::HoldingRegisterWriteMultiple::Request>,
      std::shared_ptr<modbus::srv::HoldingRegisterWriteMultiple::Response>
          response) override;
  virtual void get_com_event_log(
      const std::shared_ptr<modbus::srv::GetComEventLog::Request> request,
      std::shared_ptr<modbus::srv::GetComEventLog::Response> response) override;
  virtual void read_device_id(
      const std::shared_ptr<modbus::srv::ReadDeviceId::Request> request,
      std::shared_ptr<modbus::srv::ReadDeviceId::Response> response) override;

 private:
  rclcpp::Client<modbus::srv::HoldingRegisterRead>::SharedPtr
      clnt_holding_register_read;
  rclcpp::Client<modbus::srv::HoldingRegisterWrite>::SharedPtr
      clnt_holding_register_write;
  rclcpp::Client<modbus::srv::HoldingRegisterWriteMultiple>::SharedPtr
      clnt_holding_register_write_multiple;
  rclcpp::Client<modbus::srv::GetComEventLog>::SharedPtr clnt_get_com_event_log;
  rclcpp::Client<modbus::srv::ReadDeviceId>::SharedPtr clnt_read_device_id;
};

}  // namespace modbus

#endif  // OPENVMP_MODBUS_INTERFACE_REMOTE_H
