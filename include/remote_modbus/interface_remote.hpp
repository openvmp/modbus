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
#include "remote_modbus/interface.hpp"

namespace remote_modbus {

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
  virtual void coil_read(
      const std::shared_ptr<srv::CoilRead::Request> request,
      std::shared_ptr<srv::CoilRead::Response> response) override;
  virtual void coil_write(
      const std::shared_ptr<srv::CoilWrite::Request> request,
      std::shared_ptr<srv::CoilWrite::Response> response) override;
  virtual void coil_continuous_write(
      const std::shared_ptr<srv::CoilContinuousWrite::Request> request,
      std::shared_ptr<srv::CoilContinuousWrite::Response> response) override;

 private:
  rclcpp::Client<srv::HoldingRegisterRead>::SharedPtr
      clnt_holding_register_read;
  rclcpp::Client<srv::HoldingRegisterWrite>::SharedPtr
      clnt_holding_register_write;
  rclcpp::Client<srv::HoldingRegisterWriteMultiple>::SharedPtr
      clnt_holding_register_write_multiple;
  rclcpp::Client<srv::CoilRead>::SharedPtr clnt_coil_read;
  rclcpp::Client<srv::CoilWrite>::SharedPtr clnt_coil_write;
  rclcpp::Client<srv::CoilContinuousWrite>::SharedPtr
      clnt_coil_continuous_write;
  rclcpp::Client<srv::GetComEventLog>::SharedPtr clnt_get_com_event_log;
  rclcpp::Client<srv::ReadDeviceId>::SharedPtr clnt_read_device_id;
};

}  // namespace remote_modbus

#endif  // OPENVMP_MODBUS_INTERFACE_REMOTE_H
