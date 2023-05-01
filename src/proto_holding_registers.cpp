/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-09-24
 *
 * Licensed under Apache License, Version 2.0.
 */

#include <functional>

#include "remote_modbus/config.hpp"
#include "remote_modbus/implementation.hpp"
#include "remote_modbus/protocol.hpp"
#include "yaml-cpp/yaml.h"

namespace remote_modbus {

void Implementation::holding_register_read(
    const std::shared_ptr<srv::HoldingRegisterRead::Request> request,
    std::shared_ptr<srv::HoldingRegisterRead::Response> response) {
  if (!request->leaf_id) {
    request->leaf_id = leaf_id_.as_int();
  }
  auto ret = holding_register_read_handler_real_(request, response);

  RCLCPP_DEBUG(node_->get_logger(), "Received HoldingRegisterRead response");

  update_on_response_(request->leaf_id, MODBUS_FC_READ_HOLDING_REGISTERS,
                      response->exception_code, ret);
}

void Implementation::holding_register_write(
    const std::shared_ptr<srv::HoldingRegisterWrite::Request> request,
    std::shared_ptr<srv::HoldingRegisterWrite::Response> response) {
  if (!request->leaf_id) {
    request->leaf_id = leaf_id_.as_int();
  }
  auto ret = holding_register_write_handler_real_(request, response);

  RCLCPP_DEBUG(node_->get_logger(), "Received HoldingRegisterWrite response");

  update_on_response_(request->leaf_id, MODBUS_FC_PRESET_SINGLE_REGISTER,
                      response->exception_code, ret);
}

void Implementation::holding_register_write_multiple(
    const std::shared_ptr<srv::HoldingRegisterWriteMultiple::Request> request,
    std::shared_ptr<srv::HoldingRegisterWriteMultiple::Response>
        response)  // TODO
{
  if (!request->leaf_id) {
    request->leaf_id = leaf_id_.as_int();
  }
  auto ret = holding_register_write_multiple_handler_real_(request, response);

  RCLCPP_DEBUG(node_->get_logger(),
               "Received HoldingRegisterWriteMultiple response");

  update_on_response_(request->leaf_id, MODBUS_FC_PRESET_MULTIPLE_REGISTERS,
                      response->exception_code, ret);
}

}  // namespace remote_modbus
