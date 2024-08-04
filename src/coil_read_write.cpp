/*
 * OpenVMP, 2024
 *
 * Author: Ahmed Nazar
 * Created: 2024-5-18
 *
 * Licensed under Apache License, Version 2.0.
 */

#include <functional>

#include "remote_modbus/config.hpp"
#include "remote_modbus/implementation.hpp"
#include "remote_modbus/protocol.hpp"

namespace remote_modbus {

void Implementation::coil_read(
    const std::shared_ptr<srv::CoilRead::Request> request,
    std::shared_ptr<srv::CoilRead::Response> response) {
  if (!request->leaf_id) {
    request->leaf_id = leaf_id_.as_int();
  }
  auto ret = coil_read_handler_real_(request, response);

  RCLCPP_DEBUG(node_->get_logger(), "Received coil_read response");

  update_on_response_(request->leaf_id, MODBUS_FC_READ_COIL,
                      response->exception_code, ret);
}

void Implementation::coil_write(
    const std::shared_ptr<srv::CoilWrite::Request> request,
    std::shared_ptr<srv::CoilWrite::Response> response) {
  if (!request->leaf_id) {
    request->leaf_id = leaf_id_.as_int();
  }
  auto ret = coil_write_handler_real_(request, response);

  RCLCPP_DEBUG(node_->get_logger(), "Received coil_write response");

  update_on_response_(request->leaf_id, MODBUS_FC_WRITE_COIL,
                      response->exception_code, ret);
}

void Implementation::coil_continuous_write(
    const std::shared_ptr<srv::CoilContinuousWrite::Request> request,
    std::shared_ptr<srv::CoilContinuousWrite::Response> response) {
  if (!request->leaf_id) {
    request->leaf_id = leaf_id_.as_int();
  }
  auto ret = coil_continuous_write_handler_real_(request, response);

  RCLCPP_DEBUG(node_->get_logger(), "Received coil_continuous_write response");

  update_on_response_(request->leaf_id, MODBUS_FC_WRITE_COIL_CONTINUOUS,
                      response->exception_code, ret);
}

}  // namespace remote_modbus
