/*
 * Copyright 2022 OpenVMP Authors
 *
 * Licensed under HIPPOCRATIC LICENSE Version 3.0.
 * Generated using
 * https://firstdonoharm.dev/version/3/0/bds-bod-cl-eco-ffd-media-mil-soc-sup-sv.md
 * See https://github.com/openvmp/openvmp/blob/main/docs/License.md for more
 * details.
 *
 */

#include <functional>

#include "modbus/config.hpp"
#include "modbus/interface.hpp"
#include "modbus/protocol.hpp"
#include "modbus/srv/configured_holding_register_read.hpp"
#include "modbus/srv/configured_holding_register_write.hpp"
#include "yaml-cpp/yaml.h"

namespace modbus {

void ModbusInterface::get_com_event_log_handler_(
    const std::shared_ptr<modbus::srv::GetComEventLog::Request> request,
    std::shared_ptr<modbus::srv::GetComEventLog::Response> response) {
  auto ret = get_com_event_log_handler_real_(request, response);

  RCLCPP_DEBUG(node_->get_logger(), "Received GetComEventLog response");

  update_on_response_(request->leaf_id, MODBUS_FC_GET_COM_EVENT_LOG,
                      response->exception_code, ret);
}

void ModbusInterface::read_device_id_handler_(
    const std::shared_ptr<modbus::srv::ReadDeviceId::Request> request,
    std::shared_ptr<modbus::srv::ReadDeviceId::Response> response) {
  auto ret = read_device_id_handler_real_(request, response);

  RCLCPP_DEBUG(node_->get_logger(), "Received ReadDeviceId response");

  update_on_response_(request->leaf_id, MODBUS_FC_READ_DEVICE_ID,
                      response->exception_code, ret);
}

}  // namespace modbus