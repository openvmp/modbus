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

void ModbusInterface::configured_holding_register_read_handler_(
    uint8_t leaf_id, std::shared_ptr<ConfiguredHoldingRegister> reg,
    const std::shared_ptr<modbus::srv::ConfiguredHoldingRegisterRead::Request>
        request,
    std::shared_ptr<modbus::srv::ConfiguredHoldingRegisterRead::Response>
        response) {
  (void)request;  // no fields in the request
  configured_holding_register_read(leaf_id, reg, response->exception_code,
                                   response->value);
}

void ModbusInterface::configured_holding_register_read(
    uint8_t leaf_id, std::shared_ptr<ConfiguredHoldingRegister> reg,
    uint8_t &exception_code, uint16_t &value) {
  auto req = std::make_shared<modbus::srv::HoldingRegisterRead::Request>();
  auto resp = std::make_shared<modbus::srv::HoldingRegisterRead::Response>();

  RCLCPP_DEBUG(node_->get_logger(),
               "Reading a configured holding register '%s' for leaf id: %d",
               reg->name.c_str(), leaf_id);
  req->leaf_id = leaf_id;
  req->addr = reg->addr;
  req->count = 1;
  auto ret = holding_register_read_handler_real_(req, resp);
  update_on_response_(req->leaf_id, MODBUS_FC_READ_HOLDING_REGISTERS,
                      resp->exception_code, ret);

  exception_code = resp->exception_code;
  if (resp->values.size() > 0) {
    value = resp->values[0];

    if (reg->value.min_set) {
      if (reg->value.min > value) {
        RCLCPP_ERROR(node_->get_logger(),
                     "Leaf: %d, Holding Register: %04X, Min: %04X, Value: %04X "
                     "(< min!)",
                     leaf_id, reg->addr, reg->value.min, value);
      }
    }

    if (reg->value.max_set) {
      if (reg->value.max < value) {
        RCLCPP_ERROR(node_->get_logger(),
                     "Leaf: %d, Holding Register: %04X, Max: %04X, Value: %04X "
                     "(> max!)",
                     leaf_id, reg->addr, reg->value.max, value);
      }
    }
  }
}

void ModbusInterface::holding_register_read_handler_(
    const std::shared_ptr<modbus::srv::HoldingRegisterRead::Request> request,
    std::shared_ptr<modbus::srv::HoldingRegisterRead::Response> response) {
  auto ret = holding_register_read_handler_real_(request, response);

  RCLCPP_DEBUG(node_->get_logger(), "Received HoldingRegisterRead response");

  update_on_response_(request->leaf_id, MODBUS_FC_READ_HOLDING_REGISTERS,
                      response->exception_code, ret);
}

void ModbusInterface::holding_register_write_handler_(
    const std::shared_ptr<modbus::srv::HoldingRegisterWrite::Request> request,
    std::shared_ptr<modbus::srv::HoldingRegisterWrite::Response> response) {
  auto ret = holding_register_write_handler_real_(request, response);

  update_on_response_(request->leaf_id, MODBUS_FC_PRESET_SINGLE_REGISTER,
                      response->exception_code, ret);
}
void ModbusInterface::holding_register_write_multiple_handler_(
    const std::shared_ptr<modbus::srv::HoldingRegisterWriteMultiple::Request>
        request,
    std::shared_ptr<modbus::srv::HoldingRegisterWriteMultiple::Response>
        response)  // TODO
{
  auto ret = holding_register_write_multiple_handler_real_(request, response);

  update_on_response_(request->leaf_id, MODBUS_FC_PRESET_MULTIPLE_REGISTERS,
                      response->exception_code, ret);
}

}  // namespace modbus