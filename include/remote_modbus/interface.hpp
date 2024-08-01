/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-09-25
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_MODBUS_INTERFACE_H
#define OPENVMP_MODBUS_INTERFACE_H

#include "rclcpp/callback_group.hpp"
#include "rclcpp/rclcpp.hpp"
#include "remote_modbus/srv/coil_continuous_write.hpp"
#include "remote_modbus/srv/coil_read.hpp"
#include "remote_modbus/srv/coil_write.hpp"
#include "remote_modbus/srv/configured_holding_register_read.hpp"
#include "remote_modbus/srv/configured_holding_register_write.hpp"
#include "remote_modbus/srv/get_com_event_log.hpp"
#include "remote_modbus/srv/holding_register_read.hpp"
#include "remote_modbus/srv/holding_register_write.hpp"
#include "remote_modbus/srv/holding_register_write_multiple.hpp"
#include "remote_modbus/srv/read_device_id.hpp"

#define MODBUS_SERVICE_COIL_READ "/coil_read"
#define MODBUS_SERVICE_COIL_WRITE "/coil_write"
#define MODBUS_SERVICE_COIL_CONTINUOUS_WRITE "/coil_continuous_write"
#define MODBUS_SERVICE_HOLDING_REGISTER_READ "/holding_register_read"
#define MODBUS_SERVICE_HOLDING_REGISTER_WRITE "/holding_register_write"
#define MODBUS_SERVICE_HOLDING_REGISTER_WRITE_MULTIPLE \
  "/holding_register_write_multiple"
#define MODBUS_SERVICE_GET_COM_VENT_LOG "/get_com_event_log"
#define MODBUS_SERVICE_READ_DEBICE_ID "/read_device_id"

namespace remote_modbus {

class Interface {
 public:
  Interface(rclcpp::Node *node);
  virtual ~Interface() {}

  void generate_modbus_mappings(const std::string &prefix,
                                const std::string &config_filename);

  struct ConfiguredHoldingRegister {
    std::string name;
    std::vector<std::string> alt_names;
    std::string comment;
    uint32_t min_delay_ms;
    uint16_t addr;
    struct {
      uint16_t min, max, def;
      bool min_set, max_set, def_set;
    } value;
  };
  std::map<
      std::string,
      std::pair<std::shared_ptr<ConfiguredHoldingRegister>,
                rclcpp::Service<srv::ConfiguredHoldingRegisterRead>::SharedPtr>>
      configured_get;
  std::map<std::string,
           std::pair<
               std::shared_ptr<ConfiguredHoldingRegister>,
               rclcpp::Service<srv::ConfiguredHoldingRegisterWrite>::SharedPtr>>
      configured_set;

  virtual void holding_register_read(
      const std::shared_ptr<srv::HoldingRegisterRead::Request> request,
      std::shared_ptr<srv::HoldingRegisterRead::Response> response) = 0;
  virtual void holding_register_write(
      const std::shared_ptr<srv::HoldingRegisterWrite::Request> request,
      std::shared_ptr<srv::HoldingRegisterWrite::Response> response) = 0;
  virtual void holding_register_write_multiple(
      const std::shared_ptr<srv::HoldingRegisterWriteMultiple::Request>,
      std::shared_ptr<srv::HoldingRegisterWriteMultiple::Response>
          response) = 0;
  virtual void get_com_event_log(
      const std::shared_ptr<srv::GetComEventLog::Request> request,
      std::shared_ptr<srv::GetComEventLog::Response> response) = 0;
  virtual void read_device_id(
      const std::shared_ptr<srv::ReadDeviceId::Request> request,
      std::shared_ptr<srv::ReadDeviceId::Response> response) = 0;
  virtual void coil_read(const std::shared_ptr<srv::CoilRead::Request> request,
                         std::shared_ptr<srv::CoilRead::Response> response) = 0;
  virtual void coil_write(
      const std::shared_ptr<srv::CoilWrite::Request> request,
      std::shared_ptr<srv::CoilWrite::Response> response) = 0;
  virtual void coil_continuous_write(
      const std::shared_ptr<srv::CoilContinuousWrite::Request> request,
      std::shared_ptr<srv::CoilContinuousWrite::Response> response) = 0;

 protected:
  rclcpp::Node *node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  // leaf_id_ is the default value.
  // Doesn't make sense on the server side (e.g. 'modbus_rtu_standalone').
  rclcpp::Parameter leaf_id_;

  std::string get_prefix_();

 private:
  rclcpp::Parameter interface_prefix_;

  // configured_holding_register_read is the API call to read configured holding
  // registers. Service handlers call this function to do the job.
  void configured_holding_register_read(
      std::shared_ptr<ConfiguredHoldingRegister> reg, uint8_t &exception_code,
      uint16_t &value);
  void configured_holding_register_read_handler_(
      std::shared_ptr<ConfiguredHoldingRegister> reg,
      const std::shared_ptr<srv::ConfiguredHoldingRegisterRead::Request>
          request,
      std::shared_ptr<srv::ConfiguredHoldingRegisterRead::Response> response);

  void configured_holding_register_write(
      std::shared_ptr<ConfiguredHoldingRegister> reg, uint16_t value,
      uint8_t &exception_code, uint16_t &response_value);
  void configured_holding_register_write_handler_(
      std::shared_ptr<ConfiguredHoldingRegister> reg,
      const std::shared_ptr<srv::ConfiguredHoldingRegisterWrite::Request>
          request,
      std::shared_ptr<srv::ConfiguredHoldingRegisterWrite::Response> response);
};

}  // namespace remote_modbus

#endif  // OPENVMP_MODBUS_INTERFACE_H
