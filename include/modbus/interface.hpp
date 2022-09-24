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

#ifndef OPENVMP_MODBUS_INTERFACE_H
#define OPENVMP_MODBUS_INTERFACE_H

#include <map>
#include <memory>
#include <string>

#include "modbus/srv/configured_holding_register_read.hpp"
#include "modbus/srv/configured_holding_register_write.hpp"
#include "modbus/srv/get_com_event_log.hpp"
#include "modbus/srv/holding_register_read.hpp"
#include "modbus/srv/holding_register_write.hpp"
#include "modbus/srv/holding_register_write_multiple.hpp"
#include "modbus/srv/read_device_id.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "std_msgs/msg/u_int16.hpp"
// #include "std_msgs/msg/u_int32.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_msgs/msg/u_int8.hpp"

#define MODBUS_PUBLISH(type, name, value) \
  {                                       \
    std_msgs::msg::type msg;              \
    msg.data = value;                     \
    name->publish(msg);                   \
  }
#define MODBUS_PUBLISH_INC(type, name, inc)   \
  {                                           \
    name##_value_ += inc;                     \
    MODBUS_PUBLISH(type, name, name##_value_) \
  }

namespace modbus {

class Node;

class ModbusInterface {
 public:
  ModbusInterface(rclcpp::Node *node);
  virtual ~ModbusInterface() {}

  void generate_modbus_mappings(uint8_t leaf_id, const std::string &prefix,
                                const std::string &config_filename);

#ifndef MODBUS_STATS_DISABLE
  // status_leafs_seen is the total number of distinct leaf_id's seen
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr status_leafs_seen;
#endif /* MODBUS_STATS_DISABLE */
  // status_last_leaf is the last leaf_id that responded
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr status_last_leaf;
  // status_last_seen is the timestamp when the last leaf responded
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr status_last_seen;

  // status_last_error_leaf is the last leaf that had an error
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr status_last_error_leaf;
  // status_last_exception_code is the exception code if the error is not
  // timeout or interrupt
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr status_last_exception_code;
  // status_last_error_seen is the timestamp when the last error happened
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr status_last_error_seen;

#ifndef MODBUS_STATS_DISABLE
  class LeafInterface {
   public:
    LeafInterface(rclcpp::Node *node, const std::string &interface_prefix,
                  uint8_t leaf_id);

    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr last_seen;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr last_function_code;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr last_exception_code;
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr last_error_seen;
  };
  std::map<uint8_t, std::shared_ptr<LeafInterface>> leafs;
#endif  // MODBUS_STATS_DISABLE

  rclcpp::Service<modbus::srv::HoldingRegisterRead>::SharedPtr
      holding_register_read;
  rclcpp::Service<modbus::srv::HoldingRegisterWrite>::SharedPtr
      holding_register_write;
  rclcpp::Service<modbus::srv::HoldingRegisterWriteMultiple>::SharedPtr
      holding_register_write_multiple;
  rclcpp::Service<modbus::srv::GetComEventLog>::SharedPtr get_com_event_log;
  rclcpp::Service<modbus::srv::ReadDeviceId>::SharedPtr read_device_id;

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
                rclcpp::Service<
                    modbus::srv::ConfiguredHoldingRegisterRead>::SharedPtr>>
      configured_get;
  std::map<
      std::string,
      std::pair<std::shared_ptr<ConfiguredHoldingRegister>,
                rclcpp::Service<
                    modbus::srv::ConfiguredHoldingRegisterWrite>::SharedPtr>>
      configured_set;

  // configured_holding_register_read is the API call to read configured holding
  // registers. Service handlers call this function to do the job.
  void configured_holding_register_read(
      uint8_t leaf_id, std::shared_ptr<ConfiguredHoldingRegister> reg,
      uint8_t &exception_code, uint16_t &value);

 protected:
  rclcpp::Node *node_;
  rclcpp::Parameter interface_prefix_;

  // Operations with holding registers
  virtual rclcpp::FutureReturnCode holding_register_read_handler_real_(
      const std::shared_ptr<modbus::srv::HoldingRegisterRead::Request> request,
      std::shared_ptr<modbus::srv::HoldingRegisterRead::Response> response) = 0;
  virtual rclcpp::FutureReturnCode holding_register_write_handler_real_(
      const std::shared_ptr<modbus::srv::HoldingRegisterWrite::Request> request,
      std::shared_ptr<modbus::srv::HoldingRegisterWrite::Response>
          response) = 0;
  virtual rclcpp::FutureReturnCode
  holding_register_write_multiple_handler_real_(
      const std::shared_ptr<modbus::srv::HoldingRegisterWriteMultiple::Request>
          request,
      std::shared_ptr<modbus::srv::HoldingRegisterWriteMultiple::Response>
          response) = 0;

  void configured_holding_register_read_handler_(
      uint8_t leaf_id, std::shared_ptr<ConfiguredHoldingRegister> reg,
      const std::shared_ptr<modbus::srv::ConfiguredHoldingRegisterRead::Request>
          request,
      std::shared_ptr<modbus::srv::ConfiguredHoldingRegisterRead::Response>
          response);

  // Misc operations
  virtual rclcpp::FutureReturnCode get_com_event_log_handler_real_(
      const std::shared_ptr<modbus::srv::GetComEventLog::Request> request,
      std::shared_ptr<modbus::srv::GetComEventLog::Response> response) = 0;
  virtual rclcpp::FutureReturnCode read_device_id_handler_real_(
      const std::shared_ptr<modbus::srv::ReadDeviceId::Request> request,
      std::shared_ptr<modbus::srv::ReadDeviceId::Response> response) = 0;

 private:
#ifndef MODBUS_STATS_DISABLE
  uint8_t leafs_seen_;
  uint8_t leafs_seen_bitmap_[32];  // 32 = 256 fc / 8 bits
  std::mutex leafs_seen_bitmap_mutex_;
  void leafs_seen_bitmap_update_(uint8_t leaf_id);
  void update_on_response_(uint8_t leaf_id, uint8_t fc, uint8_t exception_code,
                           rclcpp::FutureReturnCode ret);
#endif  // MODBUS_STATS_DISABLE
  void holding_register_read_handler_(
      const std::shared_ptr<modbus::srv::HoldingRegisterRead::Request> request,
      std::shared_ptr<modbus::srv::HoldingRegisterRead::Response> response);
  void holding_register_write_handler_(
      const std::shared_ptr<modbus::srv::HoldingRegisterWrite::Request> request,
      std::shared_ptr<modbus::srv::HoldingRegisterWrite::Response> response);
  void holding_register_write_multiple_handler_(
      const std::shared_ptr<modbus::srv::HoldingRegisterWriteMultiple::Request>
          request,
      std::shared_ptr<modbus::srv::HoldingRegisterWriteMultiple::Response>
          response);
  void get_com_event_log_handler_(
      const std::shared_ptr<modbus::srv::GetComEventLog::Request> request,
      std::shared_ptr<modbus::srv::GetComEventLog::Response> response);
  void read_device_id_handler_(
      const std::shared_ptr<modbus::srv::ReadDeviceId::Request> request,
      std::shared_ptr<modbus::srv::ReadDeviceId::Response> response);
};

}  // namespace modbus

#endif  // OPENVMP_MODBUS_INTERFACE_H
