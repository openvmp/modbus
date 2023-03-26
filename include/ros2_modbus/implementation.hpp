/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-09-24
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_MODBUS_IMPLEMENTATION_H
#define OPENVMP_MODBUS_IMPLEMENTATION_H

#include <map>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "ros2_modbus/interface.hpp"
#include "ros2_modbus/srv/get_com_event_log.hpp"
#include "ros2_modbus/srv/holding_register_read.hpp"
#include "ros2_modbus/srv/holding_register_write.hpp"
#include "ros2_modbus/srv/holding_register_write_multiple.hpp"
#include "ros2_modbus/srv/read_device_id.hpp"
#include "std_msgs/msg/u_int16.hpp"
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

namespace ros2_modbus {

class Implementation : public Interface {
 public:
  Implementation(rclcpp::Node *node);
  virtual ~Implementation() {}

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

  rclcpp::Service<srv::HoldingRegisterRead>::SharedPtr
      srv_holding_register_read;
  rclcpp::Service<srv::HoldingRegisterWrite>::SharedPtr
      srv_holding_register_write;
  rclcpp::Service<srv::HoldingRegisterWriteMultiple>::SharedPtr
      srv_holding_register_write_multiple;
  rclcpp::Service<srv::GetComEventLog>::SharedPtr srv_get_com_event_log;
  rclcpp::Service<srv::ReadDeviceId>::SharedPtr srv_read_device_id;

 protected:
  // TO BE IMPLEMENTED BY RTU/TCP/ASCII
  // Operations with holding registers
  virtual rclcpp::FutureReturnCode holding_register_read_handler_real_(
      const std::shared_ptr<srv::HoldingRegisterRead::Request> request,
      std::shared_ptr<srv::HoldingRegisterRead::Response> response) = 0;
  virtual rclcpp::FutureReturnCode holding_register_write_handler_real_(
      const std::shared_ptr<srv::HoldingRegisterWrite::Request> request,
      std::shared_ptr<srv::HoldingRegisterWrite::Response> response) = 0;
  virtual rclcpp::FutureReturnCode
  holding_register_write_multiple_handler_real_(
      const std::shared_ptr<srv::HoldingRegisterWriteMultiple::Request> request,
      std::shared_ptr<srv::HoldingRegisterWriteMultiple::Response>
          response) = 0;
  // Misc operations
  virtual rclcpp::FutureReturnCode get_com_event_log_handler_real_(
      const std::shared_ptr<srv::GetComEventLog::Request> request,
      std::shared_ptr<srv::GetComEventLog::Response> response) = 0;
  virtual rclcpp::FutureReturnCode read_device_id_handler_real_(
      const std::shared_ptr<srv::ReadDeviceId::Request> request,
      std::shared_ptr<srv::ReadDeviceId::Response> response) = 0;

 private:
#ifndef MODBUS_STATS_DISABLE
  uint8_t leafs_seen_;
  uint8_t leafs_seen_bitmap_[32];  // 32 = 256 fc / 8 bits
  std::mutex leafs_seen_bitmap_mutex_;
  void leafs_seen_bitmap_update_(uint8_t leaf_id);
  void update_on_response_(uint8_t leaf_id, uint8_t fc, uint8_t exception_code,
                           rclcpp::FutureReturnCode ret);
#endif  // MODBUS_STATS_DISABLE

  // SERVICES
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
};

}  // namespace ros2_modbus

#endif  // OPENVMP_MODBUS_IMPLEMENTATION_H
