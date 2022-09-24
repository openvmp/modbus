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

#ifndef OPENVMP_MODBUS_PROTOCOL_H
#define OPENVMP_MODBUS_PROTOCOL_H

#define MODBUS_FC_READ_HOLDING_REGISTERS ((uint8_t)0x03)
#define MODBUS_FC_PRESET_SINGLE_REGISTER ((uint8_t)0x06)
#define MODBUS_FC_PRESET_MULTIPLE_REGISTERS ((uint8_t)0x10)
#define MODBUS_FC_GET_COM_EVENT_LOG ((uint8_t)0x0C)
#define MODBUS_FC_READ_DEVICE_ID ((uint8_t)0x2B)

#define MODBUS_EXC_ILLEGAL_FUNCTION 0x01
#define MODBUS_EXC_ILLEGAL_DATA_ADDRESS 0x02
#define MODBUS_EXC_ILLEGAL_DATA_VALUE 0x03
#define MODBUS_EXC_SERVER_DEVICE_FAILURE 0x04
#define MODBUS_EXC_ACKNOWLEDGE 0x05
#define MODBUS_EXC_SERVER_DEVICE_BUSY 0x06
#define MODBUS_EXC_NEGATIVE_ACKNOWLEDGE 0x07
#define MODBUS_EXC_MEMORY_PARITY_ERROR 0x08
#define MODBUS_EXC_GATEWAY_PATH_UNAVAILABLE 0x0A
#define MODBUS_EXC_GATEWAY_TARGET_DEVICE_FAILED_TO_RESPOND 0x0B

namespace modbus {}

#endif  // OPENVMP_MODBUS_PROTOCOL_H