# ROS2 Modbus

## Introduction

This package is an ultimate C++ implementation of Modbus for ROS2.

Most server use cases require
[Modbus RTU](https://github.com/openvmp/modbus_rtu) or
[Modbus TCP](https://github.com/openvmp/modbus_tcp) instead. However sometimes
the servers use this library to simulate the devices or to expose a non-Modbus
device using a Modbus interface.

ROS2 clients use this library to communicate with other ROS2 nodes in a way
that is abstracted away from the particular method (RTU or TCP).

It is a part of [the OpenVMP project](https://github.com/openvmp/openvmp)
But it is made to be universal and usable anywhere.

## Architecture

The below diagram explains how the modbus library fits into the overall ROS2
architecture both on the client and server sides. See the following sections for
more details.

```mermaid
flowchart TB
    cli_serial["# Modbus debugging\n$ ros2 topic echo /modbus/example_bus"] .-> topic_modbus[/Topic:\n/modbus/]
    subgraph modbus["Library: modbus"]
      topic_modbus
    end
    subgraph your_exe["Client process"]
      code_dds["Client code\nthat consumes\ntype definitions"]
      code_api["Client code\nthat consumes\nnative API"]
      code_dds --> topic_modbus[/Topic:\n/modbus/example_bus/]
      subgraph modbus_client_lib["Library: modbus"]
        modbus_topics["Modbus type definitions"]
        modbus_client["Modbus client API"]
        modbus_topics -.- modbus_client
      end
      modbus_topics ---> code_dds
      code_api ---> modbus_client
    end
    modbus_client --> topic_modbus
    subgraph modbus_exe["Server process"]
      modbus_server["Modbus server API"]
      subgraph modbus
        topic_modbus --> modbus_server
      end
      modbus_server <--> server["Server code"]
    end
    server .-> device{{"External devices"}}
    style device stroke-dasharray: 3 3
```

### Modbus clients

The client may consume ROS type definitions and send the messages directly to
the Modbus service providers using ROS communications.

Alternatively, the client may consume a native API which can send simple
messages.

### Modbus servers

The servers use the modbus library to instantiate ROS2 topics for communication
with clients.

This interface is used by [Modbus RTU](https://github.com/openvmp/modbus_rtu)
and [Modbus TCP](https://github.com/openvmp/modbus_tcp) libraries. But it can be
used to implement alternative access to hardware interfaces that the clients are
expecting to access using Modbus.
