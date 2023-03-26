# OpenVMP

[![License](./license.svg)](./LICENSE.txt)

This package is a part of [the OpenVMP project](https://github.com/openvmp/openvmp).
But it's designed to be universal and usable independently from the rest of OpenVMP or in a combination with select OpenVMP packages.

## ROS2 Modbus

This package is an ultimate C++ implementation of Modbus for ROS2.

Most server use cases require
[Modbus RTU](https://github.com/openvmp/modbus_rtu) or
[Modbus TCP](https://github.com/openvmp/modbus_tcp) instead. However sometimes
the servers use this library to simulate the devices or to expose a non-Modbus
device using a Modbus interface.

ROS2 clients use this library to communicate with other ROS2 nodes in a way
that is abstracted away from the particular method (RTU or TCP).

### Architecture

The below diagram explains how the modbus library fits into the overall ROS2
architecture both on the client and server sides. See the following sections for
more details.

```mermaid
flowchart TB
    cli_serial["# Modbus debugging\n$ ros2 topic echo /modbus/example_bus"] .-> topic_modbus[/ROS2 interfaces:\n/modbus/example_bus/.../]
    subgraph modbus["Library: ros2_modbus"]
      topic_modbus
    end
    users["Client's\nconsumers"]
    subgraph your_exe["Client process"]
      config["Register layout\nin YAML"]
      subgraph modbus_client_svcs["Services: .../modbus/..."]
        generated_services["Register-specific\nservices generated"]
        generated_services --> topic_modbus
      end
      code_dds["Client code\nthat consumes\ntype definitions"]
      code_api["Client code\nthat consumes\nnative API"]
      code_dds --> topic_modbus
      subgraph modbus_client_lib["Library: ros2_modbus"]
        generate["Generate\nregister\nmappings"]
        modbus_topics["ROS2 interface declarations"]
        modbus_client["Modbus client API"]
        modbus_topics -.- modbus_client
      end
      config ---> generate
      modbus_topics ---> code_dds
      code_api ---> modbus_client
    end
    users --> generated_services
    generate --> generated_services
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

### ROS2 interface

Topics:

- /modbus/\<bus-name\>/status/
  - leafs_seen
  - last_leaf
  - last_seen
- /modbus/\<bus-name\>/\<leaf-id\>/
  - last_seen
  - last_function_code
  - last_error_code

Services:

- /modbus/\<bus-name\>/\<leaf-id\>/
  - holding_register_read
  - holding_register_write _(not yet)_
  - holding_register_write_multiple _(not yet)_

### Native interface

The native interface duplicates the ROS2 service interfaces.
The same request and response data types are reused.


### Known Limitations

- As of now, the only supported object type is 'Holding register'
