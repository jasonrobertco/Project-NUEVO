# Bridge Runtime vs ROS Wrapper

This document explains the two layers involved in the ROS2 bridge design.


## The Shared Runtime

The shared runtime lives here:

```text
nuevo_ui/backend/nuevo_bridge/
```

This is a normal Python application. It is responsible for:

- owning the Arduino serial connection
- decoding and encoding the compact TLV protocol
- keeping bridge-side cached state
- serving the FastAPI and WebSocket UI
- handling commands from the UI

It must stay usable **without ROS2 installed** so the project can still be
tested with only the web UI.


## The ROS Wrapper Package

The ROS wrapper package lives here:

```text
ros2_ws/src/bridge/
```

This package is responsible for:

- creating the ROS node
- publishing raw firmware-aligned ROS topics
- subscribing to raw command topics
- converting between ROS messages and the shared bridge command/data format

It should stay thin. It should not duplicate serial transport, decode logic, or
web backend ownership.


## Why Split Them

This split gives us two entry modes over the same runtime:

### Plain Python mode

- start the shared backend only
- use the web UI
- no ROS2 required

### ROS mode

- start the ROS package `bridge`
- the same shared runtime still owns the serial connection and UI
- ROS publishers and subscribers attach to that same runtime


## One Process in ROS Mode

Package separation does **not** mean two bridge processes.

When the ROS bridge is running, the intended model is still one integrated
process:

- one serial connection
- one decode pipeline
- one UI server
- one ROS node

That keeps the UI and ROS views consistent and avoids double-opening the serial
port.


## Rule of Thumb

- Put transport, protocol, cache, and web-server code in `nuevo_bridge`
- Put ROS publishers, ROS subscribers, launch files, and ROS conversions in
  `bridge`
