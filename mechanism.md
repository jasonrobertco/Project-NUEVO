# Mechanism Branch Implementation Plan

## Goal

Add isolated mechanism test scripts for the seed-launching mechanism without changing existing Arduino firmware, camera code, bridge protocol, or existing robot behavior.

The mechanism consists of:

1. Kraken 130 3s Neo Motor — flywheel launcher motor
2. Miuzei MG90S 9G Micro Servo — seed kicker/feed servo
3. Deegoo MG996R servo — turret tilt
4. Deegoo MG996R servo — turret pan/yaw

## Current Architecture

Do not create a new direct Raspberry Pi to PCA9685 control path.

The existing architecture is:

```text
Pi / Python / ROS2
  -> Robot API
  -> ROS2 topics / bridge
  -> UART TLV
  -> Arduino MessageCenter
  -> ServoController
  -> PCA9685 over Arduino I2C
  -> Servo