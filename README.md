# Isaac ROS Jetson

Isaac ROS Jetson

## Overview

Isaac ROS Jetson simplifies the process of monitoring and managing NVIDIA Jetson, empowering developers to optimize performance, ensure system stability, and streamline development workflows for ROS 2.

## What is `jetson-stats`?

[jetson_stats](https://rnext.it/jetson_stats) is a comprehensive utility tool specifically designed for NVIDIA Jetson, including the Jetson Nano, Jetson TX1, TX2, Xavier, Orin, and Thor series.
It provides a variety of functionalities to monitor and manage the Jetson device’s performance, temperature, power usage, and more.

<div align="center"><a class="reference internal image-reference" href="https://media.githubusercontent.com/media/NVIDIA-ISAAC-ROS/.github/release-4.1/resources/isaac_ros_docs/repositories_and_packages/isaac_ros_jetson/jetson_stats.png/"><img alt="image" src="https://media.githubusercontent.com/media/NVIDIA-ISAAC-ROS/.github/release-4.1/resources/isaac_ros_docs/repositories_and_packages/isaac_ros_jetson/jetson_stats.png/" width="880px"/></a></div>

## How Does it Work with Isaac ROS?

The Isaac ROS Jetson Stats package wraps the output from the [jetson_stats](https://rnext.it/jetson_stats) package and publishes a diagnostic message with the status of your device.
It remaps all board statuses to the desired format for more accessible analysis and monitoring.

The package provides the following features:
: * System Monitoring
  * Fan Control
  * GPU Frequency Control
  * Power Management
  * Thermal Throttling Monitoring
  * Memory Usage
  * JetPack Version Detection

To learn more about Jetson Stats, refer to the [jetson_stats](https://rnext.it/jetson_stats) documentation.

## Why use in `isaac_ros_jetson` on my robotics application?

ROS designed a tool for aggregating diagnostics messages published from other nodes that publish these messages and republish them in a single diagnostic topic.
This makes it easier to monitor and manage the diagnostics of a ROS system, especially in complex robotic systems where multiple nodes may be publishing diagnostic information.

Isaac ROS Jetson provides a set of messages to monitor the health of your board while the robot is working and tracking what is happening.

As an illustration, consider a scenario where your board is under high power demand. In such a case, a node that reads the diagnostics message can dynamically adjust the `nvp` power mode or even disable certain nodes that are consuming excessive computational resources. This dynamic power management capability is a key feature of the tool.

Using the `isaac_ros_jetson` package in the Isaac ROS environment can greatly improve the robustness and reliability of your project. This method ensures a consistent and isolated environment, reducing compatibility issues and making deployment across different platforms easier.

---

## Documentation

See [Isaac ROS Documentation](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_jetson/index.html) to learn how to use this repository.

---

## Packages

* [`isaac_ros_jetson_stats`](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_jetson/isaac_ros_jetson_stats/index.html)
  * [Quickstart](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_jetson/isaac_ros_jetson_stats/index.html#quickstart)
  * [Control your Jetson](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_jetson/isaac_ros_jetson_stats/index.html#control-your-jetson)
  * [Troubleshooting](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_jetson/isaac_ros_jetson_stats/index.html#troubleshooting)
  * [API](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_jetson/isaac_ros_jetson_stats/index.html#api)
* [`isaac_ros_jetson_stats_services`](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_jetson/isaac_ros_jetson_stats_services/index.html)

## Latest

Update 2026-02-02: Support for two new Docker-optional development and deployment modes
