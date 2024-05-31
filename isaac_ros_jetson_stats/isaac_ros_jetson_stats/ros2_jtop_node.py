# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.
#
# SPDX-License-Identifier: MIT

from diagnostic_msgs.msg import DiagnosticArray
from isaac_ros_jetson_stats.utils import (board_status,
                                          cpu_status,
                                          disk_status,
                                          emc_status,
                                          engine_status,
                                          fan_status,
                                          gpu_status,
                                          other_status,
                                          power_status,
                                          ram_status,
                                          swap_status,
                                          temp_status)
from isaac_ros_jetson_stats_services.srv import Fan, JetsonClocks, NVPModel
import jtop
import rclpy
from rclpy.node import Node


class JTOPPublisher(Node):

    def __init__(self):
        super().__init__('jtop')
        self.publisher_ = self.create_publisher(
            DiagnosticArray, 'diagnostics', 1)
        self.arr = DiagnosticArray()
        # Get interval parameter, default 0.5 same of jtop
        self.declare_parameter('interval', 0.5)
        # Initialize timer at same interval of jtop frequency
        timer_period = self.get_parameter('interval')._value
        self.timer = self.create_timer(timer_period, self.jetson_callback)
        self.jetson = jtop.jtop(interval=timer_period)
        # Start jtop
        try:
            self.jetson.start()
        except jtop.JtopException as e:
            raise e
        # Check if fan controller is available and create service
        if len(self.jetson.fan) > 0:
            self.get_logger().info('Fan controller detected')
            self.fan_srv = self.create_service(Fan, '~/fan', self.fan_service)
        # Check if jetson_clocks is available and create service
        if self.jetson.jetson_clocks is not None:
            self.get_logger().info('Jetson Clocks detected')
            self.jetson_clocks_srv = self.create_service(
                JetsonClocks, '~/jetson_clocks', self.jetson_clocks_service)
        # Check if nvpmodel is available and create service
        if self.jetson.nvpmodel is not None:
            self.get_logger().info('NVPModel control detected')
            self.nvpmodel_srv = self.create_service(
                NVPModel, '~/nvpmodel', self.nvpmodel_service)
        # Extract board information
        board = self.jetson.board
        # Define hardware name
        self.hardware = board['platform']['Machine']
        self.board_status = board_status(self.hardware, board, 'board')
        self.get_logger().info(
            f'jtop started at interval {self.jetson.interval}')

    def fan_service(self, req, resp):
        fan_profile = req.profile
        fan_speed = req.speed
        # Set first the new fan profile
        try:
            self.jetson.fan.profile = fan_profile
        except jtop.JtopException:
            fan_profile = self.jetson.fan.profile
        # Set the new fan speed
        try:
            self.jetson.fan.speed = fan_speed
        except jtop.JtopException:
            fan_speed = self.jetson.fan.speed
        # Return the fan profile and speed
        resp.new_profile = fan_profile
        resp.new_speed = fan_speed
        self.get_logger().info(
            f'New Fan Profile:{fan_profile} - New Fan Speed:{fan_speed}')
        return resp

    def jetson_clocks_service(self, req, resp):
        # Set new jetson_clocks
        if self.jetson.jetson_clocks is not None:
            new_status = req.status
            try:
                self.jetson.jetson_clocks = new_status
                self.get_logger().info(f'New Jetson Clocks:{new_status}')
            except jtop.JtopException:
                new_status = self.jetson.jetson_clocks.get_enable()
                self.get_logger().error('Fail to set new Jetson clocks status')
            resp.done = new_status
        else:
            resp.done = False
            self.get_logger().warn('Jetson clocks not available')
        return resp

    def nvpmodel_service(self, req, resp):
        # Set new nvpmodel
        if self.jetson.nvpmodel is not None:
            nvpmodel = req.nvpmodel
            try:
                self.jetson.nvpmodel = nvpmodel
                power_mode = str(self.jetson.nvpmodel)
                self.get_logger().info(
                    f'New NVPModel status:{nvpmodel} {power_mode}')
            except jtop.JtopException:
                nvpmodel = self.jetson.nvpmodel
                self.get_logger().error('Fail to set new NVPModel status')
            resp.power_mode = power_mode
        else:
            resp.power_mode = 'Not available'
            self.get_logger().warn('NVPModel not available on this board')
        return resp

    def jetson_callback(self):
        # Add timestamp
        self.arr.header.stamp = self.get_clock().now().to_msg()
        # Status board and board info
        self.arr.status = [other_status(
            self.hardware, self.jetson, jtop.__version__)]
        # Make diagnostic message for each cpu
        self.arr.status += [cpu_status(self.hardware, name, cpu)
                            for name, cpu in enumerate(self.jetson.cpu['cpu'])]
        # Make diagnostic message for each gpu
        self.arr.status += [gpu_status(self.hardware, name, self.jetson.gpu[name])
                            for name in self.jetson.gpu]
        # Make diagnostic message for each engine
        self.arr.status += [engine_status(self.hardware, name, engine)
                            for name, engine in self.jetson.engine.items()]
        # Merge all other diagnostics
        self.arr.status += [ram_status(self.hardware,
                                       self.jetson.memory['RAM'], 'mem')]
        self.arr.status += [swap_status(self.hardware,
                                        self.jetson.memory['SWAP'], 'mem')]
        if 'EMC' in self.jetson.memory:
            self.arr.status += [emc_status(self.hardware,
                                           self.jetson.memory['EMC'], 'mem')]
        # Make diagnostic message for each Temperature
        self.arr.status += [temp_status(self.hardware, name, sensor)
                            for name, sensor in self.jetson.temperature.items()]
        # Make diagnostic message for each power rail
        self.arr.status += [power_status(self.hardware, name, rail)
                            for name, rail in self.jetson.power['rail'].items()]
        if 'name' in self.jetson.power['tot']:
            name_total = self.jetson.power['tot']['name']
        else:
            name_total = 'ALL'
        self.arr.status += [power_status(self.hardware,
                                         name_total, self.jetson.power['tot'])]
        # Make diagnostic message for each Fan controller
        if self.jetson.fan:
            self.arr.status += [fan_status(self.hardware, name, fan)
                                for name, fan in self.jetson.fan.items()]
        # Status board and board info
        self.arr.status += [self.board_status]
        # Add disk status
        self.arr.status += [disk_status(self.hardware,
                                        self.jetson.disk, 'board')]
        # Update status jtop
        self.publisher_.publish(self.arr)


def main(args=None):
    rclpy.init(args=args)
    # Start ROS 2 jtop publisher
    try:
        jtop_publisher = JTOPPublisher()
        rclpy.spin(jtop_publisher)
    except jtop.JtopException as e:
        print(e)
    except (KeyboardInterrupt, SystemExit):
        jtop_publisher.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
# EOF
