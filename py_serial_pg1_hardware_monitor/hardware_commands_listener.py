# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import serial
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray


class HardwareCommandsListener(Node):

    def __init__(self, ser):
        super().__init__('hardware_commands_listener')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'hardware_commands',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.ser = ser
        self.fan_speed = 123
        self.old_fan_speed = 99999
        self.headlight_state = 0
        self.old_headlight_state = 99999
        self.reset_counter = 0



    def listener_callback(self, msg):
        data = msg.data
        # data = [fan_speed 0-255, headlight_state, ]
        self.fan_speed = data[0]
        self.headlight_state = data[1]


        if(self.fan_speed != self.old_fan_speed):
            cmd = "fan_speed"+str(self.fan_speed)+"\n"
            self.ser.write(cmd.encode())
            self.old_fan_speed = self.fan_speed

        if(self.headlight_state != self.old_headlight_state):
            self.ser.write(b"headlight_on\n" if self.headlight_state else b"headlight_off\n" )
            self.old_headlight_state = self.headlight_state
        




        if(self.reset_counter >= 10): #resed message every 10 iters
            self.reset_counter = 0
            self.old_fan_speed = 99999
            self.old_headlight_state = 9999
        else:
            self.reset_counter += 1
        # self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
    ser.flush()

    hardware_commands_listener = HardwareCommandsListener(ser=ser)

    rclpy.spin(hardware_commands_listener)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    hardware_commands_listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
