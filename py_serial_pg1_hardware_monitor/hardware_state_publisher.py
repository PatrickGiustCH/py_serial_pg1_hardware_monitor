
import serial
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray


class HardwareStatePublisher(Node):

    def __init__(self, ser):
        super().__init__('hardware_state_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'hardware_state', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.ser = ser
        self.current_voltage = 0.0
        self.headlight_state = 0
        self.fan_speed = 0

    def timer_callback(self):


        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode('utf-8').rstrip()
            for i in range(0, len(line)):
                if(line[i] == 'v'):
                    self.current_voltage = ''.join(filter(str.isdigit,line[i+1:i+6]))
                    # print("current voltage is: ", current_voltage)

                elif(line[i] == 'h'):
                    self.headlight_state = ''.join(filter(str.isdigit,line[i+1:i+2]))
                    # print("current headlight is", headlight_state)
                    
                elif(line[i] == 'f'):
                    self.fan_speed = ''.join(filter(str.isdigit,line[i+1:i+4]))
                    # print("current headlight is", headlight_state)


        clear_serial_buffer(self.ser)
        msg = Float32MultiArray()
        msg.data =[float(self.current_voltage)/100, float(self.headlight_state), float(self.fan_speed)]
        self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
    ser.flush()

    hardware_state_publisher = HardwareStatePublisher(ser=ser)

    rclpy.spin(hardware_state_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    hardware_state_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



def clear_serial_buffer(serial_port):
    while serial_port.in_waiting > 0:
        serial_port.read(serial_port.in_waiting)