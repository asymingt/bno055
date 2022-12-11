# Copyright 2021 AUTHORS
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the AUTHORS nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


import sys

from bno055.connectors.i2c import I2C
from bno055.connectors.uart import UART
from bno055.error_handling.exceptions import BusOverRunException
from bno055.params.NodeParameters import NodeParameters
from bno055.sensor.SensorService import SensorService
import rclpy
from rclpy.node import Node


class Bno055Node(Node):
    """
    ROS2 Node for interfacing Bosch Bno055 IMU sensor.

    :param Node: ROS2 Node Class to initialize from
    :type Node: ROS2 Node
    :raises NotImplementedError: Indicates feature/function is not implemented yet.
    """

    sensor = None
    param = None

    rot_timer = None
    imu_timer = None
    mag_timer = None
    tmp_timer = None
    cal_timer = None
    gra_timer = None

    def __init__(self):

        # Initialize parent (ROS Node)
        super().__init__('bno055')

    def setup(self):

        # Initialize ROS2 Node Parameters:
        
        self.param = NodeParameters(self)

        # Get connector according to configured sensor connection type:

        if self.param.connection_type.value == UART.CONNECTIONTYPE_UART:
            connector = UART(self,
                             self.param.uart_baudrate.value,
                             self.param.uart_port.value,
                             self.param.uart_timeout.value)
        elif self.param.connection_type.value == I2C.CONNECTIONTYPE_I2C:
            connector = I2C(self,
                            self.param.i2c_bus.value,
                            self.param.i2c_addr.value)
        else:
            raise NotImplementedError('Unsupported connection type: '
                                      + str(self.param.connection_type.value))

        # Connect to BNO055 device:
        
        connector.connect()

        # Instantiate the sensor Service API:
        
        self.sensor = SensorService(self, connector, self.param)

        # Configure device

        self.sensor.configure()

        # Start all requested timers

        if self.param.rot_query_frequency.value > 0:
            self.rot_timer = self.create_timer(
                1.0 / float(self.param.rot_query_frequency.value), self.sensor.pub_rot_data)

        if self.param.gra_query_frequency.value > 0:
            self.gra_timer = self.create_timer(
                1.0 / float(self.param.gra_query_frequency.value), self.sensor.pub_gra_data)

        if self.param.imu_query_frequency.value > 0:
            self.imu_timer = self.create_timer(
                1.0 / float(self.param.imu_query_frequency.value), self.sensor.pub_imu_data)
        
        if self.param.mag_query_frequency.value > 0:
            self.mag_timer = self.create_timer(
                1.0 / float(self.param.mag_query_frequency.value), self.sensor.pub_mag_data)
                
        if self.param.tmp_query_frequency.value > 0:
            self.tmp_timer = self.create_timer(
                1.0 / float(self.param.tmp_query_frequency.value), self.sensor.pub_tmp_data)

        if self.param.cal_query_frequency.value > 0:
            self.cal_timer = self.create_timer(
                1.0 / float(self.param.cal_query_frequency.value), self.sensor.pub_cal_data) 

def main(args=None):
    rclpy.init()
    node = Bno055Node()
    node.setup()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Ctrl+C received - exiting...')
    finally:
        node.get_logger().info('ROS node shutdown')
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
