#!/usr/bin/env python3
import math
import time

import rospy
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from imu_driver import mpu9250
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3

# TODO: calibration and loading custom config

G_mps2 = 9.80665  # 1 G in m/s^2
DEG2RAD = math.pi / 180


class IMUNotFound(Exception):
    pass


class IMUNode(DTROS):
    def __init__(self):
        super(IMUNode, self).__init__(
            node_name="imu_node",
            node_type=NodeType.DRIVER,
        )
        # get parameters
        self._veh = rospy.get_param('~veh')
        i2c_bus = rospy.get_param("~i2c_bus")
        i2c_address = rospy.get_param("~i2c_address")
        polling_hz = rospy.get_param("~polling_hz")

        self._ang_vel_offset = DTParam(
            "~ang_vel_offset",
            param_type=ParamType.LIST,
            help="Angular velocity offsets",  # TODO
        )
        self._accel_offset = DTParam(
            "~accel_offset",
            param_type=ParamType.LIST,
            help="Acceleration offset",
        )

        trial_msg = lambda msg: "At I2C <bus{}, addr{}>: {}".format(
            i2c_bus, i2c_address, msg)
        try:
            self._imu = mpu9250(i2c_bus, i2c_address)
            _ = self._imu.accel
            _ = self._imu.gyro
            self.loginfo(trial_msg("Found IMU"))
        except IOError:
            self.logerr(trial_msg("IMU sensor not correctly detected"))
            raise IMUNotFound()

        self.pub = rospy.Publisher('~imu_data', Imu, queue_size=10)
        self.timer = rospy.Timer(
            rospy.Duration.from_sec(1.0 / polling_hz),
            self.publish_data,
        )

    def calc3(self, const, data_zip_offset):
        """
        The size of data_zip_offset should be 3
        Calculate: data[i] * const - offset[i]
        Returns a Vector3() storing in (x, y, z) the (0, 1, 2)-th calculated num
        """
        values = [data * const - offset for data, offset in data_zip_offset]
        ret = Vector3()
        ret.x, ret.y, ret.z = values
        return ret

    def calc_angular_velocity(self, gyro_data):
        return self.calc3(DEG2RAD, zip(gyro_data, self._ang_vel_offset.value))

    def calc_linear_acceleration(self, acc_data):
        return self.calc3(G_mps2, zip(acc_data, self._accel_offset.value))

    def publish_data(self, event):
        try:
            acc_data = self._imu.accel
            self.logdebug('Accel: {:.3f} {:.3f} {:.3f} mg'.format(*acc_data))
            gyro_data = self._imu.gyro
            self.logdebug('Gyro: {:.3f} {:.3f} {:.3f} dps'.format(*gyro_data))

            try:
                mag = self._imu.mag
                self.logdebug('Magnet: {:.3f} {:.3f} {:.3f} mT'.format(*mag))
                temp = self._imu.temp
                self.logdebug('Temperature: {:.3f} C'.format(temp))
            except:
                self.logdebug("Didn't manage to read magnet/temp")

            msg = Imu()
            msg.header.stamp = rospy.Time.now()
            # msg.header.frame_id = "/odom"  # TODO
            msg.angular_velocity = self.calc_angular_velocity(gyro_data)
            msg.linear_acceleration = self.calc_linear_acceleration(acc_data)

            # TODO
            for i in range(0, 9):
                msg.angular_velocity_covariance[i] = 0
                msg.linear_acceleration_covariance[i] = 0
                msg.orientation_covariance[i] = -1

            self.pub.publish(msg)

        except IOError as e:
            self.logwarn(f"I/O error: {e}")


if __name__ == '__main__':
    try:
        node = IMUNode()
        rospy.spin()
    except IMUNotFound as e:
        # Err already logged in node
        # Don't kill the entire interface container
        pass
