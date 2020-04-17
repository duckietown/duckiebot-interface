#!/usr/bin/env python

from duckietown import DTROS
from sensor_msgs.msg import BatteryState
import serial
from serial.serialutil import SerialException
import re
import os
import io
import rospy

RETRIES = 5


class BatteryNotFound(Exception):
    pass


class BatteryMonitor(DTROS):

    def __init__(self, node_name="battery_monitor"):
        super(BatteryMonitor, self).__init__(node_name)

        self.parameters['~device'] = None
        self.updateParameters()

        self.pub = rospy.Publisher("~battery_monitor", BatteryState, queue_size=1)

        devices = filter(lambda x: x.startswith('ttyACM'), os.listdir('/dev'))

        for device in devices:
            device = os.path.join("/dev", device)
            self.ser = serial.Serial(device, baudrate=9600, timeout=2)
            # self.ser_io = io.TextIOWrapper(
            #     io.BufferedRWPair(self.ser, self.ser, 1),
            #     line_buffering=True
            # )
            try:
                self.read_battery_status()
                self.log("Connecting to battery on {}".format(device))
                break
            except BatteryNotFound:
                self.log("No battery found on {}...".format(device))
        else:
            raise BatteryNotFound("No battery found on any USB port. Shutting down battery monitor.")

        self.timer = rospy.Timer(rospy.Duration.from_sec(0.9), self.read_battery_status)

    def read_battery_status(self, event=None):
        """
        Try a few times to read the status from the serial port. If successful, publish it on self.pub
        :param event: Event for using this as a timer callback
        :return: Percentage number, as int, or None if unsuccessful.
        """
        self.ser.reset_input_buffer()
        line = self.ser.readline(16).replace(b'\x00', b' ')
        percent_remaining = self.interpret_line(line)
        if percent_remaining is None:
            raise BatteryNotFound('Did not hear battery sending remaining percentage. Got line: {}'.format(repr(line)))

        """
         - VO: mV
         - TF: Time to full, minutes
         - ET: External temperature
         - CR: Current, mA, negative is discharging
         - TE: Time to empty, minutes
        """
        results = {}
        for command in [b'VO', b'TF', b'ET', b'CR', b'TE']:
            self.ser.write(command)
            # The battery always echos the command on its own line
            cmd_echo = self.ser.readline(4)
            if command not in cmd_echo:
                raise BatteryNotFound('Battery did not respond to command {}'.format(command))
            # After echoing the command, the battery prints the actual data, padded by random whitespace,
            # control characters, and null characters. So filter out all the garbage.
            cmd_response = self.ser.readline(8)
            number = int(bytes(filter(lambda x: x in b'+-0123456789', cmd_response)))
            results[command] = number

        msg = BatteryState()
        msg.voltage = results[b'VO'] / 1000.0
        msg.current = results[b'CR'] / 1000.0
        msg.charge = float('nan')
        msg.capacity = float('nan')
        msg.design_capacity = 10.0
        msg.percentage = percent_remaining / 100.0
        if percent_remaining >= 100:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_FULL
        elif msg.current == 0:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING
        elif msg.current > 0:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
        elif msg.current < 0:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        else:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
        msg.present = True
        msg.cell_voltage = [msg.voltage]
        self.pub.publish(msg)

    @staticmethod
    def interpret_line(line):
        """
        Given a line of text, extracts whatever number it can, and returns it as an integer.

        :param line: Line of text, perhaps with the number in it.
        :return: The percentage number, as an int, or None if unsuccessful
        """
        try:
            number = re.sub(r"[^0-9]", "", line)
            return int(number)
        # AttributeError if the regex does not match. ValueError if the string is not a valid number
        except (AttributeError, ValueError):
            return None


if __name__ == "__main__":
    try:
        node = BatteryMonitor()
        rospy.spin()
    except BatteryNotFound as e:
        print(e)
        print("Unable to communicate with battery. "
              "Assuming old battery is being used, and shutting down battery_monitor")
