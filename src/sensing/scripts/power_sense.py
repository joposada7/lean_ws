#!/usr/bin/env python3
import rospy
import time
import board
from std_msgs.msg import Float32
from adafruit_ina219 import ADCResolution, BusVoltageRange, INA219


class PowerSensor():
    """
    Creates a new power sensor and reads
    """
    def __init__(self, i2c_bus, address) -> None:
        self.i2c_bus = i2c_bus
        self.address = address
        self.sensor = INA219(i2c_bus, address)
        #configure to measure up to 32V and 1A
        self.sensor.set_calibration_32V_1A()
        #change configuration to use 32 samples averaging for both bus voltage and shunt voltage
        self.sensor.bus_adc_resolution = ADCResolution.ADCRES_12BIT_32S
        self.sensor.shunt_adc_resolution = ADCResolution.ADCRES_12BIT_32S
        #change voltage range to 16V
        self.sensor.bus_voltage_range = BusVoltageRange.RANGE_16V
    
    def get_voltage(self) -> float:
        return self.sensor.bus_voltage
    
    def get_power(self) -> float:
        return self.sensor.power
    
def sense():
    # define i2c communication protocol
    i2c_bus = board.I2C()
    # create sensor objects
    computation = PowerSensor(i2c_bus, 0x40)
    actuation = PowerSensor(i2c_bus, 0x41)
    # create publishers for each data
    comp_power_pub = rospy.Publisher('computation_power', Float32, queue_size=1)
    act_power_pub = rospy.Publisher('actuation_power', Float32, queue_size=1)
    comp_volt_pub = rospy.Publisher('computation_voltage', Float32, queue_size=1)
    act_volt_pub = rospy.Publisher('actuation_voltage', Float32, queue_size=1)
    rospy.init_node('sensors', anonymous=True)
    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        # publish our data
        comp_power_pub.publish(computation.get_power())
        act_power_pub.publish(actuation.get_power())
        comp_volt_pub.publish(computation.get_voltage())
        act_volt_pub.publish(actuation.get_voltage())
        rate.sleep()

if __name__ == '__main__':
    try:
        sense()
    except rospy.ROSInterruptException:
        pass