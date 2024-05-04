#!/usr/bin/env python3

import argparse
import os
import csv
from datetime import datetime
import time
import numpy as np

import board
from adafruit_ina219 import ADCResolution, BusVoltageRange, INA219

"""
This script takes a named argument --time [-t] in seconds, to record power data
on the RPi2. It then outputs the data to a .csv in ~/data/
"""

class PowerSensor():
    """
    Initialize INA219 sensor and get voltage/power of
        - Computation battery
        - Actuation battery
    """
    def __init__(self, i2c_bus, address):
        self.sensor = INA219(i2c_bus, address)
        self.sensor.set_calibration_32V_1A() # Measures up to 32V and 1A

        # Configuration to use 32 samples averaging for both bus voltage and shunt voltage
        self.sensor.bus_adc_resolution = ADCResolution.ADCRES_12BIT_32S
        self.sensor.shunt_adc_resolution = ADCResolution.ADCRES_12BIT_32S

        # 16V range
        self.sensor.bus_voltage_range = BusVoltageRange.RANGE_16V
    
    def get_voltage(self):
        return self.sensor.bus_voltage
    
    def get_power(self):
        return self.sensor.power



if __name__ == '__main__':
    # Parse arguments
    p = argparse.ArgumentParser()
    p.add_argument('-t', '--time', dest='time', type=float, nargs='?', default=10, help="Amount of time in seconds to record power data.")
    p.add_argument('-dt', '--timestep', dest='dt', type=float, nargs='?', default=0.01, help="Timestep in seconds for each time to record data.")
    args = p.parse_args()
    tf = args.time
    dt = args.dt

    # Setup power sensors
    print("Setting up power sensors...")
    i2c_bus = board.I2C()
    computation_sensor = PowerSensor(i2c_bus, 0x40)
    actuation_sensor = PowerSensor(i2c_bus, 0x41)

    times = []
    computation_power = []
    computation_voltage = []
    actuation_power = []
    actuation_voltage = []

    print("Recording power data now...")
    t = 0.0
    while t < tf:
        # Keep recording power data for duration of time.
        cp = computation_sensor.get_power()
        cv = computation_sensor.get_voltage()
        ap = actuation_sensor.get_power()
        av = actuation_sensor.get_voltage()

        computation_power.append(cp)
        computation_voltage.append(cv)
        actuation_power.append(ap)
        actuation_voltage.append(av)
        times.append(t)

        t += dt
        time.sleep(dt)
    print("Done recording data!")

    # Format data
    power_data = [times, computation_power, computation_voltage, actuation_power, actuation_voltage]
    power_data = np.column_stack(power_data)

    # Save to folder in home directory
    d = os.path.dirname(os.path.expanduser('~'))
    datapath = os.path.join(d,os.getlogin(),'data')
    if not os.path.exists(datapath):
        # If folder doesn't exist, make one
        os.makedirs(datapath)

    # Determine number for filename
    names = os.listdir(datapath)
    next_num = 1
    for name in names:
        if name.find("power_data")!=-1:
            s = name.split('-')
            num = int(s[-1][:-4])
            if num >= next_num:
                next_num = num+1
    filename = str(datetime.now().strftime("%m-%d"))+"-"+str(next_num)

    # Store power data to file
    input_path = os.path.join(datapath,'power_data-'+filename+'.csv')
    with open(input_path, 'w+', newline='') as f:
        w = csv.writer(f)
        w.writerow(["t", "computation_power", "computation_voltage", "actuation_power", "actuation_voltage"]) # Header
        w.writerows(power_data)

    print("Logged power data to ~/data/power-data-" + filename + ".csv")
