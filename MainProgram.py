#!/usr/bin/python
# encoding:utf-8
# -*- coding: utf-8 -*-
import time
import datetime
import sys,os,csv,io
import RPi.GPIO as GPIO
sys.path.append("/home/pi/Desktop/Projet2020/ADXL362")
from defined_function import *
import adxl362
from adxl362 import ADXL362
# Open file
filename = '{0:%Y%m%d-%H.%M.txt}'.format(datetime.datetime.now())
output_file = open(filename,'w')
print(output_file)
# Constants
temp_scale = -9.05 # LSB/ºC
temp_bias = 1852 # LSB
# Start sensor

sensor = ADXL362()
time.sleep(0.2)
# Reset all settings
sensor.reset_settings()
time.sleep(0.2)
# Set measure mode
sensor.set_measure_mode(drdy_off = 1, temp_off = 0, standby = 0)
# Set settings
f_s = 3 # [Hz]
dt = 1/f_s # [s]
range_g = 0x01 # 2g range
sensor._set_measure_range(range_g) #Set range
odr_lpf = 0xA # ODR: 3.906Hz, LPF: 0.977Hz
#odr_lpf = 0x04 # ODR: 500Hz, LPF: 125Hz
hpf = 0x00 # No high pass filter
sensor.set_ODR_and_filter(odr_lpf, hpf)
time.sleep(0.2)
# Display settings
range_g = sensor.get_measure_range()
range_g = bits2range(range_g)
scale = range_g[1]
odr_filter = sensor.get_ODR_and_filter()
odr_lpf_hpf = bits2odr_filter(odr_filter) # Interpret bits
offsets=sensor.get_offsets()
sync_bits = sensor.get_sync()
syncs = bits2sync(sync_bits) # Interpret bits
status = sensor.get_status()
print('Measure range: ',range_g[0],file=output_file)
print('Output Data Rate: ', odr_lpf_hpf[0], file=output_file)
print('Low pass filter: ', odr_lpf_hpf[1], file=output_file)

print('High pass filter: ', odr_lpf_hpf[2], file=output_file)
print('Data on twos_comp method', file=output_file)
print('Data sampled at: ', f_s, file=output_file)
print('\n', file=output_file)
# Display settings on terminal
print('Measure range: ', range_g[0])
print('Output Data Rate: ', odr_lpf_hpf[0])
print('Low pass filter: ', odr_lpf_hpf[1])
print('High pass filter: ', odr_lpf_hpf[2])
print('Data on twos_comp method')
print('Data sampled at: ', f_s)
print('\n')
print('Sensor running...')
print('\n')
time.sleep(0.01)
n = 1
t = 0.0
# Infinite loop
while (1):
    startTime = time.time()
    axes_temp = sensor.get_axes_and_temp()
    accel = bits2accel(axes_temp[0:3], [scale, scale, scale], offsets)
    temp = bits2temp(axes_temp[3], temp_scale, temp_bias)
    n = n + 1
    t = t + time.time() - startTime
    print('ID: %d t: %f x: %f y: %f z: %f Temp: %f' % (n, t, \
    accel[0], accel[1], accel[2], temp), file=output_file)
    time.sleep(dt)
    

