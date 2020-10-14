#!/usr/bin/python
# -*- coding: utf-8 -*-
# Import necessary libraries
import spidev
# SPI configuration
SPI_MAX_CLOCK_HZ = 10000000
SPI_MIN_CLOCK_HZ = 100000
SPI_MODE = 0b00 # CPOL = 0, CPHA = 0
SPI_BUS = 1
SPI_DEVICE = 0
# ADXL362 addresses
DEVID_AD = 0x00
DEVID_MST = 0x01
PARTID = 0x02
REVID = 0x03
XDATA=0x08
YDATA=0x09
ZDATA=0x0A
STATUS = 0x0B
ACC_WRITE = 0x0A
ACC_READ = 0x0B
SIZE_BUFFER=0x3
SIZE_FIFO=0x60
FIFO_ENTRIES_L = 0x0C
FIFO_ENTRIES_H = 0x0D
XDATA_L=0x0E
XDATA_H=0x0F
YDATA_L=0x10
YDATA_H=0x11
ZDATA_L=0x12
ZDATA_H=0x13
TEMP_L = 0x14
TEMP_H = 0x15
SOFT_RESET=0x1F
THRESH_ACT_L = 0x20
THRESH_ACT_H = 0x21
TIMETHRESH_ACT=0x22
THRESH_INACT_L=0x23
THRESH_INACT_H=0x24
TIME_INACT_L=0x25
THRESH_INACT_H=0x26
ACT_INACT_CTL=0x27
FIFO_CONTROL=0x28
FIFO_SAMPLES=0x29
INTMAP1=0x2A
INTMAP2=0x2B

FILTER_CTL=0x2C
POWER_CTL=0x2D
SELF_TEST=0x2E
SYNC=0x2B
RANGE=0x2C
 #Data range
RANGE_2G =0x00
RANGE_4G=0x01
RANGE_8G=0x03
# Values
WRITE_BIT = 0x00
READ_BIT = 0x01
DUMMY_BYTE = 0xAA
MEASURE_MODE = 0x06

class ADXL362 :

    def __init__(self, measure_range=RANGE_2G):

    # Initialize SPI
        self.spi = spidev.SpiDev()
        self.spi.open(SPI_BUS, SPI_DEVICE)
        self.spi.max_speed_hz = SPI_MAX_CLOCK_HZ
        self.spi.mode = SPI_MODE
    # Initialize sensor
        self._set_measure_range(measure_range)
        self._enable_measure_mode()
        
    def write_data(self, address, value):
 
        device_address = address << 1 | WRITE_BIT
        self.spi.xfer2([device_address, value])
        
    def read_data(self, address):
 
        device_address = address << 1 | READ_BIT
        return self.spi.xfer2([device_address, DUMMY_BYTE])[1]
        
    def _set_measure_range(self, measure_range):

    # Write data
        self.write_data(RANGE, measure_range)
        
    def get_measure_range(self):
 
    # Read data
        raw_data = self.read_data(RANGE)
    # Split data
        measure_range = (raw_data - ((raw_data >> 2) << 2))
    # Return values
        return measure_range
        
    def _enable_measure_mode(self):
 
    # Write data
        self.write_data(POWER_CTL, MEASURE_MODE)
        
    def set_measure_mode(self, drdy_off, temp_off, standby):

    # Read register before modifying it
        data = self.read_data(POWER_CTL)
    # Preserve reserved data, discard the rest
        data = ((data >> 3) << 3)
    # Add measure mode
        data = data + (drdy_off << 2) + (temp_off << 1) + standby
    # Write data
        self.write_data(POWER_CTL, data)
        
        
    def get_axes(self):
 
    # Reading data
        x_data = [self.read_data(XDATA_L), self.read_data(XDATA_H)]
        y_data = [self.read_data(YDATA_L), self.read_data(YDATA_H)]
        z_data = [self.read_data(ZDATA_L), self.read_data(ZDATAH)]
    # Join data
        x_data = (x_data[0] >> 4) + (x_data[1] << 4) \
        + (x_data[2] << 11)
        y_data = (y_data[0] >> 4) + (y_data[1] << 4) \
        + (y_data[2] << 11)
        z_data = (z_data[0] >> 4) + (z_data[1] << 4) \
        + (z_data[2] << 11)
    # Apply two complement
        x_data = twos_comp(x_data, 20)
        y_data = twos_comp(y_data, 20)
        z_data = twos_comp(z_data, 20)
    # Return values
        return [x_data, y_data, z_data]
        
        
    def get_temperature(self):
 
    # Reading data
        temp_data = [self.read_data(TEMP_L), self.read_data(TEMP_H)]
    # Join data
        temp_data = (temp_data[0]) + ((temp_data[1] \
        - ((temp_data[1] >> 4) << 4)) << 8)
    # Return values
        return temp_data
        
        
    def get_axes_and_temp(self):
 
    # Reading data
        x_data = [self.read_data(XDATA_L), self.read_data(XDATA_H)]
        y_data = [self.read_data(YDATA_L), self.read_data(YDATA_H)]
        z_data = [self.read_data(ZDATA_L), self.read_data(ZDATA_H)]
        temp_data = [self.read_data(TEMP_L), self.read_data(TEMP_H)]
    # Join data
        x_data = (x_data[0] >> 4) + (x_data[1] << 4)
        y_data = (y_data[0] >> 4) + (y_data[1] << 4)
        z_data = (z_data[0] >> 4) + (z_data[1] << 4)
        temp_data = ((temp_data[1] - ((temp_data[1] >> 4) << 4)) \
        << 8) + (temp_data[0] << 0)
    # Apply two complement
        if (x_data & (1 << 19)) != 0:
            x_data = x_data - (1 << 20)
        if (y_data & (1 << 19)) != 0:
            y_data = y_data - (1 << 20)
        if (z_data & (1 << 19)) != 0:
            z_data = z_data - (1 << 20)
    # Return values
        return [-x_data, -y_data, -z_data, temp_data]
        
        
    def set_ODR_and_filter(self, odr_lpf, hpf_filter):

    # Read register before modifying
        data = self.read_data(FILTER_CTL)
    # Preserve reserved data
        data = ((data >> 7) << 7)
    # Join data
        data = data + (hpf_filter << 4) + odr_lpf
    # Write data
        self.write_data(FILTER_CTL, data)
        
        
    def get_ODR_and_filter(self):

    # Read data
        raw_data = self.read_data(FILTER_CTL)
        odr_lpf = raw_data - ((raw_data >> 4) << 4)
        hpf_filter = (raw_data >> 4)
        hpf_filter = hpf_filter - ((hpf_filter >> 3) << 3)
    # Return values
        return [odr_lpf, hpf_filter]
        
        
    def set_sync(self, ext_clk, ext_sync):
   
    # Read data on register before modifiying
        data = self.read_data(SYNC)
    # Preserve reserved bits
        data = ((data >> 3) << 3)
    # Join data, not modifying reserved data
        data = (data) + (ext_clk << 2) + ext_sync
    # Write data
        self.write_data(SYNC, data)
        
        
    def get_sync(self):
 
    # Read data
        raw_data = self.read_data(SYNC)
    # Split bits
        ext_sync = raw_data - ((raw_data >> 2) << 2)
        ext_clk = (raw_data >> 2)
        ext_clk = ext_clk - ((ext_clk >> 1) << 1)
    # Return values
        return [ext_clk, ext_sync]
        
        
    def get_status(self):
 
    # Read data
        raw_data = self.read_data(STATUS)
    # Split bits
        data_rdy = raw_data - ((raw_data >> 1) << 1)
        raw_data = (raw_data >> 1)
        fifo_full = raw_data - ((raw_data >> 1) << 1)
        raw_data = (raw_data >> 1)
        fifo_ovr = raw_data - ((raw_data >> 1) << 1)
        raw_data = (raw_data >> 1)
        activity = raw_data - ((raw_data >> 1) << 1)
        raw_data = (raw_data >> 1)
        nvm_busy = raw_data - ((raw_data >> 1) << 1)
    # Return values
        return [data_rdy, fifo_full, fifo_ovr, activity, \
        nvm_busy]
        
    
                
    def get_offsets(self):
        def twos_comp(val,bits=8):
            if (val & (1<<(bits-1)))!=0:
                val=val-(1<<bits)
            return (val)
        x_data=[(self.read_data(XDATA_L)-self.read_data(XDATA)) ,(self.read_data(XDATA_H)-self.read_data(XDATA))]
        y_data=[(self.read_data(YDATA_L)-self.read_data(YDATA)) ,(self.read_data(YDATA_H)-self.read_data(YDATA))]
        z_data=[(self.read_data(ZDATA_L)-self.read_data(ZDATA)) ,(self.read_data(ZDATA_H)-self.read_data(ZDATA))]
        x_data=x_data[1]+(x_data[0]<<8)
        y_data=y_data[1]+(y_data[0]<<8)
        z_data=z_data[1]+(z_data[0]<<8)
        x_data=(x_data<<4)
        y_data=(y_data<<4)
        z_data=(z_data<<4)
        x_data=twos_comp(x_data,20)
        y_data=twos_comp(y_data,20)
        z_data=twos_comp(z_data,20)
        return ([x_data,y_data,z_data])
        
        
        
    def reset_settings(self):
  
    # Reset command
        self.write_data(SOFT_RESET, 0x52)
