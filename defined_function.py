def bits2range(bits):


# Interpret range
    if bits == 0x00:
        meas_range = '2g'
        scale = 256000 #LSB/g
    elif bits == 0x01:
        meas_range = '4g'
        scale = 128000 #LSB/g
    else:
        meas_range = '8g'
        scale = 64000 #LSB/g
# Return value
    return [meas_range, scale]
    
    
def bits2odr_filter(bits):

# Interpret ODR and LPF
    if bits[0] == 0x00:
        odr = '4000 Hz'
        lpf = '1000 Hz'
    elif bits[0] == 0x01:
        odr = '2000 Hz'
        lpf = '500 Hz'
    elif bits[0] == 0x02:
        odr = '1000 Hz'
        lpf = '250 Hz'
    elif bits[0] == 0x03:
        odr = '500 Hz'
        lpf = '125 Hz'
    elif bits[0] == 0x04:
        odr = '250 Hz'
        lpf = '62.5 Hz'
    elif bits[0] == 0x05:
        odr = '125 Hz'
        lpf = '31.25 Hz'
    elif bits[0] == 0x06:
        odr = '62.5 Hz'
        lpf = '15.625 Hz'
    elif bits[0] == 0x07:
        odr = '31.25 Hz'
        lpf = '7.813 Hz'
    elif bits[0] == 0x08:
        odr = '15.625 Hz'
        lpf = '3.906 Hz'
    elif bits[0] == 0x09:
        odr = '7.813 Hz'
        lpf = '1.953 Hz'
    else:
        odr = '3.906 Hz'
        lpf = '0.977 Hz'
# Interpret HPF
    if bits[1] == 0x00:
        hpf = 'No high pass filter'
    elif bits[1] == 0x01:
        hpf = '247e-3 x ODR'
    elif bits[1] == 0x01:
        hpf = '62.084e-3 x ODR'
    elif bits[1] == 0x03:
        hpf = '15.545e-3 x ODR'
    elif bits[1] == 0x04:
        hpf = '3.862e-3 x ODR'
    elif bits[1] == 0x05:
        hpf = '0.954e-3 x ODR'
    else:
        hpf = '0.238e-3 x ODR'
# Return values
    return [odr, lpf, hpf]
    
    
def bits2sync(bits):

# Interpret external clock
    if bits[0] == 0x00:
        ext_clk = 'External clock disabled'
    else:
        ext_clk = 'External clock enabled'
# Interpret external sync control
    if bits[1] == 0x00:
        ext_sync = 'Internal sync'
    elif bits[1] == 0x01:
        ext_sync = 'External sync, no interpolation filter'
    else:
        ext_sync = 'External sync, interpolation filter'
# Return value
    return [ext_clk, ext_sync]
    
    
def bits2accel(bits, scale, offset):

    x_data = bits[0] / scale[0] - offset[0]
    y_data = bits[1] / scale[1] - offset[1]
    z_data = bits[2] / scale[2] - offset[2]
    return [x_data, y_data, z_data]
    
    
def bits2temp(bits, scale, offset):

    temperature = ((bits - offset) / scale) - 10.0
    return (temperature -173)
    
    
def bits2accel_temp(bits_accel, scale_accel, offset_accel, bits_temp,scale_temp,offset_temp):

    accel = bits2accel(bits_accel, scale_accel, offset_accel)