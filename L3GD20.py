#!/usr/bin/python
import smbus 
import time

#initialize gyro variables
x_data_g = 0
y_data_g = 0
z_data_g = 0

#set the I2C ports (0x6b = gyrometer)
device_g = 0x6b

bus=smbus.SMBus(1)

#set register values

#ctrl_reg1_g
dr_ctrl_reg1_g    = 0b00   #ODR 95 Hz
bw_ctrl_reg1_g    = 0b00   #Cut-Off 12.5
pd_ctrl_reg1_g    = 0b0    #power-down mode
zen_ctrl_reg1_g   = 0b1    #z axis enabled
xen_ctrl_reg1_g   = 0b1    #x axis enabled
yen_ctrl_reg1_g   = 0b1    #y axis enabled
value_ctrl_reg1_g = (dr_ctrl_reg1_g << 6) + (bw_ctrl_reg1_g << 4) + (pd_ctrl_reg1_g << 3) + (zen_ctrl_reg1_g << 2) + (xen_ctrl_reg1_g << 1) + yen_ctrl_reg1_g
#ctrl_reg2_g
hpm_ctrl_reg2_g   = 0b00   #High pass filter mode normal
hpc_ctrl_reg2_g   = 0b0000 #ODR 95Hz / cut-off 7.2, ODR 190Hz / cut-off 13.5, ODR 380Hz / cut-off 27, ODR 760Hz / cut-off 51.4
value_ctrl_reg2_g = (hpm_ctrl_reg2_g << 4) + hpc_ctrl_reg2_g
#ctrl_reg3_g
i1_ctrl_reg3_g    = 0b00   #Interrupt disable on INT1 pin, Boot status available on INT1 disabled
h_ctrl_reg3_g     = 0b0    #Interrupt active configuration on INT1 high
pp_ctrl_reg3_g    = 0b0    #push-pull
i2_ctrl_reg3_g    = 0b0000 #Date-ready on DRDYANT2 disabled, FIFO watermarking interrupt on DRDYANT2 disabled, FIFO overrun interrupt on DRDYANT2 disabled, FIFO empty interrupt on DRDYANT2 disabled
value_ctrl_reg3_g = (i1_ctrl_reg3_g << 6) + (h_ctrl_reg3_g << 5) + (pp_ctrl_reg3_g << 4) + i2_ctrl_reg3_g
#ctrl_reg4_g
bdu_ctrl_reg4_g   = 0b0    #Block data update continuos
ble_ctrl_reg4_g   = 0b0    #Big/little endian data selection data lsb @ low address
fs_ctrl_reg4_g    = 0b00   #full scale selection 250 dps
sim_ctrl_reg4_g   = 0b0    #SPI serial interface mode selection 4-wire interface
value_ctrl_reg4_g = (bdu_ctrl_reg4_g << 7) + (ble_ctrl_reg4_g << 6) + (fs_ctrl_reg4_g << 4) + sim_ctrl_reg4_g
#ctrl_reg5_g
boot_ctrl_reg5_g  = 0b0    #Reboot memory content normal mode
fifo_ctrl_reg5_g  = 0b0    #FIFO disabled
hpen_ctrl_reg5_g  = 0b0    #High-pass filter disabled
int1_ctrl_reg5_g  = 0b00   #INT1 selection configuration wait disabled
out_ctrl_reg5_g   = 0b00   #OUT selection configuration wait disabled
value_ctrl_reg5_g = (boot_ctrl_reg5_g << 7) + (fifo_ctrl_reg5_g << 6) + (hpen_ctrl_reg5_g << 4) + (int1_ctrl_reg5_g << 2) + out_ctrl_reg5_g

who_am_i_g      = 0x0f #r   0001111  Device identification register
ctrl_reg1_g     = 0x20 #rw  0100000  Control register 1
ctrl_reg2_g     = 0x21 #rw  0100001  Control register 2
ctrl_reg3_g     = 0x22 #rw  0100010  Control register 3
ctrl_reg4_g     = 0x23 #rw  0100011  Control register 4
ctrl_reg5_g     = 0x24 #rw  0100100  Control register 5
reference_g     = 0x25 #rw  0100101  Reference value for interrupt generation
out_temp_g      = 0x26 #r   0100110  Output temperature
status_reg_g    = 0x27 #r   0100111  Status register
out_x_l_g       = 0x28 #r   0101000  X-axis angular data rate LSB
out_x_h_g       = 0x29 #r   0101001  X-axis angular data rate MSB
out_y_l_g       = 0x2a #r   0101010  Y-axis angular data rate LSB
out_y_h_g       = 0x2b #r   0101011  Y-axis angular data rate MSB
out_z_l_g       = 0x2c #r   0101100  Z-axis angular data rate LSB
out_z_h_g       = 0x2d #r   0101101  Z-axis angular data rate MSB
fifo_ctrl_reg_g = 0x2e #rw  0101110  Fifo control register
fifo_src_reg_g  = 0x2f #r   0101111  Fifo src register
int1_cfg_g      = 0x30 #rw  0110000  Interrupt 1 configuration register
int1_src_g      = 0x31 #r   0110001  Interrupt source register
int1_ths_xh_g   = 0x32 #rw  0110010  Interrupt 1 threshold level X MSB register
int1_ths_xl_g   = 0x33 #rw  0110011  Interrupt 1 threshold level X LSB register
int1_ths_yh_g   = 0x34 #rw  0110100  Interrupt 1 threshold level Y MSB register
int1_ths_yl_g   = 0x35 #rw  0110101  Interrupt 1 threshold level Y LSB register
int1_ths_zh_g   = 0x36 #rw  0110110  Interrupt 1 threshold level Z MSB register
int1_ths_zl_g   = 0x37 #rw  0110111  Interrupt 1 threshold level Z LSB register
int1_duration_g = 0x38 #rw  0111000  Interrupt 1 duration register

#Enable the Gyrometer

#CTRL_REG1_G register
#DR1 DR0 BW1 BW0 PD Zen Xen Yen
#CTRL_REG1_G description
#DR1-DR0 = Output data rate selection (refer to DR and BW configuration setting)
#BW1-BW0 = Bandwidth selection (refer to DR and BW configuration setting)
#PD = Power-down mode enable. Devault value: 0 (0: power-down mode, 1: normal mode or sleep mode) (refer to power mode selection configuration)
#Zen = Z axis enable, Default value: 1 (0: Z axis disabled, 1: Z axis enabled)
#Yen = Y axis enable, Default value: 1 (0: Y axis disabled, 1: Y axis enabled)
#Xen = X axis enable, Default value: 1 (0: X axis disabled, 1: X axis enabled)
#DR and BW configuration setting:
#0000 = ODR 95 Hz, Cut-Off 12.5
#0001 = ODR 95 Hz, Cut-Off 25
#0010 = ODR 95 Hz, Cut-Off 25
#0011 = ODR 95 Hz, Cut-Off 25
#0100 = ODR 190 Hz, Cut-Off 12.5
#0101 = ODR 190 Hz, Cut-Off 25
#0110 = ODR 190 Hz, Cut-Off 50
#0111 = ODR 190 Hz, Cut-Off 70
#1000 = ODR 380 Hz, Cut-Off 20
#1001 = ODR 380 Hz, Cut-Off 25
#1010 = ODR 380 Hz, Cut-Off 50
#1011 = ODR 380 Hz, Cut-Off 100
#1100 = ODR 760 Hz, Cut-Off 30
#1101 = ODR 760 Hz, Cut-Off 35
#1110 = ODR 760 Hz, Cut-Off 50
#1111 = ODR 760 Hz, Cut-Off 100
#power mode selection configuration:
#PD Zen Yen Xen Mode
#0  -   -   -   power-down
#1  0   0   0   sleep
#1  -   -   -   normal 
bus.write_byte_data (device_g,ctrl_reg1_g,value_ctrl_reg1_g)
time.sleep (0.01)

#CTRL_REG2_G register
#0(1) 0(1) HPM1 HPM0 HPCF3 HPCF2 HPCF1 HPCF0
#(1) These bits must be set to '0' to ensure proper operation of the device
#CTRL_REG2_G description
#HPM1-HPM0 = High-pass filter mode selection. Default value: 00 (refer to high-pass filter mode configuration)
#HPCF3-HPCF0 = High-pass filter cutoff frequency selection (refer to high-pass filter cut off frequency configuration (Hz)
#high-pass filter mode configuration
#HPM1 HPM0 High-pass filter mode
#0    0    Normal mode (reset reading HP_RESET_FILTER)
#0    1    Reference signal for filtering
#1    0    Normal mode
#1    1    Autoreset on interrupt event
#high-pass filter cut off frequency configuration
#HPCF3-0 ODR=95 Hz ODR=190 Hz ODR=380 Hz ODR=760 Hz
#0000    7.2       13.5       27         51.4
#0001    3.5        7.2       13.5       27
#0010    1.8        3.5        7.2       13.5
#0011    0.9        1.8        3.5        7.2
#0100    0.45       0.9        1.8        3.5
#0101    0.18       0.45       0.9        1.8
#0110    0.09       0.18       0.45       0.9
#0111    0.045      0.09       0.18       0.45
#1000    0.018      0.045      0.09       0.18
#1001    0.009      0.018      0.045      0.09
bus.write_byte_data (device_g,ctrl_reg2_g,value_ctrl_reg2_g)
time.sleep (0.01)
    
#CTRL_REG3_G register
#I1_Int1 I1_Boot H_Lactive PP_OD I2_DRDY I2_WTM I2_ORun I2_Empty
#CTRL_REG3_G description
#I1_Int1 = Interrupt enable on INT1 pin. Default value 0. (0: disable; 1: enable)
#I1_Boot = Boot status available on INT1. Default value 0. (0: disable; 1: enable)
#H_Lactive = Interrupt active configuration on INT1. Default value 0. (0: high; 1:low)
#PP_OD = Push-pull / Open drain. Default value: 0. (0: push- pull; 1: open drain)
#I2_DRDY = Date-ready on DRDY/INT2. Default value 0. (0: disable; 1: enable)
#I2_WTM = FIFO watermark interrupt on DRDY/INT2. Default value: 0. (0: disable; 1: enable)
#I2_ORun = FIFO overrun interrupt on DRDY/INT2 Default value: 0. (0: disable; 1: enable)
#I2_Empty = FIFO empty interrupt on DRDY/INT2. Default value: 0. (0: disable; 1: enable)
bus.write_byte_data (device_g,ctrl_reg3_g,value_ctrl_reg3_g)
time.sleep (0.01)

#CTRL_REG4_G register
#BDU BLE FS1 FS0 - 0(1) 0(1) SIM
#(1) This value must not be changed.
#CTRL_REG4_G description
#BDU = Block data update. Default value: 0 (0: continuos update; 1: output registers not updated until MSb and LSb read-ing)
#BLE = Big/little endian data selection. Default value 0.(0: Data LSb @ lower address; 1: Data MSb @ lower address)
#FS1-FS0 =  Full scale selection. Default value: 00 (00: 250 dps; 01: 500 dps; 10: 2000 dps; 11: 2000 dps)
#SIM = SPI serial interface mode selection. Default value: 0 (0: 4-wire interface; 1: 3-wire interface).
bus.write_byte_data (device_g,ctrl_reg4_g,value_ctrl_reg4_g)
time.sleep (0.01)

#CTRL_REG5_G register
#BOOT FIFO_EN - HPen INT1_Sel1 INT1_Sel0 Out_Sel1 Out_Sel0
#CTRL_REG5_G description
#BOOT = Reboot memory content. Default value: 0 (0: normal mode; 1: reboot memory content)
#FIFO_EN = FIFO enable. Default value: 0 (0: FIFO disable; 1: FIFO Enable)
#HPen = High-pass filter enable. Default value: 0 (0: HPF disabled; 1: HPF enabled See Figure 20)
#INT1_Sel1-INT1_-Sel0 = INT1 selection configuration. Default value: 0 (0: wait disabled, 1: wait enabled)
#Out_Sel1-Out_-Sel1 = Out selection configuration. Default value: 0 (0: wait disabled, 1: wait enabled)
bus.write_byte_data (device_g,ctrl_reg5_g,value_ctrl_reg5_g)
time.sleep (0.01)

def readGyro(*offset):

  #read gyro values
  try:
    val_out_x_h_g = bus.read_byte_data (device_g,out_x_h_g)
    val_out_x_l_g = bus.read_byte_data (device_g,out_x_l_g)
    val_out_z_h_g = bus.read_byte_data (device_g,out_z_h_g)
    val_out_z_l_g = bus.read_byte_data (device_g,out_z_l_g)
    val_out_y_h_g = bus.read_byte_data (device_g,out_y_h_g)
    val_out_y_l_g = bus.read_byte_data (device_g,out_y_l_g)
  except IOError:
    subprocess.call(['i2cdetect', '-y', '1'])
    flag = 1     #optional flag to signal your code to resend or something

  #gyro bite slide
  x_data_g = ((val_out_x_h_g << 8) + val_out_x_l_g)
  y_data_g = ((val_out_y_h_g << 8) + val_out_y_l_g)
  z_data_g = ((val_out_z_h_g << 8) + val_out_z_l_g)

  #calculate two's complement
  y_data_g = twos_comp(x_data_g, 16)
  y_data_g = twos_comp(y_data_g, 16)
  z_data_g = twos_comp(z_data_g, 16)

  #remove the noise
  x_data_g = x_data_g - offset[0]
  y_data_g = y_data_g - offset[1]
  z_data_g = z_data_g - offset[2]

  return [(x_data_g),(y_data_g),(z_data_g)]

def readDPS(*offset):

  #read the gyro values
  xyz_data_g = readGyro(*offset)
  gyroFactor = setGyroScale(fs_ctrl_reg4_g)

  #convert values into degrees per second
  value_dps_x_g = xyz_data_g[0] * gyroFactor
  value_dps_y_g = xyz_data_g[1] * gyroFactor
  value_dps_z_g = xyz_data_g[2] * gyroFactor
 
  return [(value_dps_x_g),(value_dps_y_g),(value_dps_z_g)]

def setGyroScale(scale):
  if scale == 0b00: #250 dps
    gyroFactor = 0.00875
  elif scale == 0b01: #500 dps
    gyroFactor = 0.01750
  elif scale == 0b10: #2000 dps
    gyroFactor = 0.070

  return gyroFactor

def twos_comp(val, bits):
  #compute the 2's compliment of int value val
  if((val&(1<<(bits-1))) != 0):
    val = val - (1<<bits)

  return val

def calibrateGyro():

  x_value_g = 0
  y_value_g = 0
  z_value_g = 0
  x_offset_g = 0
  y_offset_g = 0
  z_offset_g = 0

  print "Calibrating Gyrometer, please do not move sensor..."
  countdown(15) #delay for 15 seconds
  print "Stop moving now..."

  for i in range (1,1000):

    #read the gyro values
    x_value_g = readGyro(0,0,0)[0]
    y_value_g = readGyro(0,0,0)[1]
    z_value_g = readGyro(0,0,0)[2]

    #sum up the readings
    x_offset_g += x_value_g
    y_offset_g += y_value_g
    z_offset_g += z_value_g

  #average the readings
  x_offset_g = x_offset_g / 1000
  y_offset_g = y_offset_g / 1000
  z_offset_g = z_offset_g / 1000

  print "Gyrometer offset values are: '{0}', '{1}', '{2}'".format(x_offset_g, y_offset_g, z_offset_g)

  return [(x_offset_g),(y_offset_g),(z_offset_g)]

def countdown(n):

  while n >= 0:
    time.sleep(1)
    print (n)
    n -= 1



  
