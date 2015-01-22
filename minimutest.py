#!/usr/bin/python
import LSM303DLHC
import L3GD20
import time
import sys

#calibrate the Gyrometer
xyz_offset_g = L3GD20.calibrateGyro()

#calibrate the Magnetometer
xyz_offset_m = LSM303DLHC.calibrateMag()

print "XYZ_OFFSET_G: {0}".format(xyz_offset_g)
print "XYZ_OFFSET_M: {0}".format(xyz_offset_m)

while 1:

  magnetics = LSM303DLHC.readMagnetics(*xyz_offset_m)
  magneticGauss = LSM303DLHC.readMagneticGauss(*xyz_offset_m)
  magneticHeading = LSM303DLHC.readMagneticHeading(*xyz_offset_m)
  magneticTemp = LSM303DLHC.readTemperatures()
  accelerations = LSM303DLHC.readAccelerations()
  accelerationG = LSM303DLHC.readAccelerationG()
  gyro = L3GD20.readGyro(*xyz_offset_g)
  gyroDPS = L3GD20.readDPS(*xyz_offset_g)

  print "MagnetX: {0}, MagnetY: {1}, MagnetZ: {2}".format(magnetics[0], magnetics[1], magnetics[2])
  print "AccelX: {0}, AccelY: {1}, AccelZ: {2}".format(accelerations[0], accelerations[1], accelerations[2])
  print "MGaussX: {0}, MGaussY: {1}, MGaussZ: {2}".format(magneticGauss[0], magneticGauss[1], magneticGauss[2])
  print "MuTX: {0}, MuTY: {1}, MuTZ: {2}".format(magneticGauss[0] * 100, magneticGauss[1] * 100, magneticGauss[2] * 100)
  print "AgX: {0}, AgY: {1}, AgZ: {2}".format(accelerationG[0],  accelerationG[1],  accelerationG[2])
  print "MHeading: {0}".format(magneticHeading[0])
  print "MWindDir: {0}".format(magneticHeading[1])
  print "MTemp(F): {0}, MTemp(C): {1}".format(magneticTemp[0], magneticTemp[1])
  print "GyroX: {0}, GyroY: {1}, GyroZ: {2}".format(gyro[0], gyro[1], gyro[2])
  print "GyroDPSX: {0}, GyroDPSY: {1}, GyroDPSZ: {2}".format(gyroDPS[0], gyroDPS[1], gyroDPS[2])
  print "GyroRadSX: {0}, GyroRadSY: {1}, GyroRadSZ: {2}".format(gyroDPS[0] * 0.017453293, gyroDPS[1] * 0.017453293, gyroDPS[2] * 0.017453293)

  sys.stdout.flush()

  time.sleep(0.1)
