#!/usr/bin/python
import os
import sys
import datetime
import commands
import math
from string import split
from smbus import SMBus
from time import sleep
#class for BMP085 sensor breakout
import Adafruit_BMP.BMP085 as BMP085
#class for GPS breakout
from GPS_Controller import GpsController
#class for MinIMU-9 V2 (L3GD20, GYRO)
import L3GD20
#class for MinIMU-9 V2 (LSM303DLHC, Accelerometer and Magnetometer)
import LSM303DLHC
#class for MySQL Database
import MySQLdb
#suppress warning messages (as timeUtc and timeFix will get truncated or will get out of range)
from warnings import filterwarnings
filterwarnings('ignore', category = MySQLdb.Warning)

#set devide ID
deviceID = 1

# Default constructor will pick a default I2C bus.
#
# For the Raspberry Pi this means you should hook up to the only exposed I2C bus
# from the main GPIO header and the library will figure out the bus number based
# on the Pi's revision.
#
# For the Beaglebone Black the library will assume bus 1 by default, which is
# exposed with SCL = P9_19 and SDA = P9_20.
bmp085 = BMP085.BMP085()

# Optionally you can override the bus number:
#sensor = BMP085.BMP085(busnum=2)

# You can also optionally change the BMP085 mode to one of BMP085_ULTRALOWPOWER,
# BMP085_STANDARD, BMP085_HIGHRES, or BMP085_ULTRAHIGHRES.  See the BMP085
# datasheet for more details on the meanings of each mode (accuracy and power
# consumption are primarily the differences).  The default mode is STANDARD.
bmp085 = BMP085.BMP085(mode=BMP085.BMP085_ULTRAHIGHRES) 

#initilise the humidity & temperature sensor (HIH-6130) - uncomment appropriate line depending on Pi version used
#if using version 2 Pi (512 M) then use i2c bus is #1
#HIH6130 = SMBus(0)  # 0 indicates /dev/i2c-0
HIH6130 = SMBus(1)  # 1 indicates /dev/i2c-1

#enable the GPS socket
os.system('sudo gpsd /dev/ttyAMA0 -F /var/run/gpsd.sock')
sleep(0.5)
#create the gps controller object
gpsc = GpsController()
#start gps controller
gpsc.start()

#access the MySQL databse
try:
  #open database connection
  db = MySQLdb.connect("localhost","root","s@O/Ov2Q4@6wE4ASG8Yi","hab")
  #prepare a cursor object using cursor() method
  cursor = db.cursor()
  print 'Database connected...'
except:
  #rollback in case there is any error
  MySQLdb.connect("localhost","root","s@O/Ov2Q4@6wE4ASG8Yi","hab").rollback()

#initialize calibration offsets
xyz_offset_g = 0
xyz_offset_m = 0

def readOnboardTemp():
  #read onboard CPU temperature sensor data
  tempRC = int(open('/sys/class/thermal/thermal_zone0/temp').read()) / 1e3 #in celsius
  tempRF = int(open('/sys/class/thermal/thermal_zone0/temp').read()) / 1e3 * 1.8 + 32.0 #in fahrenheit

  return {'tempRC':tempRC, 'tempRF':tempRF}

def readBMP085():
  #fetch temp & pressure data from BMP085
  temp1OffSet = -2
  altitude1OffSet = 21.985
  pressure = bmp085.read_pressure() / 100.0
  sealvlPressure = bmp085.read_sealevel_pressure() / 100.0
  altitude1 = bmp085.read_altitude() + altitude1OffSet
  temp1C = bmp085.read_temperature() + temp1OffSet #in Celsius
  temp1F = (bmp085.read_temperature() + temp1OffSet) * 1.8 + 32.0 #in Fahrenheit

  return {'pressure':pressure, 'sealvlPressure':sealvlPressure, 'altitude1':altitude1, 'temp1C':temp1C, 'temp1F':temp1F}

def readHIH6130():
  #temporary storage array for HIH6130 data
  var = [0, 0, 0, 0]

  #fetch temp and humidity data from HIH-6130
  humidityOffSet = 6.5
  HIH6130.write_quick(0x27)
  sleep(0.050)
  var = HIH6130.read_i2c_block_data(0x27, 0)
  humidity = ((((var[0] & 0x3f) << 8) + var[1]) * 100.0 / 16383.0) + humidityOffSet

  return {'humidity':humidity}

def readGPS():
  #fetch GPS data from Ultimate GPS Breakout
  latitude = gpsc.fix.latitude
  longitude = gpsc.fix.longitude
  timeUtc = gpsc.utc
  timeFix = ''
  if isinstance(gpsc.fix.time, str):
    timeFix = gpsc.fix.time
  altitude2 = gpsc.fix.altitude
  eps = gpsc.fix.eps #speed error estimate in meter/sec			
  epx = gpsc.fix.epx #estimated Longitude error in meters
  epv = gpsc.fix.epv #estimated vertical error in meters
  ept = gpsc.gpsd.fix.ept #estimated timestamp error
  speed = gpsc.fix.speed
  climb = gpsc.fix.climb #climb (positive) or sink (negative rate, meters per second
  track = gpsc.fix.track #course over ground, degrees from true north
  mode = gpsc.fix.mode #NMEA mode: %d, 0=no mode value yet seen, 1=no fix, 2=2D, 3=3D
  satellites = len(gpsc.satellites)
		
  #make sure that the database tables get no input for as long as there is no GPS signal
  if (math.isnan(latitude) or not latitude):
    gpsSignal = 0
  elif (math.isnan(longitude) or not longitude):
    gpsSignal = 0
  elif (timeUtc is None or "nan" in timeUtc or not timeUtc):
    gpsSignal = 0
  elif (math.isnan(altitude2)):
    gpsSignal = 0
  elif (math.isnan(eps)):
    gpsSignal = 0
  elif (math.isnan(epx)):
    gpsSignal = 0
  elif (math.isnan(epv)):
    gpsSignal = 0
  elif (math.isnan(ept)):
    gpsSignal = 0
  elif (math.isnan(speed)):
    gpsSignal = 0
  elif (math.isnan(climb)):
    gpsSignal = 0
  elif (math.isnan(track)):
    gpsSignal = 0
  else:
    gpsSignal = 1

  return {'gpsSignal':gpsSignal, 'latitude':latitude, 'longitude':longitude, 'timeUtc':timeUtc, 'timeFix':timeFix, 'altitude2':altitude2, 'eps':eps, 'epx':epx, 'epv':epv, 'ept':ept, 'speed':speed, 'climb':climb, 'track':track, 'mode':mode, 'satellites':satellites}

def readMinIMU9V2():

  global xyz_offset_g
  global xyz_offset_m

  magnetics = LSM303DLHC.readMagnetics(*xyz_offset_m)
  magneticGauss = LSM303DLHC.readMagneticGauss(*xyz_offset_m)
  magneticHeading = LSM303DLHC.readMagneticHeading(*xyz_offset_m)
  magneticTemp = LSM303DLHC.readTemperatures()
  accelerations = LSM303DLHC.readAccelerations()
  accelerationG = LSM303DLHC.readAccelerationG()
  gyro = L3GD20.readGyro(*xyz_offset_g)
  gyroDPS = L3GD20.readDPS(*xyz_offset_g)

  heading = magneticHeading[0]
  windDir = magneticHeading[1]
  temp2C = magneticTemp[0]
  temp2F = magneticTemp[1]
  magX = magnetics[0]
  magY = magnetics[1]
  magZ = magnetics[2]
  magGaussX = magneticGauss[0]
  magGaussY = magneticGauss[1]
  magGaussZ = magneticGauss[2]
  accX = accelerations[0]
  accY = accelerations[1]
  accZ = accelerations[2]
  accGX = accelerationG[0]
  accGY = accelerationG[1]
  accGZ = accelerationG[2]
  gyroX = gyro[0]
  gyroY = gyro[1]
  gyroZ = gyro[2]
  gyroDPSX = gyroDPS[0]
  gyroDPSY = gyroDPS[1]
  gyroDPSZ = gyroDPS[2]

  return {'heading':heading, 'windDir':windDir, 'temp2C':temp2C, 'temp2F':temp2F, 'magX':magX, 'magY':magY, 'magZ':magZ, 'magGaussX':magGaussX, 'magGaussY':magGaussY, 'magGaussZ':magGaussZ, 'accX':accX, 'accY':accY, 'accZ':accZ, 'accGX':accGX, 'accGY':accGY, 'accGZ':accGZ, 'gyroX':gyroX, 'gyroY':gyroY, 'gyroZ':gyroZ, 'gyroDPSX':gyroDPSX, 'gyroDPSY':gyroDPSY, 'gyroDPSZ':gyroDPSZ}

def printValues(sysDate, sysTime, pressure, sealvlPressure, altitude1, temp1C, temp1F, latitude, longitude, timeUtc, timeFix, altitude2, eps, epx, epv, ept, speed, climb, track, mode, satellites, humidity, heading, windDir, temp2C, temp2F, magX, magY, magZ, magGaussX, magGaussY, magGaussZ, accX, accY, accZ, accGX, accGY, accGZ, gyroX, gyroY, gyroZ, gyroDPSX, gyroDPSY, gyroDPSZ, tempRC, tempRF):

  #print data to screen
  print "Date:           {0}".format(sysDate)
  print "Time:           {0}".format(sysTime)
  print "Pressure:       {0} hPa".format(pressure)
  print "SeaLvlPr:       {0} hPa".format(sealvlPressure)
  print "Altitude:       {0} m".format(altitude1)
  print "Temp1B:         {0} C".format(temp1C)
  print "Temp1B:         {0} F".format(temp1F)
  print "Latitude:       {0}".format(latitude)
  print "Longitude:      {0}".format(longitude)
  print "Time-Utc:       {0}".format(timeUtc)
  print "Time-Fix:       {0}".format(timeFix)
  print "Altitude-GPS:   {0}".format(altitude2)
  print "EPS:            {0}".format(eps)
  print "EPX:            {0}".format(epx)
  print "EPV:            {0}".format(epv)
  print "EPT:            {0}".format(ept)
  print "Speed:          {0} m/s".format(speed)
  print "Climb:          {0}".format(climb)
  print "Track:          {0}".format(track)
  print "Mode:           {0}".format(mode)
  print "Satellites:     {0}".format(satellites)
  print "Humidity:       {0} RH%'".format(humidity) #RH = Relative Humidity
  print "Heading:        {0} deg.".format(heading)
  print "WindDirection:  {0}".format(windDir)
  print "Temp2M:         {0} C".format(temp2C)
  print "Temp2M:         {0} F".format(temp2F)
  print "MagX:           {0}".format(magX)
  print "MagY:           {0}".format(magY)
  print "MagZ:           {0}".format(magZ)
  print "MagGaussX:      {0} Gauss".format(magGaussX)
  print "MagGaussY:      {0} Gauss".format(magGaussY)
  print "MagGaussZ:      {0} Gauss".format(magGaussZ)
  print "AccX:           {0}".format(accX)
  print "AccY:           {0}".format(accY)
  print "AccZ:           {0}".format(accZ)
  print "AccGX:          {0} G".format(accGX)
  print "AccGY:          {0} G".format(accGY)
  print "AccGZ:          {0} G".format(accGZ)
  print "GyroX:          {0}".format(gyroX)
  print "GyroY:          {0}".format(gyroY)
  print "GyroZ:          {0}".format(gyroZ)
  print "GyroDPSX:       {0}' DPS".format(gyroDPSX)
  print "GyroDPSY:       {0}' DPS".format(gyroDPSY)
  print "GyroDPSZ:       {0}' DPS".format(gyroDPSZ)
  print "TempR:          {0}' C".format(tempRC)
  print "TempR:          {0}' F".format(tempRF)

def writeSQL(deviceId, sysDate, sysTime, pressure, sealvlPressure, altitude1, temp1C, temp1F, latitude, longitude, timeUtc, timeFix, altitude2, eps, epx, epv, ept, speed, climb, track, mode, satellites, humidity, heading, windDir, temp2C, temp2F, magX, magY, magZ, magGaussX, magGaussY, magGaussZ, accX, accY, accZ, accGX, accGY, accGZ, gyroX, gyroY, gyroZ, gyroDPSX, gyroDPSY, gyroDPSZ, tempRC, tempRF):
  #write the data to the MySQL Database
  try:
    #BMP085 sensor data
    #GPS sensor data
    #HIH6130 sensor data
    #MinIMU-9 V2 sensor data
    #Raspberry Pi Onboard sensor data
    #prepare SQL query to INSERT a record into the database.
    sql = 'INSERT INTO hab_TELEMETRICS (deviceId, date, time, bmp085_pressure, bmp085_sealvlPressure, bmp085_altitude, bmp085_tempC, bmp085_tempF, gps_latitude, gps_longitude, gps_timeUtc, gps_timeFix, gps_altitude, gps_eps, gps_epx, gps_epv, gps_ept, gps_speed, gps_climb, gps_track, gps_mode, gps_satellites, hih6130_humidity, minimu9v2_heading, minimu9v2_windDir, minimu9v2_tempCM, minimu9v2_tempFM, minimu9v2_magX, minimu9v2_magY, minimu9v2_magZ, minimu9v2_magGaussX, minimu9v2_magGaussY, minimu9v2_magGaussZ, minimu9v2_accX, minimu9v2_accY, minimu9v2_accZ, minimu9v2_accGX, minimu9v2_accGY, minimu9v2_accGZ, minimu9v2_gyroX, minimu9v2_gyroY, minimu9v2_gyroZ, minimu9v2_gyroDPSX, minimu9v2_gyroDPSY, minimu9v2_gyroDPSZ, raspi_tempC, raspi_tempF) VALUES ("%i", "%s", "%s", "%6.2f", "%6.2f", "%16.11f", "%5.2f", "%5.2f", "%12.9f", "%12.9f", "%s", "%s", "%7.2f", "%7.3f", "%7.3f", "%7.3f", "%7.3f", "%8.3f", "%8.3f", "%6.2f", "%i", "%i", "%15.11f", "%6.2f", "%s", "%5.2f", "%5.2f", "%12.6f", "%12.6f", "%12.6f", "%12.6f", "%12.6f", "%12.6f", "%12.6f", "%12.6f", "%12.6f", "%12.6f", "%12.6f", "%12.6f", "%12.6f", "%12.6f", "%12.6f", "%12.6f", "%12.6f", "%12.6f", "%5.2f", "%5.2f")' % (deviceId, sysDate, sysTime, pressure, sealvlPressure, altitude1, temp1C, temp1F, latitude, longitude, timeUtc, timeFix, altitude2, eps, epx, epv, ept, speed, climb, track, mode, satellites, humidity, heading, windDir, temp2C, temp2F, magX, magY, magZ, magGaussX, magGaussY, magGaussZ, accX, accY, accZ, accGX, accGY, accGZ, gyroX, gyroY, gyroZ, gyroDPSX, gyroDPSY, gyroDPSZ, tempRC, tempRF)
    #execute the SQL command
    cursor.execute(sql)
    #commit your changes in the database
    db.commit()

  except MySQLdb.Error, e:
   print repr(e)
   #rollback in case there is any error
   db.rollback()
  except MySQLdb.Warning, e:
   print repr(e)
   #rollback in case there is any error
   db.rollback()

def main():

  try:

    global xyz_offset_g
    global xyz_offset_m

    #calibrate the Gyrometer
    xyz_offset_g = L3GD20.calibrateGyro()

    #calibrate the Magnetometer
    xyz_offset_m = LSM303DLHC.calibrateMag()

    #continuously append data
    while(True):

      #current Date and Time
      sysDate = datetime.datetime.now().date().strftime('%Y-%m-%d')
      sysTime = datetime.datetime.now().time().strftime('%H:%m:%S')

      printValues(sysDate, sysTime, readBMP085().get('pressure'), readBMP085().get('sealvlPressure'), readBMP085().get('altitude1'), readBMP085().get('temp1C'), readBMP085().get('temp1F'), readGPS().get('latitude'), readGPS().get('longitude'), readGPS().get('timeUtc'), readGPS().get('timeFix'), readGPS().get('altitude2'), readGPS().get('eps'), readGPS().get('epx'), readGPS().get('epv'), readGPS().get('ept'), readGPS().get('speed'), readGPS().get('climb'), readGPS().get('track'), readGPS().get('mode'), readGPS().get('satellites'), readHIH6130().get('humidity'), readMinIMU9V2().get('heading'), readMinIMU9V2().get('windDir'), readMinIMU9V2().get('temp2C'), readMinIMU9V2().get('temp2F'), readMinIMU9V2().get('magX'), readMinIMU9V2().get('magY'), readMinIMU9V2().get('magZ'), readMinIMU9V2().get('magGaussX'), readMinIMU9V2().get('magGaussY'), readMinIMU9V2().get('magGaussZ'), readMinIMU9V2().get('accX'), readMinIMU9V2().get('accY'), readMinIMU9V2().get('accZ'), readMinIMU9V2().get('accGX'), readMinIMU9V2().get('accGY'), readMinIMU9V2().get('accGZ'), readMinIMU9V2().get('gyroX'), readMinIMU9V2().get('gyroY'), readMinIMU9V2().get('gyroZ'), readMinIMU9V2().get('gyroDPSX'), readMinIMU9V2().get('gyroDPSY'), readMinIMU9V2().get('gyroDPSZ'), readOnboardTemp().get('tempRC'), readOnboardTemp().get('tempRF'))

      #only write Dataset to table if the GPS signal is fully valid
      if (readGPS().get('gpsSignal') == 1):
        writeSQL(deviceID, sysDate, sysTime, readBMP085().get('pressure'), readBMP085().get('sealvlPressure'), readBMP085().get('altitude1'), readBMP085().get('temp1C'), readBMP085().get('temp1F'), readGPS().get('latitude'), readGPS().get('longitude'), readGPS().get('timeUtc'), readGPS().get('timeFix'), readGPS().get('altitude2'), readGPS().get('eps'), readGPS().get('epx'), readGPS().get('epv'), readGPS().get('ept'), readGPS().get('speed'), readGPS().get('climb'), readGPS().get('track'), readGPS().get('mode'), readGPS().get('satellites'), readHIH6130().get('humidity'), readMinIMU9V2().get('heading'), readMinIMU9V2().get('windDir'), readMinIMU9V2().get('temp2C'), readMinIMU9V2().get('temp2F'), readMinIMU9V2().get('magX'), readMinIMU9V2().get('magY'), readMinIMU9V2().get('magZ'), readMinIMU9V2().get('magGaussX'), readMinIMU9V2().get('magGaussY'), readMinIMU9V2().get('magGaussZ'), readMinIMU9V2().get('accX'), readMinIMU9V2().get('accY'), readMinIMU9V2().get('accZ'), readMinIMU9V2().get('accGX'), readMinIMU9V2().get('accGY'), readMinIMU9V2().get('accGZ'), readMinIMU9V2().get('gyroX'), readMinIMU9V2().get('gyroY'), readMinIMU9V2().get('gyroZ'), readMinIMU9V2().get('gyroDPSX'), readMinIMU9V2().get('gyroDPSY'), readMinIMU9V2().get('gyroDPSZ'), readOnboardTemp().get('tempRC'), readOnboardTemp().get('tempRF'))
      else:
        print 'No GPS signal, not writing to the database...'
      #wait 5.00 seconds before continuing
      sleep(0.01)

      #clear terminal screen
      sys.stdout.flush()

  #Ctrl C user interrupt
  except KeyboardInterrupt:
    print 'User cancelled...'
 
  #name is not defined
  except NameError as e:
    print 'Name is not defined: %s on Line number: %s' % (e.message.split("'")[1], sys.exc_traceback.tb_lineno)
    raise
  
  #type error
  except TypeError as e:
    print 'Type Error: %s on Line number: %s' % (e, sys.exc_traceback.tb_lineno)
    raise

  #attribute error
  except AttributeError as e:
    print 'Attribute Error: %s on Line number: %s' % (e, sys.exc_traceback.tb_lineno)
    raise

  #unexpected error
  except:
    print 'Unexpected error:', sys.exc_info()[0]
    raise

  finally:

    #current Date and Time
    sysDate = datetime.datetime.now().date().strftime('%Y-%m-%d')
    sysTime = datetime.datetime.now().time().strftime('%H:%m:%S')

    #write exception into database
    #prepare SQL query to INSERT a record into the database.
    sql = 'INSERT INTO hab_ERROR (deviceId, date, time, error, line) VALUES ("%i", "%s", "%s", "%s", "%i")' % (deviceID, sysDate, sysTime, sys.exc_info()[0], sys.exc_traceback.tb_lineno)
    #execute the SQL command
    cursor.execute(sql)
    #commit your changes in the database
    db.commit()
    print 'Error information written to the database...'

    #closing the database
    db.close() #close the connection
    cursor.close() #close the cursor
    print 'Database disconnected...'

    print 'Stopping sensor measurement...'
    gpsc.stopController()
    #wait for the thread to finish
    gpsc.join()

  print 'Done'

  sys.exit(0)

if __name__ == '__main__':
  main()
