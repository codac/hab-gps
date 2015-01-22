#!/bin/bash

backupdate=$(date +%Y%m%d)
backuptime=$(date +%H%M%S)

#Create all directories of not already existend
mkdir -p /home/pi/Sensors/
mkdir -p /home/pi/Audio/
mkdir -p /home/pi/Video/
mkdir -p /home/pi/Photo/

echo "Disabling HDMI..."
#This command disables the HDMI output for power saving
#To enable HDMI output again use: tvservice -p
sudo /opt/vc/bin/tvservice -o
echo "HDMI disabled..."

echo "Starting Sensor Measuring..."
#This Python script starts the Adafruit Ultimate GPS Measuring and Logging, as well as the BMP085 and 
#It uses the Adfruit Python Script located at /home/pi/Adafriut-Raspberry-Pi-Python-Code/Adafruit_BMP085/Adafruit_BMP085.py in order to control the BMP085 sensor
#It also writes the measured data to the following MySQL tables:
#hab_BMP085 
#hab_DEVICES
#hab_ERROR
#hab_GPS
#hab_HIH6130
#hab_MINIMU9V2
#hab_RASPI
#It uses the Python Scrip located at /home/pi/missioncontrol/GPS_Controller.py in order to control the GPS module
#sudo gpsd /dev/ttyAMA0 -F /var/run/gpsd.sock & #enable the GPS socket
#echo "GPS enabled..."

#echo "Starting signal output..."
#sudo python /home/pi/missioncontrol/signals.py &

#echo "Starting sensor measurement..."
#sudo python /home/pi/missioncontrol/sensors.py &

#echo "Starting Audio Recording..."
#This command starts the USB Microphone Sound Recording
#It also writes the recorded audio to the following file: /home/pi/Audio/audio_<date><time>.wav
#sudo arecord -D plughw:1,0 -q -f cd /home/pi/Audio/audio_$backupdate$backuptime.wav &

echo "Starting Video Recording..."
#This command starts the RaspiCam Video Recording
#It also writes the recorded video to the following file: /home/pi/Video/video_<date><time>.avi
sudo /opt/vc/bin/raspivid -w 1920 -h 1080 -v -t 0 -ex auto -mm matrix -fps 30 -o /home/pi/Video/video_$backupdate$backuptime.avi 

echo "Starting Photo Recording..."
#This command starts the RapsiCam Photo Recording
#It also writes the recorded photos to the following file: /home/pi/Photo/photo_<date>_<time>.jpg
#for ((i=0; ;i++))
#do
#    backuptime=$(date +%H%M%S)
#    filename=”$backupdate$backuptime”
#    sudo /opt/vc/bin/raspistill -w 512 -h 288 -v -ex auto -mm matrix -o /home/pi/Photo/photo_small_$backupdate$backuptime.jpg
#    sleep 15
#    sudo /opt/vc/bin/raspistill -w 2592 -h 1944 -v -ex auto -mm matrix -o /home/pi/Photo/photo_large_$backupdate$backuptime.jpg
#    sleep 15
#done
