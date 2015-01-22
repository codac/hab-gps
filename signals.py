#!/usr/bin/python

import os
import sys
from time import sleep
# Class to check the for NAN values
import math
# Class for Piezo Buzzer and LED GPIO
from RPi import GPIO
# Class for GPS breakout
from GPS_Controller import GpsController

# Enable the GPS socket
os.system('sudo gpsd /dev/ttyAMA0 -F /var/run/gpsd.sock')
sleep(0.5)
# Create the gps controller object
gpsc = GpsController()
# Start gps controller
gpsc.start()

# Initialize variables for the Piezo Buzzer and LEDs
PIN = 18
# Beep length
BUZZER_REPETITIONS = 400
# Beep Delay
BUZZER_DELAY = 0.001
# Beep Pause
PAUSE_TIME = 1.0
ledCounter = 2
# Setup the Piezo Buzzer and LEDs
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(PIN, GPIO.OUT)

try: 
    while(True):

        # Fetch GPS data from Ultimate GPS Breakout
        altitude2 = gpsc.fix.altitude

        # Start the Piezo Buzzer and LEDs if payload reaches below 900m altitude
        if (altitude2 < 900) or (math.isnan(altitude2)):
            if (ledCounter > 4):
                for _ in xrange(BUZZER_REPETITIONS):
                    for value in [True, False]:
                        GPIO.output(PIN, value)
                        sleep(BUZZER_DELAY)
                sleep(PAUSE_TIME)
                ledCounter = 0
            else:
                ledCounter = ledCounter + 1
        else:
            print math.isnan(altitude2)

#Ctrl C
except KeyboardInterrupt:
    sys.exit('User cancelled...')

#System, hence bash script exit
except SystemExit:
    sys.exit('System Exit...')  

#Error
except:
    sys.exit('Unexpected error:', sys.exc_info()[0])

finally:
    print 'Stopping signal output...'
    gpsc.stopController()
    # Wait for the tread to finish
    gpsc.join()

print 'Done...'
