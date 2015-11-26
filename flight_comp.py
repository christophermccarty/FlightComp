import subprocess
import sys
import time
import thread
import threading
import datetime
import os
import smbus
from Adafruit_BMP085 import BMP085
import gps


# Camera function
def rpicam():
    subprocess.Popen('sudo raspivid -o ' + cwd + '/camera/' + program_run_date_string + '.h264 -w 1280 -h 720 -vs -t ' + program_run_time_string, shell=True).communicate


# GPS reinitialization and get starting altitude for bmp180 correction
def gps_startalt():
    # Restarts GPS from scratch and allow for acquisition of signal.
    print "Initializing GPS"
    #Restart and initialize GPS
    #Step 1
    subprocess.call("sudo killall gpsd", shell=True)
    time.sleep(2)
    #Step 2
    subprocess.call("sudo rm /var/run/gpsd.sock", shell=True)
    time.sleep(2)
    #Step 3
    subprocess.call("sudo gpsd /dev/ttyUSB0 -n -F /var/run/gpsd.sock", shell=True)
    time.sleep(3)
    print "GPS restarted and initialized"

    # Listen on port 2947 (gpsd) of localhost
    session = gps.gps("localhost", "2947")
    session.stream(gps.WATCH_ENABLE | gps.WATCH_NEWSTYLE)

    # Give GPS time to get an altitude fix
    print "Sleep 15 seconds for altitude fix"
    time.sleep(15)

    # Grab starting altitude value for barometric pressure sensor and store in altitude file
    print "Storing initial altitude value from GPS for BMP180..."
    result = None
    while result is None:
        starting_alt = session.next()
        # Wait for a 'TPV' report and grab altitude value
        if hasattr(starting_alt, 'class') and starting_alt['class'] == 'TPV':
            if hasattr(starting_alt, 'time'):
                result = starting_alt.alt
                f = open('altitude', 'w')
                f.write(str(starting_alt.alt))
                f.flush()
                f.close()
    print "...done"


# GPS logging
def rpigps():
    cwd = os.getcwd()
    f = open(cwd + '/runtime', 'r')
    run_time  = f.read()    # Read runtime value
    f.close()
    max_time = int(run_time) * 60
    start_time = time.time()    # remember when we started

    # Listen on port 2947 (gpsd) of localhost
    session = gps.gps("localhost", "2947")
    session.stream(gps.WATCH_ENABLE | gps.WATCH_NEWSTYLE)

    # Give GPS time to get an altitude fix
    time.sleep(20)

    # Grab starting altitude value for barometric pressure sensor and store in altitude file
    result = None
    while result is None:
        starting_alt = session.next()
        # Wait for a 'TPV' report and grab altitude value
        if hasattr(starting_alt, 'class') and starting_alt['class'] == 'TPV':
            if hasattr(starting_alt, 'time'):
                result = starting_alt.alt
                f = open('altitude', 'a')
                f.write(str(starting_alt.alt))
                f.close()

    # Set all variables now, to be updated by each report
    dt = datetime.datetime.now()
    latitude = 0.0
    longitude = 0.0
    altitude = 0.0
    horizontal_speed = 0.0
    vertical_speed = 0.0
    track = 0.0
    satcount = 0

    while (time.time() - start_time) < max_time:
        # Wait for a 'TPV' report and display the current time
        # Default GPS speed is meters/sec
        # To see all report data, uncomment the line below
        #print report
        #print satreport

        # Grab satellite count from GPS report
        satreport = session.next()
        if hasattr(satreport, 'class') and satreport['class'] == 'SKY':
            if hasattr(satreport, 'satellites'):
                satcount = (len(satreport.satellites))

        # Read remainder of data from GPS report
        report = session.next()
        if hasattr(report, 'class') and report['class'] == 'TPV':
            if hasattr(report, 'time'):
                dt = datetime.datetime.strptime(report.time, '%Y-%m-%dT%H:%M:%S.%fz')
                latitude = report.lat
                longitude = report.lon
                altitude = report.alt * 3.28084
                horizontal_speed = report.speed * gps.MPS_TO_MPH
                vertical_speed = report.climb
                track = report.track

        cwd = os.getcwd()
        logger = open(cwd + "/data/" + 'gps.log', 'a')
        #output: datetime, latitude, longitude, altitude in ft, speed, vertical speed, track?, number of satellites
        logger.write("%s, %f, %f, %.7s, %.5s, %.5f, %.5s, %.3f\n" % (dt, latitude, longitude, altitude, horizontal_speed, vertical_speed, track, satcount))
        logger.flush()
        time.sleep(1.1)
    logger.close()

    #sys.exit()


# Accelerometer
def adxl345():
    # ADXL345 Python library for Raspberry Pi
    #
    # author:  Jonathan Williamson
    # license: BSD, see LICENSE.txt included in this package
    #
    # This is a Raspberry Pi Python implementation to help you get started with
    # the Adafruit Triple Axis ADXL345 breakout board:
    # http://shop.pimoroni.com/products/adafruit-triple-axis-accelerometer

    cwd = os.getcwd()
    f = open(cwd + '/runtime', 'r')
    run_time  = f.read()    # Read runtime value
    f.close()

    # select the correct i2c bus for this revision of Raspberry Pi
    revision = ([l[12:-1] for l in open('/proc/cpuinfo','r').readlines() if l[:8]=="Revision"]+['0000'])[0]
    bus = smbus.SMBus(1 if int(revision, 16) >= 4 else 0)

    # ADXL345 constants
    # Denver specific gravity value
    EARTH_GRAVITY_MS2   = 9.798
    # Default EARTH_GRAVITY_MS2   = 9.80665
    SCALE_MULTIPLIER    = 0.004

    DATA_FORMAT         = 0x31
    BW_RATE             = 0x2C
    POWER_CTL           = 0x2D

    BW_RATE_1600HZ      = 0x0F
    BW_RATE_800HZ       = 0x0E
    BW_RATE_400HZ       = 0x0D
    BW_RATE_200HZ       = 0x0C
    BW_RATE_100HZ       = 0x0B
    BW_RATE_50HZ        = 0x0A
    BW_RATE_25HZ        = 0x09

    RANGE_2G            = 0x00
    RANGE_4G            = 0x01
    RANGE_8G            = 0x02
    RANGE_16G           = 0x03

    MEASURE             = 0x08
    AXES_DATA           = 0x32

    class ADXL345:

        address = None

        def __init__(self, address = 0x53):
            self.address = address
            self.setBandwidthRate(BW_RATE_100HZ)
            #self.setRange(RANGE_2G)
            self.setRange(RANGE_16G)
            self.enableMeasurement()

        def enableMeasurement(self):
            bus.write_byte_data(self.address, POWER_CTL, MEASURE)

        def setBandwidthRate(self, rate_flag):
            bus.write_byte_data(self.address, BW_RATE, rate_flag)

        # set the measurement range for 10-bit readings
        def setRange(self, range_flag):
            value = bus.read_byte_data(self.address, DATA_FORMAT)

            value &= ~0x0F;
            value |= range_flag;
            value |= 0x08;

            bus.write_byte_data(self.address, DATA_FORMAT, value)

        # returns the current reading from the sensor for each axis
        #
        # parameter gforce:
        #    False (default): result is returned in m/s^2
        #    True           : result is returned in gs
        def getAxes(self, gforce = False):
            bytes = bus.read_i2c_block_data(self.address, AXES_DATA, 6)

            x = bytes[0] | (bytes[1] << 8)
            if(x & (1 << 16 - 1)):
                x = x - (1<<16)

            y = bytes[2] | (bytes[3] << 8)
            if(y & (1 << 16 - 1)):
                y = y - (1<<16)

            z = bytes[4] | (bytes[5] << 8)
            if(z & (1 << 16 - 1)):
                z = z - (1<<16)

            x = x * SCALE_MULTIPLIER
            y = y * SCALE_MULTIPLIER
            z = z * SCALE_MULTIPLIER

            if gforce == False:
                x = x * EARTH_GRAVITY_MS2
                y = y * EARTH_GRAVITY_MS2
                z = z * EARTH_GRAVITY_MS2

            x = round(x, 4)
            y = round(y, 4)
            z = round(z, 4)

            return {"x": x, "y": y, "z": z}

    #if __name__ == "__main__":
    # if run directly we'll just create an instance of the class and output the current readings
    adxl345 = ADXL345()
    axes = adxl345.getAxes(True)
    # print "ADXL345 on address 0x%x:" % (adxl345.address)
    logger = open(cwd + "/data/" + 'adxl345.log', 'a')

    max_time = int(run_time) * 60
    start_time = time.time()    # remember when we started
    while (time.time() - start_time) < max_time:
        dt = datetime.datetime.now().strftime("%H:%M:%S.%f")
        axes = adxl345.getAxes(True)
        if axes['x'] < 0:
            #print axes['x'] + 0.008
            axneg_cor = axes['x'] + 0.008
            axes['x'] = axneg_cor
        if axes['x'] > 1:
            #print axes['x'] - 0.12
            axpos_cor = axes['x'] -0.12
            axes['x'] = axpos_cor
        if axes['y'] < 0:
            #print axes['y'] + 0.08
            ayneg_cor = axes['y'] + 0.08
            axes['y'] = ayneg_cor
        if axes['y'] > 1:
            #print axes['y'] - 0.052
            aypos_cor = axes['y'] - 0.052
            axes['y'] = aypos_cor
        if axes['z'] < 0:
            #print axes['z'] + 0.108
            azneg_cor = axes['z'] + 0.108
            axes['z'] = azneg_cor
        if axes['z'] > 0:
            #print axes['z'] + 0.076
            azpos_cor = axes['z'] + 0.076
            axes['z'] = azpos_cor

        logger.write("%.11s, %.2f, %.2f, %.2f, %s" % (dt, axes['x'], axes['y'], axes['z'], "xyz\n"))
        logger.flush()
        time.sleep(0.1)
    logger.close()

    #sys.exit()


# Temperature
def mcp9808():
    t_reg = 0x05
    address = 0x18
    bus = smbus.SMBus(1) # change to 0 for older RPi revision
    reading = bus.read_i2c_block_data(address, t_reg)
    t = (reading[0] << 8) + reading[1]

    cwd = os.getcwd()
    f = open(cwd + '/runtime', 'r')
    run_time = f.read()    # Read runtime value
    f.close()
    max_time = int(run_time) * 60
    start_time = time.time()    # remember when we started
    while (time.time() - start_time) < max_time:

        # calculate temperature (see 5.1.3.1 in datasheet)
        temp = t & 0x0FFF
        temp /=  16.0
        tempf = str(temp * 9.0 / 5.0 + 32.0)
        cwd = os.getcwd()
        logger = open(cwd + "/data/" + 'mcp9808.log', 'a')
        timestamp = datetime.datetime.now().strftime("%H:%M:%S.%f")
        logger.write("%.11s, %.5s\n" % (timestamp, tempf))
        if (t & 0x1000):
            temp -= 256

        logger.flush()
        time.sleep(1)
    logger.close()
    #sys.exit()


# Barometric pressure
def bmp180():
    #!/usr/bin/python

    cwd = os.getcwd()
    f = open(cwd + '/altitude', 'r')
    # Read current altitude from GPS and apply to barometric conversion
    current_altitude = float(f.read())
    f.close()

    # Initialise the BMP085 and use STANDARD mode (default value)
    # bmp = BMP085(0x77, debug=True)
    # bmp = BMP085(0x77)

    # To specify a different operating mode, uncomment one of the following:
    # bmp = BMP085(0x77, 0)  # ULTRALOWPOWER Mode
    # bmp = BMP085(0x77, 1)  # STANDARD Mode
    bmp = BMP085(0x77, 2)  # HIRES Mode
    # bmp = BMP085(0x77, 3)  # ULTRAHIRES Mode

    # Calculate sensor adjustment value for altitude above sea level. "**" provides for a number raised to a power
    # http://www.engineeringtoolbox.com/air-altitude-pressure-d_462.html
    altitude_pressure = float((101325*(1-0.0000225577*current_altitude)**(5.25588))/100)
    sensor_adjustment = float(1013.25 - (altitude_pressure))

    # Set up timer and start while loop
    cwd = os.getcwd()
    f = open(cwd + '/runtime', 'r')
    run_time  = f.read()    # Read runtime value
    f.close()
    max_time = int(run_time) * 60
    start_time = time.time()    # remember when we started

    global bmp_altitude

    while (time.time() - start_time) < max_time:

        # Grab temp from sensor and convert to fahrenheit
        temp = bmp.readTemperature()
        tempf = str(temp * 9.0 / 5.0 + 32.0)

        # Read the current barometric pressure level from the sensor and compensate for altitude
        sensor_pressure = float(bmp.readPressure())
        sensor_pressure_adjusted = float((sensor_pressure / 100) + sensor_adjustment)

        # To calculate altitude based on an estimated mean sea level pressure
        # (1013.25 hPa) call the function as follows, but this won't be very accurate
        #   altitude = bmp.readAltitude()
        # To specify a more accurate altitude, enter the correct mean sea level
        # pressure level.  For example, if the current pressure level is 1023.50 hPa
        # enter 102350 since we include two decimal places in the integer value
        try:
            bmp_altitude = float(bmp.readAltitude(sensor_pressure_adjusted*100))
        except:
            pass

        # Setup timestamp, path to log file, and logging parameters.
        timestamp = datetime.datetime.now().strftime("%H:%M:%S.%f")
        cwd = os.getcwd()
        logger = open(cwd + "/data/" + 'bmp180.log', 'a') #log to /data/
        logger.write("%.11s, %s, %.8s, %.2f\n" % (timestamp, tempf, sensor_pressure_adjusted, bmp_altitude * 3.28084))
        logger.flush()
        time.sleep(0.5)

    logger.close()

    #sys.exit()


# User input to set timer
program_run_time_input = int(raw_input("How long do you want to run the program in minutes? ")) * 60 # Prompt user for run time in minutes; Convert to seconds
program_run_time_string = str(program_run_time_input * 1000)
program_run_date = datetime.datetime.now() # Get current date and time
program_run_date_string = program_run_date.strftime("%Y%m%dT%H%M%S") # Format current date and time for use in file name


# Run time file. Writes value from user input to a file for use in loop timers.
cwd = os.getcwd()
f = open(cwd + '/runtime', 'w')
f.write(str(program_run_time_input / 60))
f.close()


# Threads for camera
#print "\nStarting Camera"
#thread.start_new_thread(rpicam, ())


# GPS Starting altitude function
gps_startalt()


# Create main sensor threads
gpsthread = threading.Thread(target=rpigps)
accelthread = threading.Thread(target=adxl345)
tempthread = threading.Thread(target=mcp9808)
pressurethread = threading.Thread(target=bmp180)


# Start main sensor threads
print "Initializing gps, acceleration, temperature, and pressure sensors"
gpsthread.start()
accelthread.start()
tempthread.start()
pressurethread.start()

# Let threads start up
time.sleep(5)

# Wait for all threads to complete before ending program
threads_alive = True
while threads_alive == True:
    try:
        gpsalive = gpsthread.isAlive()
        accelalive = accelthread.isAlive()
        tempalive = tempthread.isAlive()
        pressalive = pressurethread.isAlive()
        if gpsalive == True or accelalive == True or tempalive == True or pressalive == True:
            threads_running = True
        else:
            threads_alive = False
    except: pass


# Exit program
print "End program"
sys.exit()
