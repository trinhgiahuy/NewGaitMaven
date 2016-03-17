#!/bin/bash
#TODO might be a good idea to start the nodes at the beginning and only  start/stop the bag recording

GPSTEMPFILE="/home/ubuntu/gpstemp.log"
LOGFILE="/home/ubuntu/measurement.log"
DATADIR="/home/ubuntu/data"
# empty logfile
> $LOGFILE

GRIVEPID=-1

STOPPED=0
UPLOADED=1

IMUERR=0
GPSERR=0

online=1

# Functions

function logger {
	echo "[$(date)] $1" >> $LOGFILE
	echo "[$(date)] $1"
}

function quitScreens {
	if screen -list | grep -q "gps"; then
		logger "Shutting down the gps"

		screen -S gps -X quit
	fi

	if screen -list | grep -q "imu"; then
		logger "Shutting down the imu"

		screen -S imu -X quit
	fi

	if screen -list | grep -q "bag"; then
		logger "Shutting down bag"
		#The bag needs to exit gracefully
		screen -S bag -X stuff $'\003'
		sleep 3
		screen -S bag -X quit
	fi
}

function check3G {
	if [ "$online" -ne 0 ]; then
		if lsusb | grep -q 12d1:1003; then
			logger "Connecting 3G"
			# Connect to 3G with Huawei E160, saunalahti
			sudo sakis3g connect OTHER="USBMODEM" USBMODEM="12d1:1003" APN="internet.saunalahti"
		fi
	fi
}

# Get Xsens device
XSENS=""
#for f in /dev/serial/by-id/usb-Xsens_Xsens_COM_port*; do
for f in /dev/serial/by-id/usb-Xsens_Xsens_*; do

    ## Check if the glob gets expanded to existing files.
    ## If not, f here will be exactly the pattern above
    ## and the exists test will evaluate to false.
    if [ -e "$f" ]; then
         XSENS=$f
    fi

    ## This is all we needed to know, so we can break after the first iteration
    break
done

#/dev/serial/by-id/usb-Xsens_Xsens_USB-serial_converter_XSUO65V1-if00-port0

#echo "Trying to quit grive-screen"
#echo "Trying to quit grive-screen" >> $LOGFILE
#screen -r grive -X quit
#echo "Starting up grive-screen"
#echo "Starting up grive-screen" >> $LOGFILE
#screen -dmS grive

while true; do

	check3G

	# enough timeout for 3g
	nc -w 10 -z 8.8.8.8 53  >/dev/null 2>&1
#	if [ -f "online" ]; then
#		online=0
#	else
#		online=1
#	fi
	online=$?
	if [ $online -eq 0 ]; then

		#echo "System online, stopping recording"
		#echo "System online, stopping recording" >> $LOGFILE

		# Stop IMU, GPS and recording
		quitScreens

		if [ "$STOPPED" -ne 0 ]; then

			#Uploading the data
			logger "Waiting for everything to shut down"
			while screen -list | grep "imu\|gps\|bag"; do
				sleep 2
			done

			STOPPED=0

		fi


		if [ "$UPLOADED" -ne 0 ]; then
			if ! ps -p $GRIVEPID > /dev/null 2>&1; then
				logger "Uploading the data"
				#screen -r grive -X stuff $'\ngrive\n'
				# upload in subshell, wait for finish (should we?)
				(cd $DATADIR; grive) & >> $LOGFILE
				GRIVEPID=$!
			fi
			UPLOADED=0
		fi
	else

		UPLOADED=1

#		if [ "$GRIVEPID" -ne -1 ]; then
#			logger "Stopping GRIVE, measuring, $GRIVEPID"
#			kill -- -"$GRIVEPID"
#			GRIVEPID=-1
#		fi

		if [ "$GPSERR" -gt 10 ]; then
			logger "GPS-errors more than 10, restarting"
			GPSERR=0
			quitScreens
		fi

		#NOTE! The gps is a launch file which will take care of starting the ros core
		#If this is not used, separate core process must be launched
		if ! screen -list | grep -q "gps"; then
			logger "Offline: Starting GPS and ROSCORE"
			screen -dmS gps
			screen -r gps -X stuff $'\nrm '$GPSTEMPFILE$'\nroslaunch ublox_gps ublox_gps.launch 2> '$GPSTEMPFILE$'\n'

			# should have time to error, if going to
	                sleep 15

	                # if file exists and not empty
	                if [ -s $GPSTEMPFILE ]; then
				logger "Error on starting gps"
				logger $GPSTEMPFILE
	                        screen -S gps -X quit
	                        # start over
	                        continue
	                elif [ ! -f $GPSTEMPFILE ]; then
				logger "GPSlog not found"
				screen -S gps -X quit
                                # start over
				continue
			fi

		fi

		## Not unless gps running
		if screen -list | grep -q "gps"; then

			if [ "$IMUERR" -gt 10 ]; then
				logger "IMU-errors more than 10, restaring IMU"
				IMUERR=0
				screen -S imu -X quit
			fi

			if ! screen -list | grep -q "imu"; then
				logger "Offline: Starting IMU"
				screen -dmS imu
				# /dev/serial/by-id/usb-Xsens_Xsens_COM_port_00342762-if00
				#screen -r imu -X stuff $'\nrosrun xsens_driver mtnode_new.py _device:=/dev/serial/by-id/usb-Xsens_Xsens_COM_port_00340764-if00\n'
				screen -r imu -X stuff $'\nrosrun xsens_driver mtnode_new.py _device:='$XSENS$'\n'
			fi

			if ! screen -list | grep -q "bag"; then
				logger "Offline: Starting RECORDING"
				screen -dmS bag
				screen -r bag -X stuff $'\nrosbag record -a\n'
			fi
		else
			logger "GPS wasn't running!"
		fi

		STOPPED=1

		# Check we have wanted topics (so working)
		# TODO: might fail for some reason? Causing recording to restart...
		# Might be fixed with zeroing of errors
		if ! rostopic list | grep -q "/imu/data"; then
			((IMUERR++))
		else
			IMUERR=0
		fi

		if ! rostopic list | grep -q "/gps/fix"; then
			((GPSERR++))
		else
			GPSERR=0
		fi

	fi

	#Sleeping a while
	sleep 2

done
