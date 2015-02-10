# -*- mode: python; py-indent-offset: 4; coding: utf-8-unix; encoding: utf-8 -*\
-  
##############################################################################
#  pyRadMon - logger for Geiger counters                                     #
#  Original Copyright 2013 by station pl_gdn_1                               #
#  Copyright 2014 by Auseklis Corporation, Richmond, Virginia, U.S.A.        #
#                                                                            #
#  This file is part of The PyRadMon Project                                 #
#  https://sourceforge.net/p/pyradmon                                        #
#                                                                            #
#  PyRadMon is free software: you can redistribute it and/or modify it under #
#  the terms of the GNU General Public License as published by the Free      #
#  Software Foundation, either version 3 of the License, or (at your option) #
#  any later version.                                                        #
#                                                                            #
#  PyRadMon is distributed in the hope that it will be useful, but WITHOUT   #
#  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or     #
#  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for #
#  more details.                                                             #
#                                                                            #
#  You should have received a copy of the GNU General Public License         #
#  along with PyRadMon.  If not, see <http://www.gnu.org/licenses/>.         #
#                                                                            #
#  @license GPL-3.0+ <http://spdx.org/licenses/GPL-3.0+>                     #
##############################################################################
"""
pyRadMon - logger for Geiger counters
On the first run, a default configuration file config.cfg
will be created. Edit it to set the type of input, and
the parameters.
"""

import sys, os
import serial
import time, datetime
import threading, thread
import socket
from collections import deque
import random
import pyaudio
import math
import struct
import pdb

#? Pull this up here so that it can be overridden
#? in case this code is ever running under a GUI.
SLEEP=time.sleep
SOCKET_BUFFER_SIZE=1024

#? reporting interval - this may be hardwired to 30 seconds
INTERVAL=30

#? For pyaudio: This can be tricky to get right
# http://stackoverflow.com/questions/28174540/ioerror-errno-input-overflowed-9981
# http://www.raspberrypi.org/forums/viewtopic.php?t=48723
#? If its too big, you will lose pulses.
#? If its too small, you may get paInputOverflow.
CHUNK=1024

##############################################################################
#  version is a.b.c, change in a or b means new functionality/bugfix,        #
#  change in c = bugfix                                                      #
#  do not uncomment line below, it's currently used in HTTP headers          #
VERSION="1.1.9"
#  To see your online los, report a bug or request a new feature, please     #
#  visit http://www.radmon.org and/or https://sourceforge.net/p/pyradmon     #
##############################################################################

from optparse import OptionParser
from ConfigParser import ConfigParser
import logging
logging.basicConfig(level=logging.DEBUG)
LOG = logging.getLogger('PyRadmon')

# build the options
usage = __doc__
oParser = OptionParser(usage=usage)
oParser.add_option("-v", "--verbose",
                   action="store", dest="iVerbosity", type = "int",
                   default="4",
                   help="the Verbosity (0 - quiet, 5 - debug)")
oParser.add_option("-c", "--config",
                   action="store", dest="sConfig", type = "string",
                   default="config.cfg",
                   help="the configuration file (config.cfg)")
oParser.add_option("-L", "--list-devices",
                   action="store_true", dest="bListDevices",
                   default=False,
                   help="list the available audio devices (from pyaudio)")
oParser.add_option("-D", "--devices-info",
                   action="store_true", dest="bDevicesInfo",
                   default=False,
                   help="list the info on the default audio devices (from pyaudio)")
oParser.add_option("-s", "--stdout",
                   action="store_true", dest="bStdout",
                   default=False,
                   help="print the results on stdout (default False)")
oParser.add_option("-f", "--factor",
                   action="store", dest="fCPMUsvhr", type = "float",
                   default=0.0,
                   help="convert CPM to uSv/hr to display results on stdout (default 0)")

class baseGeigerCommunication(threading.Thread):
    """Geiger counter communication

    It should be easy to add different protocols by simply
    creating a new class based on baseGeigerCommunication, 
    as it's done in classes Demo and myGeiger
    """

    def __init__(self, oConfig):
        self.oConfig = oConfig
        super(baseGeigerCommunication, self).__init__()
        self.sPortName=oConfig.get('serial','portname')
        self.iPortSpeed=int(oConfig.get('serial','speed'))

    def initCommunication(self):
        LOG.info("Initializing geiger communication\n")
        self.stopwork=0
        self.queue=deque()
        self.queueLock=0
        self.is_running=1

    def run(self):
        try:
            LOG.info("Gathering data started\n")
            #? why not use cfg.timeout - what king of .timeout is it?
            self.serialPort = serial.Serial(self.sPortName,
                                            self.iPortSpeed,
                                            timeout=1)
            self.serialPort.flushInput()

            self.initCommunication()

            while(self.stopwork==0):
                result=self.getData()
                while (self.queueLock==1):
                    LOG.warn("Geiger communication: quene locked!\n")
                    SLEEP(0.5)
                self.queueLock=1
                self.queue.append(result)
                self.queueLock=0
                LOG.debug( "Geiger sample:\tCPM = "+str(result[0])+"\t "+str(result[1]))

            self.serialPort.close()
            LOG.info("Gathering data from Geiger stopped\n")
        except serial.SerialException as e:
            LOG.critical("Problem with serial port:\n\t" + str(e) + "\nExiting\n")
            self.stop()
            sys.exit(1)

    def sendCommand(self, command):
        self.serialPort.flushInput()
        self.serialPort.write(command)
        # assume that device responds within 0.5s
        SLEEP(0.5)
        response=""
        while (self.serialPort.inWaiting()>0 and self.stopwork==0):
            response = response + self.serialPort.read()
        return response

    def getData(self):
        cpm=25
        utcTime=datetime.datetime.utcnow()
        data=[cpm, utcTime]
        return data

    def stop(self):
        self.stopwork=1
        self.queueLock=0
        self.is_running=0

    def getResult(self):
        # check if we have some data in queue
        if len(self.queue)>0:

            # check if it's safe to process queue
            while (self.queueLock==1):
                LOG.warn("getResult: quene locked!\n")
                SLEEP(0.5)

            # put lock so measuring process will not interfere with queue,
            # processing should be fast enought to not break data acquisition from geiger
            self.queueLock=1

            cpm=0
            # now get sum of all CPM's
            for singleData in self.queue:
                cpm=cpm+singleData[0]

            # and divide by number of elements
            # to get mean value, 0.5 is for rounding up/down
            cpm=int( ( float(cpm) / len(self.queue) ) +0.5)
            # report with latest time from quene
            utcTime=self.queue.pop()[1]

            # clear queue and remove lock
            self.queue.clear()
            self.queueLock=0

            data=[cpm, utcTime]
        else:
            # no data in queue, return invalid CPM data and current time
            data=[-1, datetime.datetime.utcnow()]

        return data

class Demo(baseGeigerCommunication):

    def run(self):
        LOG.info("Gathering data started\n")

        while(self.stopwork==0):
            result=self.getData()
            while (self.queueLock==1):
                LOG.warn("Geiger communication: quene locked!\n")
                SLEEP(0.5)
            self.queueLock=1
            self.queue.append(result)
            self.queueLock=0
            LOG.debug("Geiger sample:\t%r" % (result,))

        LOG.info("Gathering data from Geiger stopped\n")

    def getData(self):
        for i in range(0,5):
            SLEEP(1)
        cpm=random.randint(5,40)
        utcTime=datetime.datetime.utcnow()
        data=[cpm, utcTime]
        return data

class myGeiger(baseGeigerCommunication):

    def getData(self):
        cpm=-1
        try:
            # wait for data
            while (self.serialPort.inWaiting()==0 and self.stopwork==0):
                SLEEP(1)

            SLEEP(0.1) # just to ensure all CPM bytes are in serial port buffer
            # read all available data
            x=""
            while (self.serialPort.inWaiting()>0 and self.stopwork==0):
                x = x + self.serialPort.read()

            if len(x)>0:
                cpm=int(x)

            utcTime=datetime.datetime.utcnow()
            data=[cpm, utcTime]
            return data
        except Exception as e:
            LOG.critical("\nProblem in getData procedure (disconnected USB device?):\n\t" + str(e) + "\nExiting\n")
            self.stop()
            sys.exit(1)

class gmc(baseGeigerCommunication):

    def initCommunication(self):

        LOG.info("Initializing GMC protocol communication\n")
        # get firmware version
        response=self.sendCommand("<GETVER>>")

        if len(response)>0:
            LOG.info("Found GMC-compatible device, version: " + response)
            # get serial number
            # serialnum=self.sendCommand("<GETSERIAL>>")
            # serialnum.int=struct.unpack('!1H', serialnum(7))[0]
            # print "Device Serial Number is: ", serialnum.int
            # disable heartbeat, we will request data from script
            self.sendCommand("<HEARTBEAT0>>")
            LOG.info("Please note data will be acquired once per 5 seconds\n")
            # update the device time
            unitTime=self.sendCommand("<GETDATETIME>>")
            LOG.info("Unit shows time as: " + unitTime + "\n")
            # self.sendCommand("<SETDATETIME[" + time.strftime("%y%m%d%H%M%S") + "]>>")
            LOG.debug("<SETDATETIME[" + time.strftime("%y%m%d%H%M%S") + "]>>")

        else:
            LOG.critical("No response from device\n")
            self.stop()
            sys.exit(1)

    def getData(self):
        cpm=-1
        try:
            # wait, we want sample every 30s
            #? bug? was 3
            for i in range(0,INTERVAL):
                SLEEP(1)

            # send request
            response=self.sendCommand("<GETCPM>>")

            if len(response)==2:
                # convert bytes to 16 bit int
                cpm=ord(response[0])*256+ord(response[1])
            else:
                LOG.critical("Unknown response to CPM request, device is not GMC-compatible?\n")
                self.stop()
                sys.exit(1)

            utcTime=datetime.datetime.utcnow()
            data=[cpm, utcTime]
            return data

        except Exception as e:
            LOG.critical("Problem in getData procedure (disconnected USB device?):\n\t" + str(e) + "\nExiting\n")
            self.stop()
            sys.exit(1)

class netio(baseGeigerCommunication):

    def getData(self):
        cpm=-1
        try:
            # we want data only once per 30 seconds, ignore rest
            # it's averaged for 60 seconds by device anyway
            for i in range(0,INTERVAL):
                SLEEP(1)

            # wait for data, should be already there (from last 30s)
            while (self.serialPort.inWaiting()==0 and self.stopwork==0):
                SLEEP(0.5)

            SLEEP(0.1) # just to ensure all CPM bytes are in serial port buffer
            # read all available data

            # do not stop receiving unless it ends with \n
            x=""
            while ( x.endswith("\n")==False and self.stopwork==0):
                while ( self.serialPort.inWaiting()>0 and self.stopwork==0 ):
                    x = x + self.serialPort.read()

            # if CTRL+C pressed then x can be invalid so check it
            if x.endswith("\n"):
                # we want only latest data, ignore older
                tmp=x.splitlines()
                x=tmp[len(tmp)-1]
                cpm=int(x)

            utcTime=datetime.datetime.utcnow()
            data=[cpm, utcTime]
            return data

        except Exception as e:
            LOG.critical("Problem in getData procedure (disconnected USB device?):\n\t" + str(e) + "\nExiting\n")
            self.stop()
            sys.exit(1)

    def initCommunication(self):
        LOG.info("Initializing NetIO\n")
        # send "go" to start receiving CPM data
        response=self.sendCommand("go\n")
        LOG.info("Please note data will be acquired once per %d seconds\n"
                 % INTERVAL)

# if the noise was longer than this many blocks, it's not a 'pulse'
#? unused
# MAX_PULSE_BLOCKS = 0.15/INPUT_BLOCK_TIME

def fGetRms( block ):
    #? This is only true if the audio format is paInt16
    # RMS amplitude is defined as the square root of the
    # mean over time of the square of the amplitude.
    # so we need to convert this string of bytes into
    # a string of 16-bit samples...

    # we will get one short out for each
    # two chars in the string.
    count = len(block)/2
    format = "%dh"%(count)
    shorts = struct.unpack( format, block )

    # iterate over the block.
    fSumSquares = 0.0
    SHORT_NORMALIZE = (1.0/32768.0)
    for sample in shorts:
        # sample is a signed short in +/- 32768.
        # normalize it to 1.0
        n = sample * SHORT_NORMALIZE
        fSumSquares += n*n

    return math.sqrt( fSumSquares / count )

class audioCommunication(threading.Thread):
    """
    audio geiger handler
    """
    def __init__(self, oConfig):
        super(audioCommunication, self).__init__()
        self.oConfig=oConfig
        self.pulse_threshold = float(oConfig.get('audio', 'threshold'))
        self.noisycount = 0
        self.stopwork=0
        self.queue=deque()
        self.queueLock=0
        self.is_running=1
        self.stream = None
        self.name = 'audioCommunication'
        
    def initCommunication(self):
        LOG.info("Initializing audio communication\n")
        self.iHostApi=int(self.oConfig.get('audio','host_api'))
        self.fRate=float(self.oConfig.get('audio','rate'))
        self.iChannels=int(self.oConfig.get('audio','channels'))
        self.iDeviceIndex=int(self.oConfig.get('audio', 'device'))
        self.iFormat = getattr(pyaudio, self.oConfig.get('audio', 'format'))
        #? ignored for now
        #? self.fBlockTime = float(self.oConfig.get('audio', 'block_time'))
        self.bSquelchIoerror = int(self.oConfig.get('audio', 'ioerror_squelch')) != 0

        #? if self.iHostApi >= pa.get_host_api_count()
        if self.fRate <= 1.0:
            # use audio/rate = 0 to signal using the device default rate
            # {'defaultSampleRate': 44100.0, 'defaultLowOutputLatency': 0.008707482993197279, 'defaultLowInputLatency': 0.008707482993197279, 'maxInputChannels': 2L, 'structVersion': 2L, 'hostApi': 0L, 'index': 0, 'defaultHighOutputLatency': 0.034829931972789115, 'maxOutputChannels': 2L, 'name': u'HDA Intel: Analog (hw:0,0)', 'defaultHighInputLatency': 0.034829931972789115}
            self.fRate= self.pa.get_device_info_by_index(self.iDeviceIndex)['defaultSampleRate']

        LOG.debug('channels = ' + str(self.iChannels) + "\n" + \
                  'rate =' + str(self.fRate) + "\n" + \
                  'host_api = ' + str(self.iHostApi) + "\n" + \
                  'input_device_index = ' + str(self.iDeviceIndex) + "\n" + \
                  'frames_per_buffer = ' + str(CHUNK)
                  )

    def run(self):
        self.initCommunication()
        try:
            LOG.info("Gathering data started\n")
            while(self.stopwork==0):
                result=self.getData()
                while (self.queueLock==1):
                    LOG.warn("Geiger communication: quene locked!\n")
                    SLEEP(0.5)
                self.queueLock=1
                self.queue.append(result)
                self.queueLock=0
                LOG.debug("Geiger sample:\tCPM =" + str(result[0]) + "\t" + str(result[1]))

            LOG.info("Gathering data from Geiger stopped\n")
        except Exception as e:
            LOG.critical("Problem with audio port:\n\t" + str(e) + "\nExiting\n")
            self.stop()
            sys.exit(1)
            
    def getData(self):
        # iInput_Frames_Per_Block = int(self.fRate*self.fBlockTime)
        #? was INPUT_BLOCK_TIME = 0.05 * 44100 = 2205
        self.stream = self.pa.open(format = self.iFormat,
                                   channels = self.iChannels,
                                   rate = self.fRate,
                                   input = True,
                                   input_device_index=self.iDeviceIndex,
                                   start=True,
                                   #? was iInput_Frames_Per_Block
                                   frames_per_buffer = CHUNK)
        #? where did the hardcoded constant 600 come from?
        #? is it: 30sec = INPUT_BLOCK_TIME * 600?
        RECORD_SECONDS = INTERVAL
        for i in range(0, int(self.fRate / CHUNK * RECORD_SECONDS)):
            try:
                #? weird behaviour on ^C
                #? ^C is always hard in threaded code
                if not self.stream: break
                #? was iInput_Frames_Per_Block
                block = self.stream.read(CHUNK)
            except (pyaudio.paInputOverflowed, IOError,):
                #? buffer overflows are a real problem in pyaudio
                #? depending on the choice of fRate and CHUNK.
                #? signal them to the user, but ignore them -
                #? play with fRate and CHUNK until they are at a minimum
                if self.is_running and not self.bSquelchIoerror:
                    LOG.warning("paInputOverflow on audio port => %d" % i )
                continue
            except Exception as ex:
                # pdb.set_trace()
                LOG.critical("Problem with audio port => %d:\n\t" % i + \
                             str(ex) + "\nExiting\n")
                if self.stream:
                    self.stream.stop_stream()
                    self.stream.close()
                    self.stream = None
                self.stop()
                sys.exit(1)

            amplitude = fGetRms( block )
            LOG.log(0, "amplitude = fGetRms => %f" % amplitude )
            if amplitude > self.pulse_threshold:
                # noisy block
                LOG.debug("amplitude count => %f %d" % (amplitude, self.noisycount) )
                self.noisycount += 1

        if self.stream:
            self.stream.stop_stream()
            self.stream.close()
            self.stream=None
            
        if self.noisycount >= 0:
            #? Maybe we should use the utc time because of dropped frames?
            cpm = self.noisycount * ( 60 / RECORD_SECONDS )
            self.noisycount = 0

        utcTime=datetime.datetime.utcnow()
        data=[cpm, utcTime]
        return data

    def stop(self):
        if self.stream:
            self.stream.stop_stream()
            self.stream.close()
            self.stream = None
        self.stopwork=1
        self.queueLock=0
        self.is_running=0
            
    def getResult(self):
        # check if we have some data in queue
        if len(self.queue)>0:

            # check if it's safe to process queue
            while (self.queueLock==1):
                LOG.warn("getResult: quene locked!\n")
                SLEEP(0.5)

            # put lock so measuring process will not interfere with queue,
            # processing should be fast enought to not break data acquisition from geiger
            self.queueLock=1

            cpm=0
            # now get sum of all CPM's
            for singleData in self.queue:
                cpm=cpm+singleData[0]

            # and divide by number of elements
            # to get mean value, 0.5 is for rounding up/down
            cpm=int( ( float(cpm) / len(self.queue) ) +0.5)
            # report with latest time from quene
            utcTime=self.queue.pop()[1]

            # clear queue and remove lock
            self.queue.clear()
            self.queueLock=0

            data=[cpm, utcTime]
        else:
            # no data in queue, return invalid CPM data and current time
            data=[-1, datetime.datetime.utcnow()]

        return data

class WebCommunication():
    """
    Web server communication
    """
    HOST="www.radmon.org"
    #HOST="127.0.0.1" # uncomment this for debug purposes on localhost
    PORT=80

    def __init__(self, oConfig):
        self.oConfig = oConfig
        self.user=oConfig.get(self.HOST, 'user')
        self.password=oConfig.get(self.HOST, 'password')

    def sendSample(self, sample):

        #? Dont do anything if there are no credentials
        if not self.user or not self.password: return

        LOG.info("Connecting to server\n")

        sampleCPM=sample[0]
        sampleTime=sample[1]

        # format date and time as required
        dtime=sampleTime.strftime("%Y-%m-%d%%20%H:%M:%S")

        url="GET /radmon.php?user="+self.user+"&password="+self.password+"&function=submit&datetime="+dtime+"&value="+str(sampleCPM)+"&unit=CPM HTTP/1.1"

        request=url+"\nHost: www.radmon.org\nUser-Agent: pyRadMon "+VERSION+"\n\n"
        LOG.debug("Sending average sample: "+str(sampleCPM)+" CPM\n")
        #print "\n### HTTP Request ###\n"+request

        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            s.connect((self.HOST, self.PORT))
            s.send(request)
            data = s.recv(SOCKET_BUFFER_SIZE)
            httpResponse=str(data).splitlines()[0]
            LOG.debug("Server response: " + httpResponse)

            if "incorrect login" in data.lower():
                LOG.critical("You are using incorrect user/password combination!\n")
                geigerCommunication.stop()
                sys.exit(1)
        finally:
            #print "\n### HTTP Response ###\n"+data+"\n"
            #? bug: this was missing parentheses
            s.close()

def sPyDevicesInfo(oPa, iHostApi=0):
    #? what about get_host_api_count() - we can have 2 sound cards...
    sRetval = "# Host Api info for device %d" % iHostApi
    info = oPa.get_host_api_info_by_index(iHostApi)
    numdevices = info.get('deviceCount')
    # for each audio device, determine if is an input or an output
    # and add it to the appropriate list and dictionary
    for i in range (0,numdevices):
        if oPa.get_device_info_by_host_api_device_index(iHostApi,i).get('maxInputChannels') > 0:
            sRetval += "# %d - %r \n" % (i, oPa.get_device_info_by_host_api_device_index(iHostApi,i))
    return sRetval
    
def sPyAudioListDevices(oPa, iHostApi=0):
    #? what about get_host_api_count() - we can have 2 sound cards...
    sRetval = "# In case of audio, input the device number here, default is 0.\n"
    info = oPa.get_host_api_info_by_index(iHostApi)
    numdevices = info.get('deviceCount')
    # for each audio device, determine if is an input or an output
    # and add it to the appropriate list and dictionary
    for i in range (0,numdevices):
        if oPa.get_device_info_by_host_api_device_index(iHostApi,i).get('maxInputChannels') > 0:
            sRetval += "# %d - %s \n" % (i,oPa.get_device_info_by_host_api_device_index(iHostApi,i).get('name'))
    return sRetval

CONFIG_TEMPLATE = \
"""# Parameter names are not case-sensitive
# Parameter values are case-sensitive
[default]
# Protocols: demo, mygeiger, gmc, netio, audio
protocol=demo
# number of 30sec readings to average for long-term
long_average=6
# you may want to squelch empty queue warnings
queue_squelch=0

[www.radmon.org]
user=test_user
password=test_password

[serial]
# Port is usually /dev/ttyUSBx in Linux and COMx in Windows
portname=/dev/ttyUSB0
speed=2400

[audio]
# this isnt wired up yet - only 0 works
host_api=0
# you may have to be careful if you use other than 44100
# use audio/rate = 0 to signal using the device default rate
rate=0
# geigers usually come in mono
channels=1
format=paInt16
# This is the threshold on the RMS average over a block
threshold=0.010
# This is now ignored because of pyaudio issues - it may come back
block_time=0.05
# you may want to squelch ioerrors from portaudio
ioerror_squelch=0
"""

def main(lArgs):
    """
    Main code
    """
    #? maybe do this after reading the config file to override it
    (oValues, lArguments) = oParser.parse_args(lArgs)
    if oValues.iVerbosity is not None:
        assert 0 <= oValues.iVerbosity <= 5, "--verbose is between 0 and 5"
        LOG.setLevel(50 - 10 * oValues.iVerbosity)

    oPa=pyaudio.PyAudio()
    # check if file exists, if not, create one and exit
    if not os.path.isfile(oValues.sConfig):
        LOG.info("\tNo configuration file, creating default one.\n\t")

        try:
            f = open(oValues.sConfig, 'wt')
            f.write(CONFIG_TEMPLATE +
                    sPyAudioListDevices(oPa, iHostApi=0) +
                    "\ndevice=0\n")
            LOG.critical("\tPlease edit config.cfg with a text editor and update configuration.\n")
        except Exception as e:
            LOG.info("\tFailed to create configuration file\n\t" + str(e))
        finally:
            f.close()
        sys.exit(1)

    # create and read configuration data
    oConfig = ConfigParser()
    try:
        oConfig.readfp(open(oValues.sConfig))
        oConfig.sConfigFile = oValues.sConfig
    except Exception as e:
        LOG.critical("\tFailed to read configuration file!" + str(e) +  "\nExiting\n")
        sys.exit(1)
        
    if oValues.bListDevices:
        iHostApi=int(oConfig.get('audio','host_api'))
        sys.stdout.write(sPyAudioListDevices(oPa, iHostApi))
        sys.exit(0)
        
    if oValues.bDevicesInfo:
        iHostApi=int(oConfig.get('audio','host_api'))
        sys.stdout.write(sPyDevicesInfo(oPa, iHostApi))
        sys.exit(0)

    # demo, mygeiger, gmc, netio, audio
    sProtocol = oConfig.get('default', 'protocol')
    # create geiger communication object
    if sProtocol == 'mygeiger':
        LOG.debug("Using myGeiger protocol for 1\n")
        geigerCommunication=myGeiger(oConfig)
    elif sProtocol == 'demo':
        LOG.debug("Using Demo mode for 1\n")
        geigerCommunication=Demo(oConfig)
    elif sProtocol == 'gmc':
        LOG.debug("Using GMC protocol for 1\n")
        geigerCommunication=gmc(oConfig)
    elif sProtocol == 'netio':
        LOG.debug("Using NetIO protocol for 1\n")
        geigerCommunication=netio(oConfig)
    elif sProtocol == 'audio':
        LOG.debug("Using audio protocol for 1\n")
        geigerCommunication = audioCommunication(oConfig)
        geigerCommunication.pa = oPa
    else:
        LOG.debug("Unknown protocol configured, can't run\n"+ \
                   sProtocol)
        sys.exit(1)

    # create web server communication object
    webService=WebCommunication(oConfig)

    try:
        # start measuring thread
        geigerCommunication.start()

        # Now send data to web site every 30 seconds
        iLast=-1
        dLongTermCPM = {}
        fLongTermAverage = 0.0
        iLongTerm = int(oConfig.get('default', 'long_average'))
        bSquelchEmpty = int(oConfig.get('default', 'queue_squelch')) != 0
        while(geigerCommunication.is_running==1):
            sample=geigerCommunication.getResult()

            if sample[0] == -1:
                if not bSquelchEmpty:
                    LOG.warn("No samples in queue, waiting 5 seconds\n")
                for i in range(0, 5):
                    SLEEP(1)
                continue

            # sample is valid, CPM !=-1
            if oValues.bStdout:
                iLast += 1
                iMod = iLast % iLongTerm
                dLongTermCPM[iMod] = sample[0]
                fLongTermAverage = sum(dLongTermCPM.values())/float(iLongTerm)
                # print iLast, dLongTermCPM
                sMsg = " at\t"
                if oValues.fCPMUsvhr > 0.0001:
                    sMsg = " uSv/hr = %5.3f(%5.3f) at\t" % (
                        sample[0]/oValues.fCPMUsvhr,
                        fLongTermAverage/oValues.fCPMUsvhr,
                    )
                print "CPM = %d(%d)" % (sample[0], round(fLongTermAverage)) \
                    + sMsg + str(sample[1])
            else:
                LOG.debug("Average result:\tCPM =" + str(sample[0]) + "\t" + str(sample[1]))

            if webService.user and webService.password:
                try:
                    webService.sendSample(sample)
                except Exception as e:
                    LOG.error("Error communicating with server:\n\t" + str(e))

            # LOG.debug("Waiting %d seconds\n" % INTERVAL)
            # actually waiting 30x1 seconds, it's has better response when CTRL+C is used, maybe will be changed in future
            for i in range(0, INTERVAL):
                SLEEP(1)

    except KeyboardInterrupt:
        LOG.info("CTRL+C pressed, exiting program\n")
        # drop through
    except SystemExit:
        LOG.info("Exiting program\n")
        # drop through
    except Exception as e:
        LOG.exception("\nUnhandled error\n\t" + str(e))
        # drop through

    LOG.debug("Stopping: " + geigerCommunication.getName())
    geigerCommunication.stop()

    #? threading bulletproofing
    LOG.debug("Waiting and reap threads")
    SLEEP(1)
    for oThread in threading.enumerate():
        if oThread.isDaemon(): continue
        if oThread.getName() == 'MainThread': continue
        LOG.warn("Stopping alive thread: " + oThread.getName())
        oThread.stop()
        SLEEP(1)
        
    #? there should only be one PA instance and it should be terminated
    if oPa:
        LOG.debug("Terminating pulse audio")
        oPa.terminate()
        oPa=None
    sys.exit(0)

if __name__ == '__main__':
    main(sys.argv[1:])


