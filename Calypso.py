#! /usr/bin/env python

# import rospy
from bluepy import btle

# from geometry_msgs.msg import Pose2D
# from std_msgs.msg import Int32

from numpy import pi
import numpy as np
import struct
import math

import sys

INFO_SERVICE = 0x180A
INFO_MANUF_CHARAC_IN = 0x2A29
INFO_MODEL_CHARAC_IN = 0x2A24
INFO_FIRMW_CHARAC_IN = 0x2A26
DATA_SERVICE = 0x180D
DATA_DATA_CHARA_IN = 0x2A39
DATA_STATU_CHARAC_IN = 0xA001
DATA_DRATE_CHARAC_IN = 0xA002
DATA_SENS_CHARAC_INOUT = 0xA003
DATA_OFFS_CHARAC_INOUT = 0xA007
DATA_CALIB_CHARAC_INOUT = 0xA008
DATA_CORREC_CHARAC_INOUT = 0xA009
DATA_FIRMWC_CHARAC_INOUT = 0xA00A

class delegateNotify(btle.DefaultDelegate):
    battery = 0
    temp = 0
    roll = 0
    pitch = 0
    compass = 0
    def __init__(self):
        btle.DefaultDelegate.__init__(self)

    def handleNotification(self, candle, data):
        #print("a notification arrived: %s" %data)
        datab = bytearray(data)
        if(len(datab) == 10):
            w_speed, w_dir, self.battery, self.temp, self.roll, self.pitch, self.compass = struct.unpack('<HHBBBBH',datab)
            w_speed = w_speed/100.0
            w_dir = math.radians((180-w_dir)%360)
            self.battery = self.battery*10.0
            self.temp = self.temp-100
            self.roll = self.roll - 90
            self.pitch = self.pitch - 90
            self.theta = w_dir
            self.x = w_speed*math.cos(w_dir)
            self.y = w_speed*math.sin(w_dir)

    def getBattery(self):
        return self.battery

    def getTemp(self):
        return self.temp

    def getRoll(self):
        return self.pitch

    def getPitch(self):
        return self.roll

    def getCompass(self):
        return self.compass

    def getWind(self):
        return self.x,self.y,self.theta
"""
if __name__ == "__main__":
    #rospy.init_node('CalypsoWind', anonymous=True)
    #pub = rospy.Publisher('/sailboat/wind', Pose2D, queue_size = 2)
    #pubBat = rospy.Publisher('/sailboat/windStatus', Int32, queue_size = 2)

    #rate = rospy.Rate(4)

    #if not rospy.has_param('~mac'):
    #    sys.exit("Missing mac param")
    #mac_addr = str(rospy.get_param('~mac','D8:2F:C8:9A:F8:A7'))
    mac_addr="D8:2F:C8:9A:F8:A7"
    dev = btle.Peripheral(mac_addr, "random")
    #for ser in dev.services:
    #    print str(ser)

    dev.setDelegate(delegateNotify())
    dataService = dev.getServiceByUUID(btle.UUID(DATA_SERVICE))
    data_charac = dataService.getCharacteristics(DATA_DATA_CHARA_IN)[0]

    #print('Turning on sensors')

    #sensors_ch = dataService.getCharacteristics(DATA_SENS_CHARAC_INOUT)[0]
    #sensors_ch.write(bytes("\01"))

    infoService = dev.getServiceByUUID(btle.UUID(INFO_SERVICE))
    manuf_charac = infoService.getCharacteristics(INFO_MANUF_CHARAC_IN)[0]
    print('Manufacturer : ' + str(manuf_charac.read()))
    model_charac = infoService.getCharacteristics(INFO_MODEL_CHARAC_IN)[0]
    print('Model : ' + str(model_charac.read()))

    print('Turning on notification')
    ch = dataService.getCharacteristics()[0]
    dev.writeCharacteristic(ch.valHandle+1, b"\x01\x00")

    #while not rospy.is_shutdown():
    try:
        dev.waitForNotifications(1.0)
    except:
        print("exception in wait notification")

    if(dev.delegate.getBattery() != 0):
        #pubBat.publish(dev.delegate.getBattery())
        print(dev.delegate.getBattery())
    #pub.publish(dev.delegate.getWind())
    print(dev.delegate.getWind())
    #rate.sleep()

    print('Turning off notification')
    dev.writeCharacteristic(ch.valHandle+1, b"\x00\x00")
    #print('Turning off sensors')
    #sensors_ch.write(bytes("\00"))
    dev.disconnect()
"""

def init_Calypso(mac_addr="D8:2F:C8:9A:F8:A7") :                
    
    dev = btle.Peripheral(mac_addr, "random")
    dev.setDelegate(delegateNotify())
    dataService = dev.getServiceByUUID(btle.UUID(DATA_SERVICE))

    # data_charac = dataService.getCharacteristics(DATA_DATA_CHARA_IN)[0]
    # infoService = dev.getServiceByUUID(btle.UUID(INFO_SERVICE))
    # manuf_charac = infoService.getCharacteristics(INFO_MANUF_CHARAC_IN)[0]
    # print('Manufacturer : ' + str(manuf_charac.read()))
    # model_charac = infoService.getCharacteristics(INFO_MODEL_CHARAC_IN)[0]
    # print('Model : ' + str(model_charac.read()))

    print('Turning on notification')
    ch = dataService.getCharacteristics()[0]
    dev.writeCharacteristic(ch.valHandle+1, b"\x01\x00")
    print("Battery : {}".format(dev.delegate.getBattery()))
    return dev

def reco() :
    connected=False
    i=1; attempts=10;
    while not connected and i <= attempts :
        try :
            dev=btle.Peripheral(mac_addr,"random")
            connected = True
        except :
            print("Failed to connect to Calypso {}/{} times".format(i,attempts))
            i+=1
    print("Reach to reconnect Calypso")
    ch=dataService.getCharacteristics()[0]
    dev.writeCharacteristic(ch.valHandle+1, b"\x01\x00")

def data_cal(dev) :
    try :
        dev.waitForNotifications(1.0)
    except :
        print("Lost the siggnal of the Calypso, trying to reconnect ...")
        reco(dev)
    if (dev.delegate.getBattery() != 0) :
        dev.delegate.getBattery()
    wind=dev.delegate.getWind()
    return wind

