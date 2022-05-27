import sys
import time

import navio2.pwm
import navio2.util
from numpy import *

navio2.util.check_apm()

def servo(component,angle) :                #component : "Sail" or "Rudder"                
    if component=="Sail" :
        piece_pin=1
    if component=="Rudder" :
        piece_pin=0
    else :
        print("Wrong component, please enter Sail or Rudder")
    with navio2.pwm.PWM(piece_pin) as pwm :
        pwm.set_period(50)
        pwm.enable()
        
        if component=="Sail" :
            #Setup values
            t=[0.8,0.95,1.173,1.405,1.637,1.872,2.105]  #values between which the servo rotate of 360°
            p0=1.45                                     #initial value where sail angle is 0°
            offset_pos=3                                #let assume that 360° for the servo corresponds to 30° for the sail
            angle_tour=30
            
            #Determination of the input angle value
            x_tour=angle/angle_tour
            pos=t[int(x_tour)+offset_pos]+(x_tour-int(x_tour))*(t[int(x_tour)+offset_pos+1]-t[int(offset_pos+x_tour)])
            
            while True :
                pwm.set_duty_cycle(p0)
        if component=="Rudder" :
            #Setup values 
            