import sys
import time

import navio2.pwm
import navio2.util
from numpy import *

navio2.util.check_apm()

#def servo(piece_pin=1) :
if __name__=="__main__" :
    PWM_OUTPUT = 1
    with navio2.pwm.PWM(PWM_OUTPUT) as pwm :
        pwm.set_period(50)
        pwm.enable()
        #Faire tourner :
        t=[0.8,0.95,1.173,1.405,1.637,1.872,2.105]
        #print("Bouge")
        #milieu à 1.40
        p0=1.45
        offset_pos=3
        #supposons que 1 tour de servo = pi/6
        angle=45
        angle_tour=30
        #1t=30°
        #x=angle
        #1t*angle/(30)
        x_tour=angle/angle_tour
        pos=t[int(x_tour)+offset_pos]+(x_tour-int(x_tour))*(t[int(x_tour)+offset_pos+1]-t[int(offset_pos+x_tour)])
        while True :
            pwm.set_duty_cycle(p0)
