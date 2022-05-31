import sys, time

import navio2.adc
import navio2.util

navio2.util.check_apm()

def init_adc() :
    adc = navio2.adc.ADC()
    results = [0] * adc.channel_count
    return adc,results

def voltage(adc,results):
    s = ''
    for i in range (0, adc.channel_count):
        results[i] = adc.read(i)
        s += 'A{0}: {1:6.4f}V '.format(i, results[i] / 1000)
    print(s)
    time.sleep(0.5)
