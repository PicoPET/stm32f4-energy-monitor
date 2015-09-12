#!/usr/bin/python

# Output continuous data in CSV format

import pyenergy
from time import sleep

em = pyenergy.EnergyMonitor('EE00')
em.connect()

em.enableMeasurementPoint(1)
em.start()

print "time, voltage, current"
while True:
    v = em.getInstantaneous(1)
    mp = v[5]
    resistor = em.measurement_params[mp]['resistor']
    gain = em.measurement_params[mp]['gain']
    vref = em.measurement_params[mp]['vref']

    print "{}, {}, {}".format(v[4] * 2. / 168000000 * 2, float(vref) / 4096. * v[2] * 2, float(vref) / gain / resistor / 4096. * v[3])
    sleep(0.0001) # 100 microseconds period, i.e., 10kHz sampling
