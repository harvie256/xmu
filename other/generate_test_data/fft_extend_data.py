# -*- coding: utf-8 -*-
"""
Created on Sat Jul 11 05:44:22 2020

@author: derryn harvie
"""

import matplotlib.pyplot as plt
import numpy as np

""" Build a test sequence by extending and modifying real periodic data """

# Load a real waveform captured from the grid
d = np.load('wave_20200711-054331.npy')
Fs = 12800
ti = np.linspace(0, len(d)/Fs, len(d), endpoint=False)

""" 
Perform and FFT to get the mag and phase, and discard everything other than the harmonics.

    In this case the input length is 2048 and the sample rate is 12800, therefore the bin size is:
    12800/2048 = 6.25
    As the system freq is 50Hz
    50/6.25 = 8
    therefore the fft will output a harmonic on every 8th fft bin.
"""
fd = np.fft.fft(d)
harmonics_mag = (np.abs(fd)*2/len(fd))[::8]
harmonics_angle = (np.angle(fd))[::8]

# Use the mag and phase to generate a new bunch of data of arbitrary length and specific fundamental freq
output_length = 12800
output_fundamental = 50.0

# We can choose how many of the harmonics to include in the output data.
num_harmonics = 50

# To simplify future investigations, we'll align the time of the vector start with fundamental at angle 0
# That way at array index 0, the fundamental will be at angle 0.
timeshift = -harmonics_angle[1]/(2*np.pi*output_fundamental)
to = np.linspace(0, output_length/Fs, output_length, endpoint=False) + timeshift

# Generate the output vector
y = np.zeros(output_length)
for h in range(1, num_harmonics+1):
    y = y + harmonics_mag[h]*np.cos(2*np.pi*output_fundamental*h*to + harmonics_angle[h])
    
# Plot the orginal data portion over the generated to look at how closely it fits.
# Noting that the input data (at least in this case) is not exactly on 50Hz (one of the fundamental problems in system meansurements),
# and we've included a limited number of harmonics, so they don't overlap exactly.
plt.plot(to, y, label='generated')
plt.plot(ti, d, label='original')
plt.legend(loc='upper right')
plt.xlabel("time (s)")
plt.ylabel("voltage (V)")
plt.title("Generated Samples")
plt.grid(True)
plt.show()
