# -*- coding: utf-8 -*-
"""
Created on Mon Jul 13 17:01:45 2020

@author: derryn harvie

This test shows the LES filter exposed to some harmonics from a grid capture.

Due to the limited number of harmonics simulated to fit within the intended
sample rate of 1600sps, it's not a perfect representation by any means.  However
it shows the algorithm does very well when presented with mildly distorted data
as typically seen in a distribution grid.

In the real world scenario, the captured data at 12800sps will need a decimation
filter as the first part of the chain to remove the high order harmonics. However
this would have complicated the test as it would introduce a phase delay.
"""
import numpy as np
import matplotlib.pyplot as plt

from les import LES



# Generating test data from modified grid capture.  See the blog for more details
def gen_seq(Fs=12800, output_length=12800, output_fundamental = 50.0, num_harmonics = 50):    
    # Load a real waveform captured from the grid
    d = np.load('wave_20200711-054331.npy')
    fd = np.fft.fft(d)
    harmonics_mag = (np.abs(fd)*2/len(fd))[::8]
    harmonics_angle = (np.angle(fd))[::8]

    
    # To simplify future investigations, we'll align the time of the vector start with fundamental at angle 0
    # That way at array index 0, the fundamental will be at angle 0.
    timeshift = -harmonics_angle[1]/(2*np.pi*output_fundamental)
    to = np.linspace(0, output_length/Fs, output_length, endpoint=False) + timeshift
    
    # Generate the output vector
    y = np.zeros(output_length)
    for h in range(1, num_harmonics+1):
        y = y + harmonics_mag[h]*np.cos(2*np.pi*output_fundamental*h*to + harmonics_angle[h])
    
    return y


ft = 48.1
tt = np.linspace(0, 32*10/1600, 32*10, endpoint=False)
yt =  gen_seq(Fs=1600, output_length=32*10, output_fundamental=ft, num_harmonics = 10)

plt.plot(tt,yt)
plt.title('Input wave')

les = LES(iterations=1)
les_out = les.run_les(yt)


# A phase comparison vector. This should be the idea output.
pt = (2*np.pi*ft*tt+np.pi) % (2*np.pi) - np.pi


# Visualisation of the output
fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, sharex=True, sharey=False)
fig.suptitle('LES steady state test', fontsize=16)

# Plots have the LES generated lines wider so the correct lines is overlayed and visable.
# Plot the phases
ax1.plot(tt, les_out[1], linewidth=3)
ax1.plot(tt, pt)
ax1.set_title('Phase (rad)')

# Plot the magnitude
ax2.plot(tt, les_out[2], linewidth=3)
ax2.set_title('Magnitude')

# Plot the frequency
ax3.plot(tt, les_out[0], linewidth=3)
ax3.plot(tt, np.ones(len(tt))*ft)
ax3.set_title('Frequency (Hz)')

# Plot the DC offset
ax4.plot(tt, les_out[3], linewidth=3)
ax4.set_title('DC Offset')

plt.show()

