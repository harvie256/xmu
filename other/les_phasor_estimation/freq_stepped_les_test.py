# -*- coding: utf-8 -*-
"""
Created on Sun Jul 12 18:24:46 2020

@author: Derryn Harvie

The following test is of the LES phasor estimation algorithm with a stepped input.

"""

import numpy as np
import matplotlib.pyplot as plt

from les import LES


ft = 48 # This will be the second part, the first will be 50hz to make the test vector generation simple
mag = 330.0
dc_offset = 0.0

# A time vector, noting 1600sps is used.
tt = np.linspace(0, 32*10/1600, 32*10, endpoint=False)

# A phase comparison vector. This should be the idea output.
pt = np.concatenate((((2*np.pi*50.0*tt+np.pi) % (2*np.pi) - np.pi),(2*np.pi*ft*tt+np.pi) % (2*np.pi) - np.pi))

# The test input vector
yt =  np.concatenate((mag*np.cos(2*np.pi*50*tt) + dc_offset, mag*np.cos(2*np.pi*ft*tt) + dc_offset))

# Re-define tt to extend the full length.
tt = np.linspace(0, 2*32*10/1600, 2*32*10, endpoint=False)
step_time = (32*10)/1600 # used for drawing a line of the plots

# Create and run the filter, noting increasing the number of iterations
# will reduce the LES filter settling time at the cost of higher computation time.
les = LES(iterations=1)
les_out = les.run_les(yt)


# Visualisation of the output
fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, sharex=True, sharey=False)
fig.suptitle('LES Freq Step Test', fontsize=16)

# Plots have the LES generated lines wider so the correct lines is overlayed and visable.
# Plot the phases
ax1.plot(tt, les_out[1], linewidth=3)
ax1.plot(tt, pt)
ax1.set_title('Phase (rad)')
ax1.axvline(x=step_time, ls='--', c='red')

# Plot the freqency
ax2.plot(tt, les_out[2], linewidth=3)
ax2.plot(tt, np.ones(len(tt))*mag)
ax2.set_title('Magnitude')
ax2.axvline(x=step_time, ls='--', c='red')

# Plot the magnitude
ax3.plot(tt, les_out[0], linewidth=3)
#ax3.plot(tt, np.ones(len(tt))*ft)
ax3.set_title('Frequency (Hz)')
ax3.axvline(x=step_time, ls='--', c='red')


# Plot the DC offset
ax4.plot(tt, les_out[3], linewidth=3)
ax4.plot(tt, np.ones(len(tt))*dc_offset)
ax4.set_title('DC Offset')
ax4.axvline(x=step_time, ls='--', c='red')

plt.show()
