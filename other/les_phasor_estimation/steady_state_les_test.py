# -*- coding: utf-8 -*-
"""
Created on Sun Jul 12 18:24:46 2020

@author: Derryn Harvie

The following test is of the phasor estimation algorithm under steady state
conditions with perfect input. Note the output of the algorithm does not
start until 1.5 window lengths + 1 sample in (as it needs samples to work on).

"""

import numpy as np
import matplotlib.pyplot as plt

from les import LES


ft = 48.1
mag = 330.0
dc_offset = 0.3

# A time vector, noting 1600sps is used.
tt = np.linspace(0, 32*10/1600, 32*10, endpoint=False)

# A phase comparison vector. This should be the idea output.
pt = (2*np.pi*ft*tt+np.pi) % (2*np.pi) - np.pi

# The test input vector
yt = mag*np.cos(2*np.pi*ft*tt) + dc_offset

# Create and run the filter, noting increasing the number of iterations
# will reduce the LES filter settling time at the cost of higher computation time.
les = LES(iterations=1)
les_out = les.run_les(yt)



# Visualisation of the output
fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, sharex=True, sharey=False)
fig.suptitle('LES steady state test', fontsize=16)

# Plots have the LES generated lines wider so the correct lines is overlayed and visable.
# Plot the phases
ax1.plot(tt, les_out[1], linewidth=3)
ax1.plot(tt, pt)
ax1.set_title('Phase (rad)')

# Plot the freqency
ax2.plot(tt, les_out[2], linewidth=3)
ax2.plot(tt, np.ones(len(tt))*mag)
ax2.set_title('Magnitude')

# Plot the magnitude
ax3.plot(tt, les_out[0], linewidth=3)
ax3.plot(tt, np.ones(len(tt))*ft)
ax3.set_title('Frequency (Hz)')

# Plot the DC offset
ax4.plot(tt, les_out[3], linewidth=3)
ax4.plot(tt, np.ones(len(tt))*dc_offset)
ax4.set_title('DC Offset')

plt.show()
