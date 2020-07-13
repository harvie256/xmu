# -*- coding: utf-8 -*-
"""
Created on Fri Jul  3 22:58:53 2020

@author: derryn harvie

This is a floating point implementation of the algorithm proposed in:
    "A Simple Synchrophasor Estimation Algorithm 
    Considering IEEE Standard C37.118.1-2011 and
    Protection Requirements"

See the blog for more details and discussion.

"""


import numpy as np
import math

class LES:

    def __init__(self, Fs=1600, Wo=50.0, iterations=1):
        self.Fs = Fs
        self.Wo = Wo
        self.iterations = iterations
        self.build_pseudo_inverse_matrix()
    
    
    def build_pseudo_inverse_matrix(self):
        samples_per_cycle = self.Fs/self.Wo
        self.window = 1.5*samples_per_cycle
        t = np.linspace(-self.window/self.Fs, 0,int(self.window), endpoint=False)

        # Build a complete pseudo-inv matrix
        pinv_list = []
        pinv_space = np.linspace(45, 55, 1001)
        for W_line in pinv_space:
            
            # modeling multiple harmonics, in this case 1 to 10
            line = []
            for n in range(1, 10):
                line.extend([np.cos(2*np.pi*W_line*t*n), np.sin(2*np.pi*W_line*t*n)])
            line.extend([np.ones(len(t))])
            A = np.array(line)

            Apinv = np.linalg.pinv(A)
            pinv_list.append(Apinv)
        
        self.pinv_array = np.array(pinv_list)
        
        """
         The following piece of commented code can be used to
         normalise the range of the array from -1 to 1 for storage in Q number format
         This is entirely to allow efficent use of microcontroller memory/time
         and should not be scaled if running in floating point implementation
        """
        # arr_max = max([(np.amax(pinv_array)), -(np.amin(pinv_array))])
        # arr_scaler = 1/arr_max
        # pinv_array = pinv_array * arr_scaler


    # The filter function, returns a tuple of phase, DC component, magnitude
    def fil_samples(self, s, freq):
        f_index = round((freq - 45.0) * 100)
        xm = np.matmul(s, self.pinv_array[f_index])
        phi = math.atan2(-xm[1],xm[0])
        mag = math.sqrt(xm[0]*xm[0]+xm[1]*xm[1])
        return phi, xm[-1], mag
    
    # calculate the freq between phi's
    def calc_freq(self, phi_1, phi_2):
        phase_rotation = phi_2 - phi_1
        phase_rotation = (phase_rotation + np.pi) % (2*np.pi) - np.pi # normalise within 0-2pi range
        freq = phase_rotation * self.Fs / (2*np.pi)
        return round(freq,2)



    def run_les(self, samples):
        # Init vars
        est_freq = 50.0
        
        # Output lists
        est_freq_out = np.zeros(int(self.window+1)).tolist()
        est_phi_out = np.zeros(int(self.window+1)).tolist()
        est_offset_out = np.zeros(int(self.window+1)).tolist()
        est_mag_out = np.zeros(int(self.window+1)).tolist()
        
        report_f = []
        report_phi = []
        report_mag = []
        
        for shift in range(int(self.window+1), int(samples.size)):
            
            for _ in range(0,self.iterations):
                data_window1 = samples[int(shift-self.window-1) : shift-1]
                data_window2 = samples[int(shift-self.window) : shift]
            
            
                phi_1, _, _ = self.fil_samples(data_window1, est_freq)
                phi_2, b, m = self.fil_samples(data_window2, est_freq)
            
                est_freq = self.calc_freq(phi_1, phi_2)
                
                if est_freq > 55.0:
                    est_freq = 55.0
                
                if est_freq < 45.0:
                    est_freq = 45.0
        
            est_freq_out.append(est_freq)
            est_phi_out.append(phi_2)
            est_offset_out.append(b)
            est_mag_out.append(m)
        
            if shift % 50 == 0 and shift != 0:
                report_f.append(est_freq)
                report_phi.append(phi_2)
                report_mag.append(b)
        
        return est_freq_out, est_phi_out, est_mag_out, est_offset_out
        

