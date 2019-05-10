"""
 Copyright (c) 2018-2019, Rice University 
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
 Author(s): Clay Shepard: cws@rice.edu
            Rahman Doost-Mohamamdy: doost@rice.edu

---------------------------------------------------------------------
 CSI analysis API file 
---------------------------------------------------------------------
"""

import sys
import struct
import numpy as np
import os
import math
import time
import datetime
import lts
from detect_peaks import detect_peaks

lts_freq = np.array([0,0,0,0,0,0,1,1,-1,-1,1,1,-1,1,-1,1,1,1,1,1,1,-1,-1,1,1,-1,1,-1,1,1,1,1,0,1,-1,-1,1,1,-1,1,-1,1,-1,-1,-1,-1,-1,1,1,-1,-1,1,-1,1,-1,1,1,1,1,0,0,0,0,0])

def samps2csi(samps, num_users,samps_per_user=224, fft_size=64, offset=0, bound=94, cp=0):
    """Input samps dims: Frame, Cell, Antenna, User, Sample"""
    """Returns iq with Frame, Cell, User, Pilot Rep, Antenna, Sample"""
    """Returns csi with Frame, Cell, User, Pilot Rep, Antenna, Subcarrier"""
    chunkstart = time.time()	
    #num_users = samps.shape[2]//samps_per_user
    usersamps = np.reshape(samps, (samps.shape[0],samps.shape[1],num_users,samps.shape[3],samps_per_user, 2) )
    nbat = min([(samps_per_user-bound)/(fft_size+cp),2]) 
    iq = np.empty((samps.shape[0],samps.shape[1],num_users,samps.shape[3],nbat,fft_size),dtype='complex64')
    for i in range(nbat): #2 seperate estimates
        iq[:,:,:,:,i,:] = (usersamps[:,:,:,:,offset+cp+i*fft_size:offset+cp+(i+1)*fft_size,0]+usersamps[:,:,:,:,offset+cp+i*fft_size:offset+cp+(i+1)*fft_size,1]*1j)*2**-15
    iq = iq.swapaxes(3,4)
    fftstart = time.time()
    csi = np.empty(iq.shape,dtype='complex64')
    if fft_size == 64:
        csi = np.fft.fftshift(np.fft.fft(iq, fft_size, 5),5)*lts_freq
        endtime = time.time()
        print("chunk time: %f fft time: %f" % (fftstart - chunkstart, endtime -fftstart) )
        csi = np.delete(csi,[0,1,2,3,4,5,32,59,60,61,62,63],5) #remove zero subcarriers
    return csi,iq
    
def samps2csi_findLTS(samps, num_users, samps_per_user=606, fft_size=64):
    chunkstart = time.time()	
    #num_users = samps.shape[2]//samps_per_user
    #print samps.shape
    	
    # number of frames, number of cells, number of users, number of antennas, number of samples in each frame, IQ	
    usersamps = np.reshape(samps, (samps.shape[0],samps.shape[1],num_users,samps.shape[3],samps_per_user, 2) )
    iq = np.empty((samps.shape[0],samps.shape[1],num_users,samps.shape[3],2,fft_size),dtype='complex64')
    k = 0 #use samples of ant 0 to find peak
    # print usersamps.shape
    ofst = [[0]*usersamps.shape[0] for i in range(usersamps.shape[3])] 
    good_frames = []
    for i in range(usersamps.shape[0]):
        appnd = True
        for k in range(usersamps.shape[3]):
            this_pilot = (usersamps[i,0,0,k,128:,0]+1j*usersamps[i,0,0,k,128:,1])*2**-15 
            best, actual_ltss, peaks = lts.findLTS(this_pilot)
            peaks_max_loc = detect_peaks(np.abs(peaks),mph=max(abs(peaks))/2)
            if len(peaks_max_loc)==0:
                		offset = 64
            elif peaks_max_loc[0]<140 and peaks_max_loc[0]>110:
                		offset = peaks_max_loc[0]-64
            else:
                		offset = 64
            ofst[k][i] = offset+128
            if ofst[k][i] != ofst[0][i]:
                appnd = False
        #offset = offset+128
        amp = np.mean(np.abs(this_pilot))
        if appnd and ofst[0][i] == 192 and amp > 0.01:
            good_frames.append(i)
        for j in range(2): #2 seperate estimates
            iq[i,:,:,:,j,:] = (usersamps[i,:,:,:,ofst[0][i]+j*fft_size:ofst[0][i]+(j+1)*fft_size,0]+usersamps[i,:,:,:,ofst[0][i]+j*fft_size:ofst[0][i]+(j+1)*fft_size,1]*1j)*2**-15	
    
    iq = iq.swapaxes(3,4)
    fftstart = time.time()
    csi = np.fft.fftshift(np.fft.fft(iq, fft_size, 5),5)*lts.lts_freq#*signal_max	
    		
    endtime = time.time()
    print("chunk time: %f fft time: %f" % (fftstart - chunkstart, endtime -fftstart) )
    csi = np.delete(csi,[0,1,2,3,4,5,32,59,60,61,62,63],5) #remove zero subcarriers
    return csi, iq, ofst, good_frames

def calCorr(userCSI, corr_vec):
	"""
	Calculate the instantaneous correlation with a given correlation vector
	"""
	sig_intf = np.empty((userCSI.shape[0],userCSI.shape[1],userCSI.shape[1],userCSI.shape[3]),dtype='float32')

	for sc in range(userCSI.shape[3]):
		sig_intf[:,:,:,sc] = np.abs(np.dot(userCSI[:,:,:,sc],corr_vec[:,:,sc])) / np.dot( np.abs(userCSI[:,:,:,sc]), np.abs(corr_vec[:,:,sc]) ) #TODO: can we get rid of the for loop?

	sig_sc = np.diagonal(sig_intf,axis1=1,axis2=2) # gets correlation of subcarriers for each user across bs antennas
	sig_sc = np.swapaxes(sig_sc,1,2)
	corr_total = np.mean(sig_sc,axis=2)  # averaging corr across users
	
	return corr_total, sig_sc

def demult(csi, data, method='zf'): #TODO include cell dimension for both csi and data and symbol num for data
    """csi: Frame, User, Pilot Rep, Antenna, Subcarrier"""
    """data: Frame, Antenna, Subcarrier"""
    #compute beamweights based on the specified frame.
    userCSI = np.mean(csi, 2) #average over both LTSs
    sig_intf = np.empty((userCSI.shape[0],userCSI.shape[1],userCSI.shape[3]),dtype='complex64')
    for frame in range(csi.shape[0]):
        for sc in range(userCSI.shape[3]):
            if method == 'zf':
                sig_intf[frame,:,sc] = np.dot(data[frame,:,sc],np.linalg.pinv(userCSI[frame,:,:,sc]))
            else:
                sig_intf[frame,:,sc] = np.dot(data[frame,:,sc], np.transpose(np.conj(userCSI[frame,:,:,sc]),(1,0)))
    return sig_intf 

