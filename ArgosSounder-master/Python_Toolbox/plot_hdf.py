#!/usr/bin/python
"""
 Copyright (c) 2018-2019, Rice University 
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
 Author(s): Clay Shepard: cws@rice.edu
            Rahman Doost-Mohamamdy: doost@rice.edu
            Jian Ding: jd38@rice.edu

---------------------------------------------------------------------
 Plotting from HDF5 file
---------------------------------------------------------------------
"""

import sys
import struct
import numpy as np
#import configparser 
import os
import math
import h5py
import time
import datetime
import logging
import matplotlib.pyplot as plt
#import lts
import pdb
#from detect_peaks import detect_peaks
from channel_analysis import *

class plot_hdf:

    def __init__(self, filename, uplink_samples):
        self.plot_h5file = None
        self.filename = filename
        self.uplink = uplink_samples
    def get_h5(self):
        '''Get the most recent log file, open it if necessary.'''
        if (not self.plot_h5file) or (self.filename != self.plot_h5file.filename): #if it's closed, e.g. for the C version, open it
            print('Opening %s...' % self.filename)
            self.plot_h5file = h5py.File(self.filename,'r')
        return self.plot_h5file
	
    def plot_frame(self,samps=None,bs_ant=None,frame=None):
        '''Debug plot of real part of one frame that is useful for checking sync and AGC.'''
        if (samps is None):
            h5f = self.get_h5()
            samps = h5f['Pilot_Samples']
 
        ncell = samps.shape[1]
        frame = frame if frame is not None else samps.shape[0]//2 #random frame -- let's just choose one in the middle.
        fig,axes = plt.subplots(nrows=ncell,ncols=1)
        c = 0
        if ncell == 1: axes = [axes]
        for ax1 in axes: #for c in range(ncell):
            #ax1 = fig.add_subplot(1, ncell, c+1)
            ax1.set_ylabel('Real Part of Frame %d' % frame)
            ax1.set_title('Cell %d'%c)
            if bs_ant is None: bs_ant = range(samps.shape[2])
            for bs in bs_ant:
                #corr = np.correlate(samps[frame,c,bs,:,0]+samps[frame,c,bs,:,1]*1j, lts.genLTS(cp=0), 'full')
                ax1.plot(samps[frame,c,bs,:,0])
                #ax1.plot(corr)
            if c == ncell - 1:
                ax1.set_xlabel('Sample')
            c += 1
        fig.show()
		

    def plot_corr(self,samps=None,users=None,length=512+94,offset=256+82):
    	'''Debug plot that is useful for checking sync.'''
        if (samps is None or user is None):
            h5f = self.get_h5()
            pilots = h5f['/Data/Pilot_Samples']
            if users is None: users = pilots.shape[2]
        csi, iq = samps2csi(pilots,users,length,64, offset=offset)
        amps = np.mean(np.abs(iq[:,0,0,0,0,:]), axis=1)
        pilot_frames = [i for i in range(len(amps)) if amps[i] > 0.01]
        print("good frames len %d" % len(pilot_frames))
        ref_frame = pilot_frames[len(pilot_frames)//2] 
        #print("amp of reference frame %f", np.mean(np.abs(iq[ref_frame,0,0,0,0,:])))
        print("amp of reference frame %d is %f"% (ref_frame, amps[ref_frame]))
    	ncell = iq.shape[1]
    	c = 0
    	fig,axes = plt.subplots(nrows=ncell,ncols=1)
    	if ncell == 1: axes = [axes]
    	for ax1 in axes: #c in range(ncell):
    		#cellCSI = np.squeeze(csi[:,c,:,:,:,:], axis=1)
                cellCSI = csi[:,0,:,:,:,:]
                #cellCSI = np.delete(cellCSI,[2,3],3)
    		userCSI = np.mean(cellCSI[:,:,:,:,:],2)
                corr_total,sig_sc = calCorr(userCSI, np.transpose(np.conj(userCSI[ref_frame,:,:,:]),(1,0,2)))
                best_frames = [i for i in  pilot_frames if corr_total[i,0]>0.99]
                good_frames = [i for i in  pilot_frames if corr_total[i,0]>0.95]
                bad_frames = [i for i in   pilot_frames if corr_total[i,0]>0.9 and corr_total[i,0]<=0.94]
                worst_frames = [i for i in pilot_frames if corr_total[i,0]<0.9]
                print("num of best frames %d" % len(best_frames))
                print("num of good frames %d" % len(good_frames))
                print("num of bad frames   %d" % len(bad_frames))
                print("num of worst frames   %d" % len(worst_frames))
    		ax1.set_ylabel('Correlation with Frame %d' % ref_frame)
    		ax1.set_ylim([0,1.1])
    		ax1.set_title('Cell %d offset %d'%(c,offset))
    		#ax1.plot(corr_total[:,0])
                for u in range(users):
    		    ax1.plot(corr_total[pilot_frames,u])
    		ax1.set_xlabel('Frame')		
    		c += 1
    	plt.show()

    def plot_csi(self,samps=None, subcarrier=1):       
        if (samps is None):
            h5f = self.get_h5()
            pilots = h5f['/Data/Pilot_Samples']
        csi, iq, ofst, good_frames = samps2csi_findLTS(pilots,1,512+94)
        valid_idx = abs(csi[:,0,0,0,0,5])>=0.08 #check valid frames
        samples = np.arange(1,csi.shape[0]+1)
        fig,axes = plt.subplots(nrows=4,ncols=1)
        axes[0].set_title('Channel amplitude over time')
        axes[0].set_ylabel('Amplitude')
        axes[0].set_xlabel('Sample index')		
        axes[0].plot(np.abs(csi[valid_idx,0,0,0,:,subcarrier]))
		
        axes[1].set_title('Channel phase relative to ant 0')
        axes[1].set_ylabel('Phase')
        axes[1].set_xlabel('Sample index')		
        axes[1].plot(np.angle(csi[valid_idx,0,0,0,:,subcarrier]*np.conj(csi[valid_idx,0,0,0,0,subcarrier][:,np.newaxis])))		

        plt.show()

    def plot_csi_new(self,samps=None, users=None, frame=0, length=512+94, offset=82+256, subcarrier=1):       
        if (samps is None):
            h5f = self.get_h5()
            pilots = h5f['/Data/Pilot_Samples']
            if users is None: users = pilots.shape[2]
        csi, iq = samps2csi(pilots,users,length,64, offset=offset)
        amps = np.mean(np.abs(iq[:,0,0,0,0,:]), axis=1)
        pilot_frames = [i for i in range(len(amps)) if amps[i] > 0.01]
        ref_frame = pilot_frames[len(pilot_frames)//2] 
        cellCSI = csi[:,0,:,:,:,:]
    	userCSI = np.mean(cellCSI[:,:,:,:,:],2)
    	corr_total,sig_sc = calCorr(userCSI, np.transpose(np.conj(userCSI[ref_frame,:,:,:]),(1,0,2)))
        valid_idx = [i for i in  pilot_frames if corr_total[i,0]>0]#0.95]
        print("num of good frames %d" % len(valid_idx))
        fig,axes = plt.subplots(nrows=2,ncols=1)
        axes[0].set_title('Channel amplitude over time')
        axes[0].set_ylabel('Amplitude')
        axes[0].set_xlabel('Sample index')
        for i in range(csi.shape[4]):	
            axes[0].plot(np.abs(csi[valid_idx,0,0,0,i,subcarrier]), label="ant={0}".format(i))
        axes[0].legend(fontsize=10)	
        axes[1].set_title('Channel phase relative to ant 0')
        axes[1].set_ylabel('Phase')
        axes[1].set_xlabel('Sample index')		
        axes[1].plot(np.angle(csi[valid_idx,0,0,0,:,subcarrier]*np.conj(csi[valid_idx,0,0,0,0,subcarrier][:,np.newaxis])))
        #fig1,axes1 = plt.subplots(nrows=1,ncols=1)
        #axes1.set_title('CSI phase')
        #axes1.set_ylabel('Phase')
        #axes1.set_xlabel('Subcarrier')	
        #idx = valid_idx[0]
        #for i in range(csi.shape[4]):	
        #    axes1.plot(np.angle(csi[idx,0,0,0,i,:]))	

        plt.show()

    def analyze_frames(self,samps=None, frame1=0, frame2=1):
        plt.close("all")
        if (samps is None):
            h5f = self.get_h5()
            pilots = h5f['/Data/Pilot_Samples']
        csi, samps = samps2csi(pilots,1,512+94,512+94, offset=0,bound=0)
        offset = 0
	gold = lts.genLTS(upsample=1, cp=0)[:64]
        pilot1 = samps[frame1,0,0,0,:,:]
        pilot2 = samps[frame2,0,0,0,:,:]
        best, actual_ltss, peaks = lts.findLTS(pilot1[0,:])
        peaks_loc = detect_peaks(np.abs(peaks),mph=max(abs(peaks))/2)
        print(peaks_loc[0])
        best, actual_ltss, peaks = lts.findLTS(pilot1[1,:])
        peaks_loc = detect_peaks(np.abs(peaks),mph=max(abs(peaks))/2)
        print(peaks_loc[0])
        best, actual_ltss, peaks = lts.findLTS(pilot2[0,:])
        peaks_loc = detect_peaks(np.abs(peaks),mph=max(abs(peaks))/2)
        print(peaks_loc[0])
        best, actual_ltss, peaks = lts.findLTS(pilot2[1,:])
        peaks_loc = detect_peaks(np.abs(peaks),mph=max(abs(peaks))/2)
        print(peaks_loc[0])
        fig, axes = plt.subplots(nrows=4,ncols=1)
        axes[0].set_title('Cell 0')
        axes[0].set_ylabel('Real Part of Frame %d ant 0' % frame1)
        axes[0].plot(np.real(pilot1[0,:]))
        #axes[0].plot(np.real(np.correlate(pilot1[0,:],gold,'full') ))

        axes[1].set_ylabel('Real Part of Frame %d ant 1' % frame1)
        axes[1].plot(np.real(pilot1[1,:]))

        axes[2].set_ylabel('Real Part of Frame %d ant 0' % frame2)
        axes[2].plot(np.real(pilot2[0,:]))
        #axes[2].plot(np.real(np.correlate(pilot2[0,:],gold,'full') ))

        axes[3].set_ylabel('Real Part of Frame %d ant 1' % frame2)
        axes[3].plot(np.real(pilot2[1,:]))

        axes[3].set_xlabel('Sample')
        plt.show()
		
    def plot_constellation(self, frame=100, length=512+94+30, offset=82, fft_size=64, cp=0):
        plt.close("all")
        h5f = self.get_h5()
        
        samps = h5f['/Data/UplinkData'] # Frame, Cell, Symbol, Antenna, Sample     
        pilots = h5f['/Data/Pilot_Samples'] # Frame, Cell, User, Antenna, Sample     
        users = pilots.shape[2]
        csi,_ = samps2csi(pilots, users, length, fft_size, offset=offset, cp=cp)
        usersamps = np.reshape(samps, (samps.shape[0],samps.shape[1],samps.shape[2],samps.shape[3],samps.shape[4]/2, 2) )
        iq = (usersamps[:,:,:,:,offset+cp:offset+cp+fft_size,0]+usersamps[:,:,:,:,offset+cp:offset+cp+fft_size,1]*1j)*2**-15
        ffdata = np.empty(iq.shape,dtype='complex64')
        ffdata = np.fft.fftshift(np.fft.fft(iq, fft_size, 4),4)
        ffdata = np.delete(ffdata,[0,1,2,3,4,5,32,59,60,61,62,63],4) #remove zero subcarriers
        data = demult(np.squeeze(csi), np.squeeze(ffdata), method='conj')
        num_users = data.shape[1]
        fig,axes = plt.subplots(nrows=num_users+1,ncols=1)
        axes[0].set_title('Cell 0')
        axes[0].set_ylabel('Users Constellation in Frame %d' % frame)
        for i in range(num_users):
            axes[0].plot(np.real(data[frame,i,:]), np.imag(data[frame,i,:]), 'o')

        for i in range(data.shape[1]):
            axes[i+1].set_ylabel('Users Constellation of user %d' % (i+1))
            userdata = data[:,i,:].flatten()
            axes[i+1].plot(np.real(userdata), np.imag(userdata), 'o')
        plt.show()

    def plot_debug(self,samps=None, frame=100, length=512+94+30, uplink=0):
        plt.close("all")
        if (samps is None):
            h5f = self.get_h5()
            if uplink:
                # Frame, Cell, Symbol, Antenna, Sample   
                pilots = h5f['/Data/UplinkData']
                users = 1
            else:
                # Frame, Cell, User, Antenna, Sample   
                pilots = h5f['/Data/Pilot_Samples']
                users = pilots.shape[2]
        # samps Frame, Cell, User, Pilot Rep, Antenna, Sample"""
        # csi   rame, Cell, User, Pilot Rep, Antenna, Subcarrier"""
        csi, samps = samps2csi(pilots,users,length,length, offset=0, bound=0)
        fig,axes = plt.subplots(nrows=5,ncols=1)
        axes[0].set_title('Cell 0')
        axes[0].set_ylabel('Real Part of Frame %d ant 0' % frame)
        axes[0].plot(np.real(samps[frame,0,0,0,1,:]))

        axes[1].set_ylabel('Real Part of Frame %d ant 1' % frame)
        axes[1].plot(np.real(samps[frame,0,0,0,1,:]))

        axes[2].set_ylabel('Real Part of All Frames ant 0')
        axes[2].plot(np.real(samps[:,0,0,0,0,:]).flatten())

        axes[3].set_ylabel('Real Part of All Frames ant 1')
        axes[3].plot(np.real(samps[:,0,0,0,1,:]).flatten())

        axes[4].set_ylabel('Amplitude')
        for i in range(samps.shape[4]):
            axes[4].plot(np.mean(np.abs(samps[:,0,0,0,i,:]), axis=1).flatten())

        axes[4].set_xlabel('Sample')
        plt.show()
		
if __name__ == '__main__':
    if len(sys.argv) > 1:
        if sys.argv[1] == "-h":
            print('format: ./plot_hdf.py filename frame_length frame_index uplink(1)/pilot(0)')
            sys.exit(0)
        filename = sys.argv[1]
        frame_length = int(sys.argv[2])
        fnum = int(sys.argv[3])
        uplink = int(sys.argv[4])
        cc = plot_hdf(filename,uplink) 
        #cc.analyze_frames(frame1=103961,frame2=fnum)
        cc.plot_debug(frame=fnum, length=frame_length,uplink=uplink)
        if uplink==0:
            cc.plot_corr(length=frame_length)
            cc.plot_csi_new(length=frame_length, subcarrier=2)
        elif uplink==1:
            cc.plot_constellation(frame=fnum, length=frame_length, offset=82+80, cp=16)
    else:
        frame_length = 512+94+30
        cc = plot_hdf('test.hdf5', 0)
        cc.plot_debug(frame=0, length=frame_length)
        cc.plot_corr(frame=0, length=frame_length)
        cc.plot_csi_new(length=frame_length, subcarrier=2)
