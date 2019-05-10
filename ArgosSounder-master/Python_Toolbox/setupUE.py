#!/usr/bin/python
"""
 Copyright (c) 2018-2019, Rice University 
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
 Author(s): Rahman Doost-Mohamamdy: doost@rice.edu

---------------------------------------------------------------------
 Setup an Iris board as a user for channel sounding
---------------------------------------------------------------------
"""

import random as rd
import threading
import SoapySDR
from SoapySDR import * #SOAPY_SDR_ constants
from optparse import OptionParser
import numpy as np
import time
import os
import sys
import signal
import math
import pdb 
import json
import preambles as Preambles
import pickle
from functools import partial
import lts
import ofdm

# CORRTHRESHOLDING REGS
CORR_THRESHOLD = 0x4
CORR_RST = 0x0
CORR_SCNT = 0x8
CORR_CONF = 60
# TDD Register Set
RF_RST_REG = 48
TDD_CONF_REG = 120
SCH_ADDR_REG = 136
SCH_MODE_REG = 140
TX_GAIN_CTRL = 88

running = True
record = True
bsdr = None
msdr = None
txStreamM = None
rxStreamB = None
rxStreamM = None

def cfloat2uint32(arr, order='IQ'):
        arr_i = (np.real(arr) * 32767).astype(np.uint16)
        arr_q = (np.imag(arr) * 32767).astype(np.uint16)
        if order == 'IQ':
            return np.bitwise_or(arr_q ,np.left_shift(arr_i.astype(np.uint32), 16))
        else:
            return np.bitwise_or(arr_i ,np.left_shift(arr_q.astype(np.uint32), 16))

def uint32tocfloat(arr, order='IQ'):
	arr_hi = ((np.right_shift(arr, 16).astype(np.int16))/32768.0)
	arr_lo = (np.bitwise_and(arr, 0xFFFF).astype(np.int16))/32768.0
        if order == 'IQ':
	    return (arr_hi + 1j*arr_lo).astype(np.complex64)
        else:
	    return (arr_lo + 1j*arr_hi).astype(np.complex64)

def write_to_file(name,arr,num_bits=12):
    """Save complex numpy array val to files prefixed with name in binary twos-complement binary format with num_bits."""
    fi = open(name+'.bin', 'wb')
    for a in arr:
        #fi.write(np.binary_repr(a,width=num_bits))
        pickle.dump(a,fi)
    fi.close()

def write_to_file_text(name,arr,dtype="complex"):
    """Save complex numpy array val to text file."""
    fi = open(name+'.txt', 'wb')
    for a in arr:
        if dtype=="complex":
            fi.write("%f %f\n"%(np.real(a),np.imag(a)))
        else:
            fi.write("%d\n"%a)
    fi.close()

def read_from_file(name,leng,num_bits=12):
    """Save complex numpy array val to files prefixed with name in binary twos-complement binary format with num_bits."""
    fi = open(name+'.bin', 'rb')
    arr = np.array([0]*leng, np.uint32)
    for a in range(leng):
        #fi.write(np.binary_repr(a,width=num_bits))
        arr[a] = pickle.load(fi)
    fi.close()
    return arr

def tx_thread(sdr, rate, txStream, rxStream, waveTx, numSamps, numSyms, txSymNum, startSymbol):
    global running
    firstTime = True
    waveRxA = np.array([0]*numSamps, np.uint32)
    waveRxB = np.array([0]*numSamps, np.uint32)
    flags = 0
    sdr.activateStream(txStream)
    sdr.activateStream(rxStream, flags, 0)
    while(running):
        sr = sdr.readStream(rxStream, [waveRxA, waveRxB], numSamps)
        if sr.ret == numSamps:
            txTime = sr.timeNs & 0xFFFFFFFF00000000
            txTime += (0x500000000 + (startSymbol << 16))
            if firstTime:
                print("first receive time 0x%X" % sr.timeNs)
                print("first transmit time 0x%X" % txTime)
                firstTime = False
        else:
            continue
        flags = SOAPY_SDR_HAS_TIME #| SOAPY_SDR_END_BURST
        for j in range(txSymNum):
            txTimeNs = txTime #SoapySDR.ticksToTimeNs(txTime, rate)
            if j == txSymNum-1:
                flags |= SOAPY_SDR_END_BURST 
            st = sdr.writeStream(txStream, [waveTx, waveTx], numSamps, flags, timeNs=txTimeNs)
            sts = sdr.readStreamStatus(txStream)
            if sts.ret != 0:
                print(SoapySDR.errToStr(sts.ret))
            txTime += 0x10000
    sdr.deactivateStream(rxStream)
    sdr.deactivateStream(txStream)
    sdr.closeStream(rxStream)
    sdr.closeStream(txStream)
    print("Exiting TX Thread")


def setupUE(args, serial, rate, freq, txgain, rxgain, numSamps, numSyms, pilotSymbol, txSymbol, txSymNum, threshold, tx_advance, prefix_length, postfix_length, cp, calibrate, both_channels, wait_trigger, auto_tx_gain):
    global msdr, txStreamM, rxStreamM
    msdr = SoapySDR.Device(dict(serial=serial))

    #some default sample rates
    for sdr in [msdr]:
        info = sdr.getHardwareInfo();
        print("%s settings" % info["frontend"])
        for ch in [0,1]:
            sdr.setSampleRate(SOAPY_SDR_TX, ch, rate)
            sdr.setSampleRate(SOAPY_SDR_RX, ch, rate)
            #sdr.setFrequency(SOAPY_SDR_TX, ch, freq)
            #sdr.setFrequency(SOAPY_SDR_RX, ch, freq)
            sdr.setFrequency(SOAPY_SDR_TX, ch, 'RF', freq-.75*rate)
            sdr.setFrequency(SOAPY_SDR_RX, ch, 'RF', freq-.75*rate)
            sdr.setFrequency(SOAPY_SDR_TX, ch, 'BB', .75*rate)
            sdr.setFrequency(SOAPY_SDR_RX, ch, 'BB', .75*rate)
            if ("CBRS" in info["frontend"]):
                sdr.setGain(SOAPY_SDR_TX, ch, 'ATTN', 0) #[-18,0] by 3
                sdr.setGain(SOAPY_SDR_TX, ch, 'PA1', 15) #[0|15]
                sdr.setGain(SOAPY_SDR_TX, ch, 'PA2', 0) #[0|15]
                sdr.setGain(SOAPY_SDR_TX, ch, 'PA3', 30) #[0|30]
            if ("UHF" in info["frontend"]):
                sdr.setGain(SOAPY_SDR_TX, ch, 'ATTN', 0) #[-18,0] by 3
            sdr.setGain(SOAPY_SDR_TX, ch, 'IAMP', 0) #[0,12]
            sdr.setGain(SOAPY_SDR_TX, ch, 'PAD', txgain) #[0,52]

            if ("CBRS" in info["frontend"]):
                sdr.setGain(SOAPY_SDR_RX, ch, 'ATTN', 0) #[-18,0]
                sdr.setGain(SOAPY_SDR_RX, ch, 'LNA1', 30) #[0,33]
                sdr.setGain(SOAPY_SDR_RX, ch, 'LNA2', 17) #[0,17]
            if ("UHF" in info["frontend"]):
                sdr.setGain(SOAPY_SDR_RX, ch, 'ATTN1', -6) #[-18,0]
                sdr.setGain(SOAPY_SDR_RX, ch, 'ATTN2', -12) #[-18,0]
            sdr.setGain(SOAPY_SDR_RX, ch, 'LNA', rxgain) #[0,30]
            sdr.setGain(SOAPY_SDR_RX, ch, 'TIA', 0) #[0,12]
            sdr.setGain(SOAPY_SDR_RX, ch, 'PGA', 0) #[-12,19]
            sdr.setAntenna(SOAPY_SDR_RX, ch, "TRX")
            #sdr.setBandwidth(SOAPY_SDR_TX, ch, 30e6)
            #sdr.setBandwidth(SOAPY_SDR_RX, ch, 30e6)
            sdr.setDCOffsetMode(SOAPY_SDR_RX, ch, True)
        if calibrate:
            for ch in [0,1]:
                sdr.writeSetting(SOAPY_SDR_RX, ch, "CALIBRATE", 'SKLK')
                sdr.writeSetting(SOAPY_SDR_TX, ch, "CALIBRATE", 'SKLK')
        #if not both_channels:
        #    sdr.writeSetting("SPI_TDD_MODE", "SISO")
        #    sdr.writeSetting(SOAPY_SDR_RX, 1, 'ENABLE_CHANNEL', 'false')
        #    sdr.writeSetting(SOAPY_SDR_TX, 1, 'ENABLE_CHANNEL', 'false')
        #else:
        #    sdr.writeSetting("SPI_TDD_MODE", "MIMO")

        sdr.writeRegister("IRIS30", RF_RST_REG, (1<<29) | 0x1)
        sdr.writeRegister("IRIS30", RF_RST_REG, (1<<29))
        sdr.writeRegister("IRIS30", RF_RST_REG, 0)
    msdr.writeRegister("ARGCOR", CORR_RST, 0x1) # reset corr
    msdr.writeRegister("IRIS30", TX_GAIN_CTRL, 0) 
    minfo = msdr.getHardwareInfo()
    #packet size
    symSamp = numSamps + prefix_length + postfix_length
    print("symSamp = %d"%symSamp)
    print("txSymNum = %d"%txSymNum)

    upsample = 1
    # preambles to be sent from BS and correlated against in UE
    preambles_bs, highest_peak_to_second_ratio_bs = Preambles.getPreambles('gold_ifft', 128, 0, upsample=1) #the base station may upsample, but the mobiles won't
    preambles = preambles_bs[:,::upsample] #the correlators can run at lower rates, so we only need the downsampled signal.
    
    beacon = preambles[0,:]*.25
    coe = cfloat2uint32(np.conj(beacon), order='QI') # FPGA correlator takes coefficients in QI order
    ltsSym = lts.genLTS(upsample=upsample,cp=1 if cp else 0)  
    pad1 = np.array([0]*(prefix_length), np.complex64) # to comprensate for front-end group delay
    pad2 = np.array([0]*(postfix_length), np.complex64) # to comprensate for rf path delay
    wb_pilot = np.tile(ltsSym,numSamps/len(ltsSym)).astype(np.complex64)*.5
    wbz = np.array([0]*(symSamp), np.complex64)
    wb_pilot1 = np.concatenate([pad1,wb_pilot,pad2]) 
    wb_pilot2 = wb_pilot1 if both_channels else wbz
    
    Ts = 1/rate
    s_freq = 1e6
    s_time_vals = np.array(np.arange(0,symSamp)).transpose()*Ts
    nb_data = np.exp(s_time_vals*1j*2*np.pi*s_freq).astype(np.complex64)*.25

    rand_data = [1,0,1,1,0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, \
				0, 1, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 1, \
				0, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1, 0 ,1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, \
				0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1 ,0, 1, 0, 0, \
				0, 0, 1, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 0, \
				0, 1, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 1, 0, 0, 1, 0, 1, 0, 1, 1, 1, 1, 0, 1, 0, 1, 0, 0, 1, 0, \
				0, 1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 1, 1, 1, 1, 0, 0, 0, 1, 1, 0, 0, 1, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 1]
    #data_16qam = genDATA(np.random.randint(2, size=64*4),0)
    data_16qam = ofdm.genDATAQPSK(rand_data,0)
    if not cp: data_16qam[16:]
    wb_data = np.tile(data_16qam,numSamps/len(data_16qam)).astype(np.complex64)*.5
    wb_data = np.concatenate([pad1,wb_data,pad2])

    if txSymNum > 0:
        txStreamM = msdr.setupStream(SOAPY_SDR_TX, SOAPY_SDR_CF32, [0, 1])  
        rxStreamM = msdr.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, [0, 1]) 
 
    msdr.writeRegister("IRIS30", CORR_CONF, int("00004001", 16)) # enable the correlator, with zeros as inputs 
    for i in range(128):
        msdr.writeRegister("ARGCOE", i*4, 0)
    time.sleep(0.1)
    msdr.writeRegister("ARGCOR", CORR_THRESHOLD, int(threshold))
    msdr.writeRegister("ARGCOR", CORR_RST, 0x1) # reset corr
    msdr.writeRegister("ARGCOR", CORR_RST, 0x0) # unrst corr 
    for i in range(128):
        msdr.writeRegister( "ARGCOE", i*4, int(coe[i]))
    if auto_tx_gain:
        max_gain = int(txgain)
        min_gain = max(0, max_gain-15)
        gain_reg = 0xF000 | (max_gain & 0x3F) << 6 | (min_gain & 0x3F)
        print("gain reg 0x%X" % gain_reg)
        msdr.writeRegister("IRIS30", TX_GAIN_CTRL, gain_reg) # [15] en, [14] mode, [13:12] step, [11:6] stop, [5:0] start 
    
    if both_channels:
        pilotSymNum = 2
        msched = ''.join("G"*(pilotSymbol-2))+"PP"
    else:
        pilotSymNum = 1
        msched = ''.join("G"*(pilotSymbol-2))+"P"
    if txSymNum > 0:
        msched += ''.join("G"*(txSymbol-pilotSymbol-pilotSymNum))+''.join("T"*txSymNum)+''.join("G"*(numSyms-txSymNum-txSymbol))
        msched = "GR" + msched
    else:
        msched += ''.join("G"*(numSyms-pilotSymbol-pilotSymNum))
        msched = "GG" + msched
    print("Client schedule %s " % msched)
    mconf = {"tdd_enabled": True, "trigger_out": True, "wait_trigger": True, "dual_pilot": both_channels, "symbol_size" : symSamp, "frames": [msched]}
    msdr.writeSetting("TDD_CONFIG", json.dumps(mconf))

    # DEV: ueTrigTime = 153 (prefix_length=0), CBRS: ueTrigTime = 235 (prefix_length=82), tx_advance=prefix_length
    ueTrigTime = prefix_length + len(beacon) + postfix_length + 17 + tx_advance 
    sf_start = ueTrigTime/symSamp
    sp_start = ueTrigTime%symSamp
    print("UE starting symbol and sample count (%d, %d)" % (sf_start, sp_start))
    msdr.setHardwareTime(SoapySDR.ticksToTimeNs((sf_start<<16) | sp_start, rate),"TRIGGER") # make sure to set this after TDD mode is enabled "writeSetting("TDD_CONFIG", ..."

    for sdr in [msdr]:
        sdr.writeSetting("TX_SW_DELAY", str(30))
    msdr.writeSetting("TDD_MODE", "true")

    replay_addr = 0
    if both_channels:
        msdr.writeRegisters("TX_RAM_A", replay_addr+(pilotSymbol%2)*2048, cfloat2uint32(wb_pilot1, order='QI').tolist())
        msdr.writeRegisters("TX_RAM_B", replay_addr+((pilotSymbol+1)%2)*2048, cfloat2uint32(wb_pilot1, order='QI').tolist())
    else:
        msdr.writeRegisters("TX_RAM_A", replay_addr, cfloat2uint32(wb_pilot1, order='QI').tolist())
        msdr.writeRegisters("TX_RAM_B", replay_addr, cfloat2uint32(wb_pilot2, order='QI').tolist())

    #pdb.set_trace()
    msdr.writeRegister("IRIS30", CORR_CONF, int("00004011", 16)) # enable the correlator, with inputs from adc
    signal.signal(signal.SIGINT, partial(signal_handler, rate, numSyms, txSymNum))
    if txSymNum>0:
        txth = threading.Thread(target=tx_thread, args=(msdr, rate, txStreamM, rxStreamM, wb_data, symSamp, numSyms, txSymNum, txSymbol))
        txth.start()
    #signal.pause()
    num_trig = 0
    while True:
        time.sleep(0.25)
        t = SoapySDR.timeNsToTicks(msdr.getHardwareTime(""),rate) >> 32 #trigger count is top 32 bits.
        print("%d new triggers, %d total" % (t - num_trig, t))
        num_trig = t

def signal_handler(rate, numSyms, txSymNum, signal, frame):
    global msdr, running, txStreamM, rxStreamM
    msdr.writeRegister("IRIS30", CORR_CONF, 0) # stop mobile correlator first, to prevent from the tdd manager going
    msdr.writeRegister("IRIS30", TX_GAIN_CTRL, 0) 
    # stop tx/rx threads
    if txSymNum>0:
        running = False
        time.sleep(1)
    print("printing number of frames")
    print("0x%X" % SoapySDR.timeNsToTicks(msdr.getHardwareTime(""),rate))
    # ADC_rst, stops the tdd time counters
    msdr.writeRegister("IRIS30", RF_RST_REG, (1<<29)| 0x1)
    msdr.writeRegister("IRIS30", RF_RST_REG, (1<<29))
    msdr.writeRegister("IRIS30", RF_RST_REG, 0)
    #msdr.writeSetting("TDD_MODE", str(0x0)) #disable TDD mode
    for i in range(numSyms):
        msdr.writeRegister("RFCORE", SCH_ADDR_REG, i) # subframe 0
        msdr.writeRegister("RFCORE", SCH_MODE_REG, 0) # 01 replay
    msdr.writeSetting("TDD_MODE", "false")
    msdr = None
    sys.exit(0)

def main():
    parser = OptionParser()
    parser.add_option("--args", type="string", dest="args", help="device factory arguments", default="")
    parser.add_option("--serial", type="string", dest="serial", help="serial number of the device", default="")
    parser.add_option("--rate", type="float", dest="rate", help="Tx sample rate", default=5e6)
    parser.add_option("--txgain", type="float", dest="txgain", help="Optional Tx gain (dB)", default=20.0)
    parser.add_option("--rxgain", type="float", dest="rxgain", help="Optional Rx gain (dB)", default=20.0)
    parser.add_option("--freq", type="float", dest="freq", help="Optional Tx freq (Hz)", default=3.6e9)
    parser.add_option("--numSamps", type="int", dest="numSamps", help="Num samples to receive", default=512)
    parser.add_option("--numSyms", type="int", dest="numSyms", help="Number of symbols in one frame", default=20)
    parser.add_option("--pilotSymbol", type="int", dest="pilotSymbol", help="index (zero-based) of pilot symbol in one frame", default=2)
    parser.add_option("--txSymbol", type="int", dest="txSymbol", help="index (zero-based) of starting tx symbol in one frame", default=3)
    parser.add_option("--txSymNum", type="int", dest="txSymNum", help="Number of tx symbols in one frame", default=0)
    parser.add_option("--corr-threshold", type="int", dest="threshold", help="Correlator Threshold Value", default=128)
    parser.add_option("--tx-advance", type="int", dest="tx_advance", help="sample advance for tx vs rx", default=68)
    parser.add_option("--prefix-pad", type="int", dest="prefix_length", help="prefix padding length for beacon and pilot", default=82)
    parser.add_option("--postfix-pad", type="int", dest="postfix_length", help="postfix padding length for beacon and pilot", default=68)
    parser.add_option("--cp", action="store_true", dest="cp", help="add cyclic prefix to both data and pilots",default=False)
    parser.add_option("--both-channels", action="store_true", dest="both_channels", help="transmit from both channels",default=False)
    parser.add_option("--calibrate", action="store_true", dest="calibrate", help="Performs radio calibration during initialization",default=False)
    parser.add_option("--auto-tx-gain", action="store_true", dest="auto_tx_gain", help="automatically go over tx gains",default=False)
    parser.add_option("--wait-trigger", action="store_true", dest="wait_trigger", help="Indicates if each frame will be initiated with a trigger",default=False)
    (options, args) = parser.parse_args()
    setupUE(
        args=options.args,
        serial=options.serial,
        rate=options.rate,
        freq=options.freq,
        txgain=options.txgain,
        rxgain=options.rxgain,
        numSamps=options.numSamps,
        numSyms=options.numSyms,
        pilotSymbol=options.pilotSymbol,
        txSymbol=options.txSymbol,
        txSymNum=options.txSymNum,
        threshold=options.threshold,
        tx_advance=options.tx_advance,
        prefix_length=options.prefix_length,
        postfix_length=options.postfix_length,
        cp=options.cp,
        calibrate=options.calibrate,
        both_channels=options.both_channels,
        wait_trigger=options.wait_trigger,
        auto_tx_gain=options.auto_tx_gain,
    )

if __name__ == '__main__':
    main()
 
