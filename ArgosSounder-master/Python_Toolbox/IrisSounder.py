#!/usr/bin/python
"""
 Copyright (c) 2018-2019, Rice University 
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
 Author(s): Rahman Doost-Mohamamdy: doost@rice.edu

---------------------------------------------------------------------
 Demonstrates a simple channel sounding scenario with 2 Iris boards
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
import lts
import pdb 
import json
import preambles as Preambles
import matplotlib.pyplot as plt
import pickle
import scipy.io as sio 
from functools import partial

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
exit_plot = False
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
        if txSymNum == 0:
            continue
        sr = sdr.readStream(rxStream, [waveRxA, waveRxB], numSamps)
        if sr.ret > 0:
            txTime = sr.timeNs & 0xFFFFFFFF00000000
            txTime += (0x000000000 + (startSymbol << 16))
            if firstTime:
                print("first receive time 0x%X" % sr.timeNs)
                print("first transmit time 0x%X" % txTime)
                firstTime = False
        else:
            continue 
        flags = SOAPY_SDR_HAS_TIME #| SOAPY_SDR_END_BURST
        for j in range(txSymNum):
            txTimeNs = txTime #SoapySDR.ticksToTimeNs(txTime, rate)
            st = sdr.writeStream(txStream, [waveTx, waveTx], numSamps, flags, timeNs=txTimeNs)
            #sts = sdr.readStreamStatus(txStream)
            txTime += 0x10000
            if st.ret < 0:
                print("ret=%d,flags=%d,timeNs=0x%X,txTime=0x%X" % (st.ret, st.flags, st.timeNs, txTime))
    print("Exiting TX Thread")

def rx_thread(sdr, rxStream, numSamps, txSymNum, both_channels):
    global running
    fip = open('rxpilot.bin', 'wb')
    fid = open('rxdata.bin', 'wb')
    if both_channels:
        fip2 = open('rxpilotB.bin', 'wb')
        fid2 = open('rxdataB.bin', 'wb')
    rxFrNum = 0
    pilotSymNum = 2 if both_channels else 1
    waveRxA = np.array([0]*numSamps, np.uint32)
    waveRxB = np.array([0]*numSamps, np.uint32)
    flags = 0
    r1 = sdr.activateStream(rxStream, flags, 0)
    if r1<0:
        print("Problem activating stream #1")
    while (running):
        for i in range(pilotSymNum):
            sr = sdr.readStream(rxStream, [waveRxA, waveRxB], numSamps)
            if sr.ret < 0 or sr.ret > numSamps:
                print("readStream returned %d"%sr.ret)
            for i,a in enumerate(waveRxA):
                pickle.dump(a,fip)
                if both_channels: pickle.dump(waveRxB[i],fip2)
        for i in range(txSymNum):
            sr = sdr.readStream(rxStream, [waveRxA, waveRxB], numSamps)
            if sr.ret < 0 or sr.ret > numSamps:
                print("readStream returned %d"%sr.ret)
            for i,a in enumerate(waveRxA):
                pickle.dump(a,fid)
                if both_channels: pickle.dump(waveRxB[i],fid2)
        rxFrNum += 1
    fip.close()
    fid.close()
    if both_channels:
        fip2.close()
        fid2.close()
    sdr.deactivateStream(rxStream)
    sdr.closeStream(rxStream)
    print("Exiting RX Thread, Read %d Frames" % rxFrNum)

def siso_sounder(serial1, serial2, rate, freq, txgain, rxgain, bw, numSamps, numSyms, txSymNum, threshold, tx_advance, prefix_length, postfix_length, both_channels, wait_trigger, calibrate, record, use_trig, auto_tx_gain):
    global bsdr, msdr, txStreamM, rxStreamB
    print("setting %s as eNB and %s as UE" % (serial1, serial2))
    bsdr = SoapySDR.Device(dict(serial=serial1))
    msdr = SoapySDR.Device(dict(serial=serial2))

    #some default sample rates
    for i, sdr in enumerate([bsdr, msdr]):
        info = sdr.getHardwareInfo();
        print("%s settings on device %d" % (info["frontend"], i))
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
                sdr.setGain(SOAPY_SDR_TX, ch, 'PA1', 15) #[0,15]
                sdr.setGain(SOAPY_SDR_TX, ch, 'PA2', 0) #[0,15]
                sdr.setGain(SOAPY_SDR_TX, ch, 'PA3', 30) #[0,30]
            sdr.setGain(SOAPY_SDR_TX, ch, 'IAMP', 12) #[0,12]
            sdr.setGain(SOAPY_SDR_TX, ch, 'PAD', txgain) #[-52,0]

            if ("CBRS" in info["frontend"]):
                sdr.setGain(SOAPY_SDR_RX, ch, 'ATTN', 0) #[-18,0]
                sdr.setGain(SOAPY_SDR_RX, ch, 'LNA1', 30) #[0,33]
                sdr.setGain(SOAPY_SDR_RX, ch, 'LNA2', 17) #[0,17]
            sdr.setGain(SOAPY_SDR_RX, ch, 'LNA', rxgain) #[0,30]
            sdr.setGain(SOAPY_SDR_RX, ch, 'TIA', 0) #[0,12]
            sdr.setGain(SOAPY_SDR_RX, ch, 'PGA', 0) #[-12,19]
            sdr.setAntenna(SOAPY_SDR_RX, ch, "TRX")
            #sdr.setBandwidth(SOAPY_SDR_RX, ch, bw)
            #sdr.setBandwidth(SOAPY_SDR_TX, ch, bw)
            sdr.setDCOffsetMode(SOAPY_SDR_RX, ch, True)
        for ch in [0,1]:
            if calibrate:
                sdr.writeSetting(SOAPY_SDR_RX, ch, "CALIBRATE", 'SKLK')
                sdr.writeSetting(SOAPY_SDR_TX, ch, "CALIBRATE", '')

        sdr.writeRegister("IRIS30", RF_RST_REG, (1<<29) | 0x1)
        sdr.writeRegister("IRIS30", RF_RST_REG, (1<<29))
        sdr.writeRegister("IRIS30", RF_RST_REG, 0)
        if not both_channels:
            sdr.writeSetting("SPI_TDD_MODE", "SISO")
            sdr.writeSetting(SOAPY_SDR_RX, 1, 'ENABLE_CHANNEL', 'false')
            sdr.writeSetting(SOAPY_SDR_TX, 1, 'ENABLE_CHANNEL', 'false')
        else:
            sdr.writeSetting("SPI_TDD_MODE", "MIMO");
    #pdb.set_trace()
    msdr.writeRegister("IRIS30", TX_GAIN_CTRL, 0)  
    if use_trig:
        bsdr.writeSetting("SYNC_DELAYS", "")
        #bsdr.writeSetting("FPGA_DIQ_MODE", "PATTERN")
    else:
        msdr.writeRegister("ARGCOR", CORR_RST, 0x1) # reset corr
    #packet size
    symSamp = numSamps + prefix_length + postfix_length
    print("numSamps = %d"%symSamp)
    print("txSymNum = %d"%txSymNum)
    upsample = 1
    Ts = 1/rate
    s_freq = 1e6
    s_time_vals = np.array(np.arange(0,numSamps)).transpose()*Ts
    nb_data = np.exp(s_time_vals*1j*2*np.pi*s_freq).astype(np.complex64)*.25

    #create streams
    rxStreamM = msdr.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, [0, 1])  
    txStreamM = msdr.setupStream(SOAPY_SDR_TX, SOAPY_SDR_CF32, [0, 1])  
    if record:
        rxStreamB = bsdr.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, [0, 1])  

    # preambles to be sent from BS and correlated against in UE
    preambles_bs, highest_peak_to_second_ratio_bs = Preambles.getPreambles('gold_ifft', 128, 0, upsample=1) #the base station may upsample, but the mobiles won't
    preambles = preambles_bs[:,::upsample] #the correlators can run at lower rates, so we only need the downsampled signal.
    
    beacon = preambles[0,:]*.25
    coe = cfloat2uint32(np.conj(beacon), order='QI') # FPGA correlator takes coefficients in QI order
    ltsSym = lts.genLTS(upsample=1,cp=0)  
    pad1 = np.array([0]*(prefix_length), np.complex64) # to comprensate for front-end group delay
    pad2 = np.array([0]*(postfix_length), np.complex64) # to comprensate for rf path delay
    wb_pilot = np.tile(ltsSym,numSamps/len(ltsSym)).astype(np.complex64)*.5
    wbz = np.array([0]*(symSamp), np.complex64)
    wb_pilot1 = np.concatenate([pad1,wb_pilot,pad2])  
    wb_pilot2 = wbz #wb_pilot1 if both_channels else wbz
    bcnz = np.array([0]*(symSamp-prefix_length-len(beacon)), np.complex64)  
    beacon1 = np.concatenate([pad1,beacon,bcnz])
    beacon2 = wbz #beacon1 if both_channels else wbz   

    bsched = "PGR"+''.join("G"*(numSyms-txSymNum-4))+''.join("R"*txSymNum)+"G" 
    msched = "GGP"+''.join("G"*(numSyms-txSymNum-4))+''.join("T"*txSymNum)+"G"
    if both_channels:
        bsched = "PGRR"+''.join("G"*(numSyms-txSymNum-5))+''.join("R"*txSymNum)+"G" 
        msched = "GGPP"+''.join("G"*(numSyms-txSymNum-5))+''.join("T"*txSymNum)+"G"
    print("Node 1 schedule %s " % bsched) 
    print("Node 2 schedule %s " % msched)
    bconf = {"tdd_enabled": True, "trigger_out": False, "symbol_size" : symSamp, "frames": [bsched]}
    mconf = {"tdd_enabled": True, "trigger_out": not use_trig, "wait_trigger": wait_trigger, "dual_pilot": both_channels, "symbol_size" : symSamp, "frames": [msched]}
    #mconf = {"tdd_enabled": True, "trigger_out": not use_trig, "wait_trigger": True, "symbol_size" : symSamp, "frames": [msched]}
    bsdr.writeSetting("TDD_CONFIG", json.dumps(bconf))
    msdr.writeSetting("TDD_CONFIG", json.dumps(mconf))

    for sdr in [bsdr, msdr]:
        sdr.writeSetting("TX_SW_DELAY", str(30))

    if not use_trig:
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

        # DEV: ueTrigTime = 153 (prefix_length=0), CBRS: ueTrigTime = 235 (prefix_length=82), tx_advance=prefix_length, corr delay is 17 cycles
        ueTrigTime = prefix_length + len(beacon) + postfix_length + 17 + tx_advance 
        sf_start = ueTrigTime/symSamp
        sp_start = ueTrigTime%symSamp
        print("UE starting symbol and sample count (%d, %d)" % (sf_start, sp_start))
        msdr.setHardwareTime(SoapySDR.ticksToTimeNs((sf_start<<16) | sp_start, rate),"TRIGGER") # make sure to set this after TDD mode is enabled "writeSetting("TDD_CONFIG", ..."

    msdr.writeSetting("TDD_MODE", "true")
    bsdr.writeSetting("TDD_MODE", "true")

    replay_addr = 0
    bsdr.writeRegisters("TX_RAM_A", replay_addr, cfloat2uint32(beacon1, order='IQ').tolist())
    bsdr.writeRegisters("TX_RAM_B", replay_addr, cfloat2uint32(beacon2, order='IQ').tolist())

    msdr.writeRegisters("TX_RAM_A", replay_addr, cfloat2uint32(wb_pilot1, order='IQ').tolist())
    msdr.writeRegisters("TX_RAM_B", replay_addr, cfloat2uint32(wbz, order='IQ').tolist())
    if both_channels:
        msdr.writeRegisters("TX_RAM_A", replay_addr+2048, cfloat2uint32(wbz, order='IQ').tolist())
        msdr.writeRegisters("TX_RAM_B", replay_addr+2048, cfloat2uint32(wb_pilot2, order='IQ').tolist())

    if not use_trig:    
        msdr.writeRegister("IRIS30", CORR_CONF, int("00004011", 16)) # enable the correlator, with inputs from adc
    signal.signal(signal.SIGINT, partial(signal_handler, rate, numSyms))
    bsdr.writeSetting("TRIGGER_GEN", "")
    txth = threading.Thread(target=tx_thread, args=(msdr, rate, txStreamM, rxStreamM, nb_data, symSamp, numSyms, txSymNum, numSyms-txSymNum-1))
    txth.start()
    if record:
        rxth = threading.Thread(target=rx_thread, args=(bsdr, rxStreamB, symSamp, txSymNum, both_channels))
        rxth.start()
    signal.pause()

def signal_handler(rate, numSyms, signal, frame):
    global bsdr, msdr, running, txStreamM, rxStreamB
    msdr.writeRegister("IRIS30", CORR_CONF, 0) # stop mobile correlator first, to prevent from the tdd manager going
    msdr.writeRegister("IRIS30", TX_GAIN_CTRL, 0)  
    # stop tx/rx threads
    running = False

    print("printing number of frames")
    print("NB 0x%X" % SoapySDR.timeNsToTicks(bsdr.getHardwareTime(""), rate))
    print("UE 0x%X" % SoapySDR.timeNsToTicks(msdr.getHardwareTime(""), rate))
    #print("UE SCNT: 0x%X" % msdr.readRegister("ARGCOR", CORR_SCNT))
    # ADC_rst, stops the tdd time counters
    for sdr in [bsdr, msdr]:
        sdr.writeRegister("IRIS30", RF_RST_REG, (1<<29)| 0x1)
        sdr.writeRegister("IRIS30", RF_RST_REG, (1<<29))
        sdr.writeRegister("IRIS30", RF_RST_REG, 0)
    for i in range(numSyms):
        msdr.writeRegister("RFCORE", SCH_ADDR_REG, i) # subframe 0
        msdr.writeRegister("RFCORE", SCH_MODE_REG, 0) # 01 replay
        bsdr.writeRegister("RFCORE", SCH_ADDR_REG, i) # subframe 0
        bsdr.writeRegister("RFCORE", SCH_MODE_REG, 0) # 01 replay
    bsdr.writeRegister("RFCORE", TDD_CONF_REG, 0) 
    msdr.writeRegister("RFCORE", TDD_CONF_REG, 0) 
    msdr.writeSetting("TDD_MODE", "false")
    bsdr.writeSetting("TDD_MODE", "false")
    bsdr = None
    msdr = None
    if exit_plot:
        fig = plt.figure(figsize=(20, 8), dpi=100)
        ax1 = fig.add_subplot(2, 1, 1)
        ax2 = fig.add_subplot(2, 1, 2)
        pilot = uint32tocfloat(read_from_file("rxpilot", 256))
        rxdata = uint32tocfloat(read_from_file("rxdata", 256))
        ax1.plot(np.real(pilot), label='pilot i')
        ax1.plot(np.imag(pilot), label='pilot q')
        ax2.plot(np.real(rxdata), label='rx data i')
        ax2.plot(np.imag(rxdata), label='rx data q')
        plt.show()
    sys.exit(0)

def main():
    parser = OptionParser()
    parser.add_option("--serial1", type="string", dest="serial1", help="serial number of the master device", default="")
    parser.add_option("--serial2", type="string", dest="serial2", help="serial number of the slave device", default="")
    parser.add_option("--rate", type="float", dest="rate", help="Tx sample rate", default=5e6)
    parser.add_option("--txgain", type="float", dest="txgain", help="Optional Tx gain (dB)", default=20.0)
    parser.add_option("--rxgain", type="float", dest="rxgain", help="Optional Rx gain (dB)", default=20.0)
    parser.add_option("--freq", type="float", dest="freq", help="Optional Tx freq (Hz)", default=3.6e9)
    parser.add_option("--bw", type="float", dest="bw", help="Optional Analog Filter Bandwidth (Hz)", default=30e6)
    parser.add_option("--numSamps", type="int", dest="numSamps", help="Num samples to receive", default=512)
    parser.add_option("--prefix-length", type="int", dest="prefix_length", help="prefix padding length for beacon and pilot", default=82)
    parser.add_option("--postfix-length", type="int", dest="postfix_length", help="postfix padding length for beacon and pilot", default=68)
    parser.add_option("--numSyms", type="int", dest="numSyms", help="Number of symbols in one frame", default=20)
    parser.add_option("--txSymNum", type="int", dest="txSymNum", help="Number of tx symbols in one frame", default=0)
    parser.add_option("--corr-threshold", type="int", dest="threshold", help="Correlator Threshold Value", default=128)
    parser.add_option("--ue-tx-advance", type="int", dest="tx_advance", help="sample advance for tx vs rx", default=68)
    parser.add_option("--both-channels", action="store_true", dest="both_channels", help="transmit from both channels",default=False)
    parser.add_option("--calibrate", action="store_true", dest="calibrate", help="transmit from both channels",default=False)
    parser.add_option("--use-trig", action="store_true", dest="use_trig", help="uses chain triggers for synchronization",default=False)
    parser.add_option("--wait-trigger", action="store_true", dest="wait_trigger", help="wait for a trigger to start a frame",default=False)
    parser.add_option("--auto-tx-gain", action="store_true", dest="auto_tx_gain", help="automatically go over tx gains",default=False)
    parser.add_option("--record", action="store_true", dest="record", help="record received pilots and data",default=True)
    (options, args) = parser.parse_args()
    siso_sounder(
        serial1=options.serial1,
        serial2=options.serial2,
        rate=options.rate,
        freq=options.freq,
        txgain=options.txgain,
        rxgain=options.rxgain,
        bw=options.bw,
        numSamps=options.numSamps,
        numSyms=options.numSyms,
        txSymNum=options.txSymNum,
        threshold=options.threshold,
        tx_advance=options.tx_advance,
        prefix_length=options.prefix_length,
        postfix_length=options.postfix_length,
        both_channels=options.both_channels,
        wait_trigger=options.wait_trigger,
        calibrate=options.calibrate,
        record=options.record,
        auto_tx_gain=options.auto_tx_gain,
        use_trig=options.use_trig
    )

if __name__ == '__main__':
    main()
 
