#!/usr/bin/python
import SoapySDR
from SoapySDR import * #SOAPY_SDR_ constants
from optparse import OptionParser
import numpy as np
import time
import os
import sys
import math
import json
import matplotlib.pyplot as plt

# TDD Register Set
RF_RST_REG = 48
TDD_CONF_REG = 120
SCH_ADDR_REG = 136
SCH_MODE_REG = 140

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

def siso_tdd_burst(serial1, serial2, rate, freq, txgain, rxgain, numSamps, prefix_pad, postfix_pad, both_channels, wideband):
    bsdr = SoapySDR.Device(dict(driver='iris',serial=serial1))
    msdr = SoapySDR.Device(dict(driver='iris',serial=serial2))

    #some default sample rates
    for i, sdr in enumerate([bsdr, msdr]):
        info = sdr.getHardwareInfo();
        print("%s settings on device %d" % (info["frontend"], i))
        for ch in [0, 1]:
            sdr.setFrequency(SOAPY_SDR_TX, ch, freq)
            sdr.setFrequency(SOAPY_SDR_RX, ch, freq)
            sdr.setSampleRate(SOAPY_SDR_TX, ch, rate)
            sdr.setSampleRate(SOAPY_SDR_RX, ch, rate)
            if ("CBRS" in info["frontend"]):
                sdr.setGain(SOAPY_SDR_TX, ch, 'ATTN', 0) #[-18,0] by 3
                sdr.setGain(SOAPY_SDR_TX, ch, 'PA1', 15) #[0|15]
                sdr.setGain(SOAPY_SDR_TX, ch, 'PA2', 0) #[0|15]
                sdr.setGain(SOAPY_SDR_TX, ch, 'PA3', 30) #[0|30]
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
            sdr.setDCOffsetMode(SOAPY_SDR_RX, ch, True)
        #for ch in [0,1]:
        #    sdr.writeSetting(SOAPY_SDR_RX, ch, "CALIBRATE", '')
        #    sdr.writeSetting(SOAPY_SDR_TX, ch, "CALIBRATE", '')

        #sdr.writeSetting(SOAPY_SDR_RX, 1, 'ENABLE_CHANNEL', 'false')
        #sdr.writeSetting(SOAPY_SDR_TX, 1, 'ENABLE_CHANNEL', 'false')

        sdr.writeRegister("IRIS30", RF_RST_REG, (1<<29) | 0x1)
        sdr.writeRegister("IRIS30", RF_RST_REG, (1<<29))
        sdr.writeRegister("IRIS30", RF_RST_REG, 0)

    bsdr.writeSetting("SYNC_DELAYS", "")
    symSamp = numSamps + prefix_pad + postfix_pad
    print("numSamps = %d"%numSamps)
    print("symSamps = %d"%symSamp)

    waveRxA1 = np.array([0]*symSamp, np.uint32)
    waveRxB1 = np.array([0]*symSamp, np.uint32)
    waveRxA2 = np.array([0]*symSamp, np.uint32)
    waveRxB2 = np.array([0]*symSamp, np.uint32)
    #bsdr.writeSetting("FPGA_DIQ_MODE", "PATTERN")
    #msdr.writeSetting("FPGA_DIQ_MODE", "PATTERN")

    # CS16 makes sure the 4-bit lsb are samples are being sent
    rxStreamB = bsdr.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, [0, 1],  dict(WIRE=SOAPY_SDR_CS16))  
    rxStreamM = msdr.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, [0, 1],  dict(WIRE=SOAPY_SDR_CS16))  

    if wideband:
        ltsSym = lts.genLTS(upsample=1,cp=0)  
        pilot = np.tile(ltsSym,numSamps/len(ltsSym)).astype(np.complex64)*.5
    else:
        Ts = 1/rate
        s_freq = 1e5
        s_time_vals = np.array(np.arange(0,numSamps)).transpose()*Ts
        pilot = np.exp(s_time_vals*1j*2*np.pi*s_freq).astype(np.complex64)*.25
    pad1 = np.array([0]*prefix_pad, np.complex64)
    pad2 = np.array([0]*postfix_pad, np.complex64)
    wbz = np.array([0]*(symSamp), np.complex64)
    pilot1 = np.concatenate([pad1,pilot,pad2]) 
    pilot2 = pilot1 if both_channels else wbz
  
    # configure tdd mode 
    bconf = {"tdd_enabled": True, "trigger_out": True, "symbol_size" : symSamp, "frames": ["PGRG"]}
    bsdr.writeSetting("TDD_CONFIG", json.dumps(bconf))
    mconf = {"tdd_enabled": True, "trigger_out": True, "symbol_size" : symSamp, "frames": ["RGPG"]}
    msdr.writeSetting("TDD_CONFIG", json.dumps(mconf))
    msdr.writeSetting("TDD_MODE", "true")
    bsdr.writeSetting("TDD_MODE", "true")

    for sdr in [bsdr, msdr]:
        sdr.writeSetting("TX_SW_DELAY", str(30))

    replay_addr = 0
    pilot1_ui32 = cfloat2uint32(pilot1) 
    pilot2_ui32 = cfloat2uint32(pilot2) 
    for sdr in [bsdr, msdr]:
        sdr.writeRegisters("TX_RAM_A", replay_addr, cfloat2uint32(pilot1).tolist())
        sdr.writeRegisters("TX_RAM_B", replay_addr, cfloat2uint32(pilot2).tolist())

    flags = 0
    r1 = bsdr.activateStream(rxStreamB, flags, 0)
    r2 = msdr.activateStream(rxStreamM, flags, 0)
    if r1<0:
        print("Problem activating stream #1")
    if r2<0:
        print("Problem activating stream #2")
    bsdr.writeSetting("TRIGGER_GEN", "")
    r1 = msdr.readStream(rxStreamM, [waveRxA1, waveRxB1], symSamp)
    print("reading stream #1 ({})".format(r1))
    r2 = bsdr.readStream(rxStreamB, [waveRxA2, waveRxB2], symSamp)
    print("reading stream #2 ({})".format(r2))
 
    print("printing number of frames")
    print("NB {}".format(SoapySDR.timeNsToTicks(bsdr.getHardwareTime(""), rate)))
    print("UE {}".format(SoapySDR.timeNsToTicks(msdr.getHardwareTime(""), rate)))
    # ADC_rst, stops the tdd time counters, makes sure next time runs in a clean slate


    bsdr.writeRegister("IRIS30", RF_RST_REG, (1<<29)| 0x1)
    bsdr.writeRegister("IRIS30", RF_RST_REG, (1<<29))
    bsdr.writeRegister("IRIS30", RF_RST_REG, 0)
    msdr.writeRegister("IRIS30", RF_RST_REG, (1<<29)| 0x1)
    msdr.writeRegister("IRIS30", RF_RST_REG, (1<<29))
    msdr.writeRegister("IRIS30", RF_RST_REG, 0)
    for i in range(4):
        msdr.writeRegister("RFCORE", SCH_ADDR_REG, i) # subframe 0
        msdr.writeRegister("RFCORE", SCH_MODE_REG, 0) # 01 replay
        bsdr.writeRegister("RFCORE", SCH_ADDR_REG, i) # subframe 0
        bsdr.writeRegister("RFCORE", SCH_MODE_REG, 0) # 01 replay
    bsdr.writeRegister("RFCORE", TDD_CONF_REG, 0)
    msdr.writeRegister("RFCORE", TDD_CONF_REG, 0)

    msdr.closeStream(rxStreamM)
    bsdr.closeStream(rxStreamB)
    msdr = None
    bsdr = None

    fig = plt.figure(figsize=(20, 8), dpi=120)
    fig.subplots_adjust(hspace=.5, top=.85)
    ax1 = fig.add_subplot(2, 1, 1)
    ax1.grid(True)
    ax1.set_title('Serials: (%s, %s)' % (serial1,serial2))
    ax1.set_ylabel('Signal (units)')
    ax1.set_xlabel('Sample index')
    ax1.plot(range(len(waveRxA1)), np.real(uint32tocfloat(waveRxA1)), label='ChA I Node 1')
    ax1.plot(range(len(waveRxB1)), np.real(uint32tocfloat(waveRxB1)), label='ChB I Node 1')
    ax1.set_ylim(-1, 1)
    ax1.set_xlim(0, symSamp)
    ax1.legend(fontsize=10)
    ax2 = fig.add_subplot(2, 1, 2)
    ax2.grid(True)
    ax2.set_ylabel('Signal (units)')
    ax2.set_xlabel('Sample index')
    ax2.plot(range(len(waveRxA2)), np.real(uint32tocfloat(waveRxA2)), label='ChA I Node 2')
    ax2.plot(range(len(waveRxB2)), np.real(uint32tocfloat(waveRxB2)), label='ChB I Node 2')
    ax2.set_ylim(-1, 1)
    ax2.set_xlim(0, symSamp)
    ax2.legend(fontsize=10)
    plt.show()


def main():
    parser = OptionParser()
    parser.add_option("--serial1", type="string", dest="serial1", help="serial number of the device 1", default="")
    parser.add_option("--serial2", type="string", dest="serial2", help="serial number of the device 2", default="")
    parser.add_option("--rate", type="float", dest="rate", help="Tx sample rate", default=5e6)
    parser.add_option("--txgain", type="float", dest="txgain", help="Optional Tx gain (dB)", default=20.0)
    parser.add_option("--rxgain", type="float", dest="rxgain", help="Optional Tx gain (dB)", default=20.0)
    parser.add_option("--freq", type="float", dest="freq", help="Optional Tx freq (Hz)", default=3.6e9)
    parser.add_option("--numSamps", type="int", dest="numSamps", help="Num samples to receive", default=512)
    parser.add_option("--prefix-pad", type="int", dest="prefix_length", help="prefix padding length for beacon and pilot", default=82)
    parser.add_option("--postfix-pad", type="int", dest="postfix_length", help="postfix padding length for beacon and pilot", default=68)
    parser.add_option("--both-channels", action="store_true", dest="both_channels", help="transmit from both channels",default=False)
    parser.add_option("--wideband", action="store_true", dest="wideband", help="send wideband pilot",default=False)
    (options, args) = parser.parse_args()
    siso_tdd_burst(
        serial1=options.serial1,
        serial2=options.serial2,
        rate=options.rate,
        freq=options.freq,
        txgain=options.txgain,
        rxgain=options.rxgain,
        numSamps=options.numSamps,
        prefix_pad=options.prefix_length,
        postfix_pad=options.postfix_length,
        both_channels=options.both_channels,
        wideband=options.wideband,
    )

if __name__ == '__main__':
    main()
 
