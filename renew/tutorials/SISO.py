# near copy of siso tx/rx tutorial, with some additional comments

import sys
sys.path.append('../IrisUtils/')

import SoapySDR
from SoapySDR import * # SOAPY_SDR_ constants
from optparse import OptionParser
import numpy as np
import time
import os
import math
import json
import matplotlib.pyplot as plt
#from cfloat2uint32 import *
#from uint32tocfloat import *
from type_conv import cfloat2uint32
from type_conv import uint32tocfloat

# Registers:
RF_RST_REG = 48
TDD_CONF_REG = 120
SCH_ADDR_REG = 136
SCH_MODE_REG = 140
TX_GAIN_CTRL = 88


# Core functions
def siso_tdd_burst(serial1, serial2, rate, freq, txgain, rxgain, numSamps, prefix_pad, postfix_pad):
	# "base station" SDR object
	bsdr = SoapySDR.Device(dict(driver='iris', serial=serial1))
	# "mobile" SDR object
	msdr = SoapySDR.Device(dict(driver='iris', serial=serial2))

	# some default sample rates
	for i, sdr in enumerate([bsdr, msdr]):
		info = sdr.getHardwareInfo()
		print("%s settings on device %d" % (info["frontend"], i))
		for ch in [0]:
			sdr.setSampleRate(SOAPY_SDR_TX, ch, rate)
			sdr.setSampleRate(SOAPY_SDR_RX, ch, rate)

			sdr.setFrequency(SOAPY_SDR_TX, ch, 'RF', freq - 0.75*rate)
			sdr.setFrequency(SOAPY_SDR_RX, ch, 'RF', freq - 0.75*rate)
			sdr.setFrequency(SOAPY_SDR_TX, ch, 'BB', 0.75*rate)
			sdr.setFrequency(SOAPY_SDR_RX, ch, 'BB', 0.75*rate)

			if "CBRS" in info["frontend"]:
				sdr.setGain(SOAPY_SDR_TX, ch, 'ATTN', 0) # [-18,0] by 3
				sdr.setGain(SOAPY_SDR_TX, ch, 'PA1', 15) # [0|15]
				sdr.setGain(SOAPY_SDR_TX, ch, 'PA2', 0) # [0|15]
				sdr.setGain(SOAPY_SDR_TX, ch, 'PA3', 30) # [0|30]
			sdr.setGain(SOAPY_SDR_TX, ch, 'IAMP', 12) # [0,12]
			sdr.setGain(SOAPY_SDR_TX, ch, 'PAD', txgain) # [-52,0]

			if "CBRS" in info["frontend"]:
				sdr.setGain(SOAPY_SDR_RX, ch, 'ATTN', 0) # [-18,0]
				sdr.setGain(SOAPY_SDR_RX, ch, 'LNA1', 30) # [0,33]
				sdr.setGain(SOAPY_SDR_RX, ch, 'LNA2', 17) # [0,17]
			sdr.setGain(SOAPY_SDR_RX, ch, 'LNA', rxgain) # [0,30]
			sdr.setGain(SOAPY_SDR_RX, ch, 'TIA', 0) # [0,12]
			sdr.setGain(SOAPY_SDR_RX, ch, 'PGA', 0) # [-12,19]

			sdr.setAntenna(SOAPY_SDR_RX, ch, "TRX")
			sdr.setDCOffsetMode(SOAPY_SDR_RX, ch, True)

		# following 3 commands will go away in next release due to irrelevance:
		sdr.writeSetting("SPI_TDD_MODE","SISO")
		sdr.writeSetting(SOAPY_SDR_RX,1,'ENABLE_CHANNEL','false')
		sdr.writeSetting(SOAPY_SDR_TX,1,'ENABLE_CHANNEL','false')

	#TX_GAIN_CTRL and SYNC_DELAYS
	msdr.writeRegister("IRIS30", TX_GAIN_CTRL, 0)
	bsdr.writeSetting("SYNC_DELAYS", "")


	# packet size
	symSamp = numSamps + prefix_pad + postfix_pad
	print("numSamps = %d" % numSamps)
	print("symSamps = %d" % symSamp)

	# generate sinusoid pilot
	Ts = 1/rate
	s_freq = 1e5
	s_time_vals = np.array(np.arange(numSamps)).transpose() *Ts
	pilot = np.exp(s_time_vals * 1j * 2 * np.pi * s_freq).astype(np.complex64) * 1
	pad1 = np.array([0]*prefix_pad, np.complex64)
	pad2 = np.array([0]*postfix_pad, np.complex64)
	wbz = np.array([0]*symSamp, np.complex64)
	pilot1 = np.concatenate([pad1,pilot,pad2])
	pilot2 = wbz # all 0s, basically an empty pilot

	# initialize RX arrays
	waveRxA1 = np.array([0]*symSamp, np.uint32)
	waveRxB1 = np.array([0]*symSamp, np.uint32)
	waveRxA2 = np.array([0]*symSamp, np.uint32)
	waveRxB2 = np.array([0]*symSamp, np.uint32)

	# create RX streams (required to be CS16)
	rxStreamB = bsdr.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, [0,1])
	rxStreamM = msdr.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, [0,1])

	# using the TDD framer
	# set schedule
	bsched = "PGRG"
	msched = "RGPG"
	print("Node 1 schedule %s " % bsched)
	print("Node 2 schedule %s " % msched)

	# send 1 frame (set max_frame to 1)
	bconf = {"tdd_enabled":True, "frame_mode": "free_running", "symbol_size": symSamp, "frames": [bsched], \
		"max_frame": 1}
	mconf = {"tdd_enabled":True, "frame_mode": "free_running", "dual_pilot": False, "symbol_size": symSamp, \
		"frames": [msched], "max_frame": 1}
	bsdr.writeSetting("TDD_CONFIG", json.dumps(bconf))
	msdr.writeSetting("TDD_CONFIG", json.dumps(mconf))

	# SW Delays
	for sdr in [bsdr, msdr]:
		sdr.writeSetting("TX_SW_DELAY", str(30))

	msdr.writeSetting("TDD_MODE", "true")
	bsdr.writeSetting("TDD_MODE", "true")

	# load pilots into RAM
	replay_addr = 0
	for sdr in [bsdr,msdr]:
		sdr.writeRegisters("TX_RAM_A", replay_addr, cfloat2uint32(pilot1, order='IQ').tolist())
		sdr.writeRegisters("TX_RAM_B", replay_addr, cfloat2uint32(pilot2, order='IQ').tolist())

	#activate streams
	flags = 0
	r1 = bsdr.activateStream(rxStreamB, flags, 0)
	r2 = msdr.activateStream(rxStreamM, flags, 0)
	if r1<0:
		print("Problem activating stream 1")
	if r2<0:
		print("Problem activating stream 2")

	bsdr.writeSetting ("TRIGGER_GEN","")

	# read RX streams
	r1 = msdr.readStream(rxStreamM, [waveRxA1, waveRxB1], symSamp)
	r2 = bsdr.readStream(rxStreamB, [waveRxA2, waveRxB2], symSamp)
	print("reading stream 1 ({})".format(r1))
	print("reading stream 2 ({})".format(r2))

	print("printing number of frames")
	print("UE {}".format(SoapySDR.timeNsToTicks(msdr.getHardwareTime(""), rate)))
	print("NB {}".format(SoapySDR.timeNsToTicks(bsdr.getHardwareTime(""), rate)))


	# cleanup
	# ADC_rst, stops tdd time counters, makes sure next time runs in a clean slate
	bsdr.writeRegister("IRIS30", RF_RST_REG, (1<<29) | 0x1)
	bsdr.writeRegister("IRIS30", RF_RST_REG, (1<<29))
	bsdr.writeRegister("IRIS30", RF_RST_REG, 0)
	msdr.writeRegister("IRIS30", RF_RST_REG, (1<<29) | 0x1)
	msdr.writeRegister("IRIS30", RF_RST_REG, (1<<29))
	msdr.writeRegister("IRIS30", RF_RST_REG, 0)

	for i in range(4):
		msdr.writeRegister("RFCORE", SCH_ADDR_REG, i) # subframe 0
		msdr.writeRegister("RFCORE", SCH_MODE_REG, 0) # 01 replay
		bsdr.writeRegister("RFCORE", SCH_ADDR_REG, i) # subframe 0
		bsdr.writeRegister("RFCORE", SCH_MODE_REG, 0) # 01 replay
	bsdr.writeRegister("RFCORE", TDD_CONF_REG, 0)
	msdr.writeRegister("RFCORE", TDD_CONF_REG, 0)

	msdr.deactivateStream(rxStreamM)
	bsdr.deactivateStream(rxStreamB)
	msdr.closeStream(rxStreamM)
	bsdr.closeStream(rxStreamB)
	msdr = None
	bsdr = None


	#plotting
	fig = plt.figure(figsize=(20,8), dpi=120)
	fig.subplots_adjust(hspace=.5, top=.85)

	ax1 = fig.add_subplot(2,1,1)
	ax1.grid(True)
	ax1.set_title('Serials: (%s, %s)' % (serial1, serial2))
	ax1.set_ylabel('Signal (units)')
	ax1.set_xlabel('Sample index')
	ax1.plot(range(len(waveRxA1)), np.real(uint32tocfloat(waveRxA1)), label='ChA I Node 1')
	ax1.plot(range(len(waveRxB1)), np.real(uint32tocfloat(waveRxB1)), label='ChB I Node 1')
	ax1.set_ylim(-1,1)
	ax1.set_xlim(0, symSamp)
	ax1.legend(fontsize=10)

	ax2 = fig.add_subplot(2,1,2)
	ax2.grid(True)
	ax2.set_ylabel('Signal (units)')
	ax2.set_xlabel('Sample index')
	ax2.plot(range(len(waveRxA2)), np.real(uint32tocfloat(waveRxA2)), label='ChA I Node 2')
	ax2.plot(range(len(waveRxB2)), np.real(uint32tocfloat(waveRxB2)), label='ChB I Node 2')
	ax2.set_ylim(-1,1)
	ax2.set_xlim(0, symSamp)
	ax2.legend(fontsize=10)

	plt.show()


def main():
	parser = OptionParser()
	parser.add_option("--serial1", type="string", dest="serial1", help="serial number of the device 1", default="")
	parser.add_option("--serial2", type="string", dest="serial2", help="serial number of the device 2", default="")
	parser.add_option("--rate", type="float", dest="rate", help="Tx sample rate", default=5e6)
	parser.add_option("--txgain", type="float", dest="txgain", help="Optional Tx gain (dB)", default=25.0)
	parser.add_option("--rxgain", type="float", dest="rxgain", help="Optional Tx gain (dB)", default=20.0)
	parser.add_option("--freq", type="float", dest="freq", help="Optional Tx freq (Hz)", default=2.6e9)
	parser.add_option("--numSamps", type="int", dest="numSamps", help="Num samples to receive", default=512)
	parser.add_option("--prefix-pad", type="int", dest="prefix_length", help="prefix padding length for beacon and pilot", default=82)
	parser.add_option("--postfix-pad", type="int", dest="postfix_length", help="postfix padding length for beacon and pilot", default=68)
	(options, args) = parser.parse_args()

	# call function
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
	)


if __name__ == '__main__':
	main()
