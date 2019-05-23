from argparse import ArgumentParser
import SoapySDR
from SoapySDR import * #SOAPY_SDR_constants
import numpy as np
import time



class TRX:
	def __init__(self,txserial,rxserial,freq,rate,rxgain):
		serialtx = txserial
		serialrx = rxserial

		# instantiate device
		self.sdr = SoapySDR.Device(dict(driver='iris',serial=serialtx))

		# disable TX gain control
		TX_GAIN_CTRL = 88 # register 88
		tx_gain_ctrl_en = 0
		self.sdr.writeRegister("IRIS30", TX_GAIN_CTRL, tx_gain_ctrl_en)

		# RSSI
		self.rssi = self.rssi()

		# get hardware info
		self.info = self.sdr.getHardwareInfo()

		# set up channels
#		self.ch_setup(rate,rxgain)

	def rssi(self):
		"""return the received signal strength indicator"""

		FPGA_IRIS030_RD_MEASURED_RSSI = 284 # register 284

		return self.sdr.readRegister("IRIS30", FPGA_IRIS030_RD_MEASURED_RSSI)

'''
	def ch_setup(self,rate,rxgain):
		for ch in [0,1]:
			self.sdr.setSampleRate(SOAPY_SDR_TX, ch, rate)
'''

	def stream(self, numSamps):
		txStream = self.sdr.setupStream(SOAPY_SDR_TX, SOAPY_SDR_CF32, [0,1])
		self.sdr.activateStream(txStream)

		# get current timestamp
		sr = self.sdr.readStream(rxStream, [waveRxA, waveRxB], numSamps)
		# if valid timestamp
		if sr.ret > 0:
			txTime = sr.timeNs & 0xFFFFFFFF00000000
			txTime += (0x00000000 + (startSymbol << 16))

		flags = SOAPY_SDR_HAS_TIME | SOAPY_SDR_END_BURST
		for j in range(txSymNum):
			txTimeNs = txTime
			st = self.sdr.writeStream(txStream, [waveTxA, waveTxB], numSamps, flags, timeNs=txTimeNs)



if __name__ == '__main__':

	# parse arguments
	parser = ArgumentParser()
	parser.add_argument("--txserial", type=str, dest="txserial", help="TX SDR Serial Number, e.g. RF3C000020", default=None)
	parser.add_argument("--freq", type=float, dest="freq", help="Optional freq (Hz)", default = 2e9)
	parser.add_argument("--rate", type=float, dest="rate", help="optional sample rate", default = 5e6)
	parser.add_argument("--rxgain", type=float, dest="rxgain", help="Optional Rx gain (dB)", default=20)
	args = parser.parse_args()

	# instantiate class
	tx_sdr = TX(
		txserial=args.txserial,
		freq=args.freq,
		rate=args.rate
		rxgain = rxgain
	)
