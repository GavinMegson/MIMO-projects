#Interesting -- it looks like there was a bug with the circshift -- since
#seq1 didn't have the right dimensionality, it wasn't doing a shift...
#also, the scale wasn't working since it didn't have the abs (these bugs
#are in Eugenio's original code...)
#also, it is best to move the 0 to the 0th subcarrier...
#example usage preamble = preamble_generator(7, 1, 0);

#import numpy  #not sure if this is needed
import os
import sys
import numpy as np
#root = os.path.abspath(os.sep)
#sys.path.append(root + 'MUBF\Code\Argos\common\matlab')
from read_precomp_code import *

#check bpsk. I think if the nargins is less than 5, bpsk should also default to 0

def preamble_generator(N, index=0, CP=0, bpsk=0, shift=0, upsample=1):
	
	numsyms = 2**N
	seq1 = read_precomp_code(N, index)
	
	
	quadseq = seq1 + seq1*1j #you can shift the imaginary component...
	quadseq = np.concatenate([quadseq[0:int(np.ceil(len(quadseq))/2)], [0], quadseq[int(np.ceil(len(quadseq))/2):len(quadseq)]])


	#Enable second line for BPSK
	if bpsk:
		sequence = np.concatenate((quadseq, quadseq),axis = 1).transpose()
		#todo: enable upsampling here
	else:
		symseq = np.zeros(2*numsyms, dtype=complex)
		symseq[0:2*(numsyms)-1:2] = quadseq
		up_zeros = np.zeros(len(symseq)//2*(upsample-1))
		symseq_up = np.concatenate((up_zeros,symseq,up_zeros))
		sequence = np.fft.ifft((np.fft.ifftshift(symseq_up)))
	
	if CP:
		#Notice that we are adding a cyclic prefix of 1/4 of the signature txlen
		if CP == 1:
			sequence = np.concatenate([sequence[3/4*(numsyms)*2:(numsyms)*2], sequence]) #legacy use -- this is way too long of a preamble for N > 5
		else:
			sequence = np.concatenate([sequence[len(sequence)-CP : len(sequence)], sequence])
	scale = max(max(abs(sequence.real)) , max(abs(sequence.imag)))
	preamble = sequence * (1-2**-15) / scale
	
	return preamble
