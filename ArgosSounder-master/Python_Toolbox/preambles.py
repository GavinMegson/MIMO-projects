import fractions
import numpy as np
from lteZadoffChuSeq import *
from  preamble_generator import *

def getPreambles(preamble_type='gold_ifft', seq_length=128, cp=0, upsample=1):
# Returns matrix where each row is an individual
# preambles. Row are ordered such that top row has
# the highest autocorrelation highest_peak_to_second_ratio

    #preambles = []
    #preambles = np.array([]).reshape(0,2*seq_length)
    preambles = np.empty((seq_length,2*seq_length*upsample+cp),dtype='complex64')
    highest_peak_to_second_ratio = []
    auto_corr = []

    preamble_type = preamble_type.lower()

## Generate ZadoffSequences
    if preamble_type ==  'zadoff_chu':         #ERROR: NOT TESTED# #TODO: add upsampling
        for i in range(1,seq_length): 
            if fractions.gcd(seq_length, i) == 1:
                p = lteZadoffChuSeq(i, seq_length)
                p_short = p
                for n in range(len(p)):
                    auto_corr[n] =  p_short*np.roll(p_short.conj().transpose(), n) 
                idx = abs(auto_corr).argsort()
                auto_corr = auto_corr[idx]
                auto_corr = auto_corr[::-1]
                #highest_peak_to_second_ratio[end+1] = abs(auto_corr[0]) - abs(auto_corr[1])
                highest_peak_to_second_ratio.append(abs(auto_corr[0]) - abs(auto_corr[1]))
                # Add CP and repetition
                p = np.concatonate([p[len(p)-cp:len(p)], p, p],axis = 1)  ## Not sure if the axis is right
                preambles = np.concatenate((preambles,p))
								

    ## Generate IFFT GoldCode sequences
    if preamble_type == 'gold_ifft':
        auto_corr = []
        integ = np.round(np.log2(seq_length))
        if (2**integ - seq_length == 0):
            for i in range(seq_length):
                thisP = preamble_generator(int(round(np.log2(seq_length))), i, cp, 0, upsample=upsample)
                thisP = (1-2**-15)*thisP/np.amax(abs(thisP))
                p_short = thisP[cp+seq_length:]
                auto_corr = np.correlate(p_short,p_short, mode = "full")
                idx = abs(auto_corr).argsort()
                auto_corr = auto_corr[idx]
                auto_corr = auto_corr[::-1]
                abs_p = np.square(abs(p_short))
                PAPR = (max(abs_p)/(sum(abs_p)/len(abs_p)))
                #print (float((sum(abs_p)/len(abs_p))))  #error seems to be that the 108th term seems to be truncated   32 bit vs 64 bit precision 
                highest_peak_to_second_ratio.append(PAPR)
                preambles[i,:] = thisP
        else:
            print('not here')

    ## Generate BPSK GoldCode sequences
    if preamble_type == 'gold_bpsk':                    #ERROR: NOT TESTED#
        auto_corr = []
        integ = np.round(np.log2(seq_length))
        if (2**integ - seq_length == 0):
            for i in range(seq_length):
                p = preamble_generator(int(np.log2(seq_length)), i, cp, 1, upsample=upsample)
                p_short = p[cp+seq_length:len(p)]
                auto_corr = []
                p_short = p_short.reshape(len(p_short),1)
                for n in  range(len(p_short)):
                    auto_corr.append((np.dot(p_short,np.roll(p_short.conj().transpose(), n))))  #<---- error is here!
                auto_corr = auto_corr[::-1]
                highest_peak_to_second_ratio.append(abs(auto_corr[0])-abs(auto_corr[1]))    
                preambles = np.concatenate((preambles,p[np.newaxis])) #<----

    ## Order them according to highest autocorrelation Peak
    highest_peak_to_second_ratio_idx = np.argsort(highest_peak_to_second_ratio)     
    #print (float(highest_peak_to_second_ratio[108]))
    #print (float(highest_peak_to_second_ratio[84]))
    #print (highest_peak_to_second_ratio_idx)

    '''
    if seq_length == 128 and preamble_type == 'gold_ifft':    #temporary fix needed if indexing is wrong due to precision error
        y = highest_peak_to_second_ratio_idx[-2]
        highest_peak_to_second_ratio_idx[-2] = highest_peak_to_second_ratio_idx[-3]
        highest_peak_to_second_ratio_idx[-3] = y
    '''
    preambles = (1-2**-15)*preambles[highest_peak_to_second_ratio_idx]

    return preambles, highest_peak_to_second_ratio_idx
