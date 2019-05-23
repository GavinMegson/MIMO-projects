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
from cfloat2uint32 import *
from uint32tocfloat import *

# Registers:
RF_RST_REG = 48
TDD_CONF_REG = 120
SCH_ADDR_REG = 136
SCH_MODE_REG = 140
TX_GAIN_CTRL = 88


