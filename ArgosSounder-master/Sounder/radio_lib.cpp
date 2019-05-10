/*
 Copyright (c) 2018-2019, Rice University 
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
 Author(s): Rahman Doost-Mohamamdy: doost@rice.edu
 
---------------------------------------------------------------------
 Initializes and Configures Radios in the massive-MIMO base station 
---------------------------------------------------------------------
*/

#include "radio_lib.hpp"
RadioConfig::RadioConfig(Config *cfg):
    _cfg(cfg)
{
    SoapySDR::Kwargs args;
    SoapySDR::Kwargs sargs;
    //load channels
    std::vector<size_t> channels;
    if (_cfg->nChannels == 1) channels = {0};
    else if (_cfg->nChannels == 2) channels = {0, 1};
    else
    {
        std::cout << "Error! Supported number of channels 1 or 2, setting to 2!" << std::endl;
        _cfg->nChannels = 2;
        channels = {0, 1};
    }

    this->_radioNum = _cfg->radio_ids.size();
    this->_antennaNum = _radioNum * _cfg->nChannels;
     std::cout << "radio num is " << this->_radioNum << std::endl;
    if (_cfg->hub_ids.size() != 0)
    { 
        args["serial"] = _cfg->hub_ids.at(0);
        hubs.push_back(SoapySDR::Device::make(args)); 
    }

    for (int i = 0; i < this->_radioNum; i++)
    {
        args["serial"] = _cfg->radio_ids.at(i);
        baStn.push_back(SoapySDR::Device::make(args));
    }

    for (int i = 0; i < this->_radioNum; i++)
    { 
        //use the TRX antenna port for both tx and rx
        for (auto ch : channels) baStn[i]->setAntenna(SOAPY_SDR_RX, ch, "TRX");

        std::cout << "setting samples rates to " << cfg->rate/1e6 << " Msps..." << std::endl;
        SoapySDR::Kwargs info = baStn[i]->getHardwareInfo();
        for (auto ch : channels)
        {
            //baStn[i]->setBandwidth(SOAPY_SDR_RX, ch, 30e6);
            //baStn[i]->setBandwidth(SOAPY_SDR_TX, ch, 30e6);

            baStn[i]->setSampleRate(SOAPY_SDR_RX, ch, cfg->rate);
            baStn[i]->setSampleRate(SOAPY_SDR_TX, ch, cfg->rate);

            //baStn[i]->setFrequency(SOAPY_SDR_RX, ch, cfg->freq);  
            //baStn[i]->setFrequency(SOAPY_SDR_TX, ch, cfg->freq); 
            baStn[i]->setFrequency(SOAPY_SDR_RX, ch, "RF", cfg->freq-.75*cfg->rate);
            baStn[i]->setFrequency(SOAPY_SDR_RX, ch, "BB", .75*cfg->rate);
            baStn[i]->setFrequency(SOAPY_SDR_TX, ch, "RF", cfg->freq-.75*cfg->rate);
            baStn[i]->setFrequency(SOAPY_SDR_TX, ch, "BB", .75*cfg->rate);
 
            if (info["frontend"].find("CBRS") != std::string::npos)
            {
                baStn[i]->setGain(SOAPY_SDR_RX, ch, "ATTN", 0); //[-18,0]  
                baStn[i]->setGain(SOAPY_SDR_RX, ch, "LNA1", 30); //[0,33]
                baStn[i]->setGain(SOAPY_SDR_RX, ch, "LNA2", 17); //[0,17]
            }
            if (info["frontend"].find("UHF") != std::string::npos)
            {
                baStn[i]->setGain(SOAPY_SDR_RX, ch, "ATTN1", -6); //[-18,0]  
                baStn[i]->setGain(SOAPY_SDR_RX, ch, "ATTN2", -12); //[-18,0]  
                //baStn[i]->setGain(SOAPY_SDR_RX, ch, "LNA1", 30); //[0,33]
                //baStn[i]->setGain(SOAPY_SDR_RX, ch, "LNA2", 17); //[0,17]
            }
            baStn[i]->setGain(SOAPY_SDR_RX, ch, "LNA", ch ? cfg->rxgainB : cfg->rxgainA);  //[0,30]
            baStn[i]->setGain(SOAPY_SDR_RX, ch, "TIA", 0);  //[0,12]
            baStn[i]->setGain(SOAPY_SDR_RX, ch, "PGA", 0);  //[-12,19]

            if (info["frontend"].find("CBRS") != std::string::npos)
            {
                baStn[i]->setGain(SOAPY_SDR_TX, ch, "ATTN", 0);  //[-18,0] by 3
                baStn[i]->setGain(SOAPY_SDR_TX, ch, "PA1", 15);  //[0|15]
                baStn[i]->setGain(SOAPY_SDR_TX, ch, "PA2", 0);   //[0|15]
                baStn[i]->setGain(SOAPY_SDR_TX, ch, "PA3", 30);  //[0|30]
            }
            if (info["frontend"].find("UHF") != std::string::npos)
            {
                baStn[i]->setGain(SOAPY_SDR_TX, ch, "ATTN", 0);  //[-18,0] by 3
            }

            baStn[i]->setGain(SOAPY_SDR_TX, ch, "IAMP", 0);     //[0,12]
            baStn[i]->setGain(SOAPY_SDR_TX, ch, "PAD", ch ? cfg->txgainB : cfg->txgainA);  //[0,30]

        }

        for (auto ch : channels)
        {
            //baStn[i]->writeSetting(SOAPY_SDR_RX, ch, "CALIBRATE", "SKLK");
            //baStn[i]->writeSetting(SOAPY_SDR_TX, ch, "CALIBRATE", "");
            baStn[i]->setDCOffsetMode(SOAPY_SDR_RX, ch, true);
        }

        // we disable channel 1 because of the internal LDO issue.
        // This will be fixed in the next revision (E) of Iris.
        if (_cfg->freq > 3e9 and _cfg->radio_ids[i].find("RF3E") == std::string::npos)
        {
            if (_cfg->nChannels == 1)
            {
                std::vector<unsigned> txActive, rxActive;
                unsigned ch = baStn[i]->readRegister("LMS7IC", 0x0020);
                baStn[i]->writeRegister("LMS7IC", 0x0020, (ch & 0xFFFC) | 1);
                unsigned regRfeA = baStn[i]->readRegister("LMS7IC", 0x010C);
                unsigned regRfeALo = baStn[i]->readRegister("LMS7IC", 0x010D);
                unsigned regRbbA = baStn[i]->readRegister("LMS7IC", 0x0115);
                unsigned regTrfA = baStn[i]->readRegister("LMS7IC", 0x0100);
                unsigned regTbbA = baStn[i]->readRegister("LMS7IC", 0x0105);

                // disable TX
                txActive = {
                    //0xa10C0000 | 0xfe, //RFE in power down
                    //0xa10D0000 | 0x0, //RFE SISO and disables
                    0xa1150000 | 0xe, //RBB in power down
                    //0xa1000000 | regTrfA //TRF stays the same
                    0xa1050000 | regTbbA //TBB stays the same
                };
                baStn[i]->writeRegisters("LMS7_PROG_SPI", 16, txActive); //trig1 offset
                // disable RX
                rxActive = {
                    //0xa10C0000 | regRfeA, //RFE stays the same
                    //0xa10D0000 | regRfeALo, //RFE stays the same
                    0xa1150000 | regRbbA, //RBB stays the same
                    //0xa1000000 | 0xe //TRF in power down + SISO
                    0xa1050000 | 0x1e //TBB in power down
                };
                baStn[i]->writeRegisters("LMS7_PROG_SPI", 32, rxActive); //trig2 offset

                //baStn[i]->writeSetting("SPI_TDD_MODE", "SISO"); // a FPGA hack that bypasses the LDO issue
                baStn[i]->writeSetting(SOAPY_SDR_RX, 1, "ENABLE_CHANNEL", "false");
                baStn[i]->writeSetting(SOAPY_SDR_TX, 1, "ENABLE_CHANNEL", "false");
            } else
            {
                std::vector<unsigned> txActive, rxActive;
                unsigned ch = baStn[i]->readRegister("LMS7IC", 0x0020);
                baStn[i]->writeRegister("LMS7IC", 0x0020, (ch & 0xFFFC) | 1);
                unsigned regRfeA = baStn[i]->readRegister("LMS7IC", 0x010C);
                unsigned regRfeALo = baStn[i]->readRegister("LMS7IC", 0x010D);
                unsigned regRbbA = baStn[i]->readRegister("LMS7IC", 0x0115);
                unsigned regTrfA = baStn[i]->readRegister("LMS7IC", 0x0100);
                unsigned regTbbA = baStn[i]->readRegister("LMS7IC", 0x0105);

                ch = baStn[i]->readRegister("LMS7IC", 0x0020);
                baStn[i]->writeRegister("LMS7IC", 0x0020, (ch & 0xFFFC) | 2);
                unsigned regRfeB = baStn[i]->readRegister("LMS7IC", 0x010C);
                unsigned regRbbB = baStn[i]->readRegister("LMS7IC", 0x0115);
                unsigned regTrfB = baStn[i]->readRegister("LMS7IC", 0x0100);
                unsigned regTbbB = baStn[i]->readRegister("LMS7IC", 0x0105);

                txActive = {
                    //0xe10C0000 | 0xfe, //RFE in power down
                    //0xe10D0000 | 0x0, //RFE SISO and disables
                    0xe1150000 | 0xe, //RBB in power down
                    //0xe1000000 | regTrfA, //TRF stays the same
                    0xe1050000 | regTbbA}; //TBB stays the same

                rxActive = {
                    //0xe10C0000 | regRfeA, //RFE stays the same
                    //0xe10D0000 | regRfeALo, //RFE stays the same
                    0xe1150000 | regRbbA, //RBB stays the same
                    //0xe1000000 | 0xe, //TRF in power down + SISO
                    0xe1050000 | 0x1e}; //TBB in power down

                baStn[i]->writeRegisters("LMS7_PROG_SPI", 16, txActive); //trig1 offset
                baStn[i]->writeRegisters("LMS7_PROG_SPI", 32, rxActive); //trig2 offset
                //baStn[i]->writeSetting("SPI_TDD_MODE", "MIMO");
            }
        }
        //The following must be done by the driver at initialization
        baStn[i]->writeRegister("RFCORE", 120, 0); // reset the tdd mode in the FPGA
        // resets the DATA_clk domain logic. 
        baStn[i]->writeRegister("IRIS30", 48, (1<<29) | 0x1);
        baStn[i]->writeRegister("IRIS30", 48, (1<<29));
        baStn[i]->writeRegister("IRIS30", 48, 0);

        this->rxStreams.push_back(baStn[i]->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, channels, sargs));
        this->txStreams.push_back(baStn[i]->setupStream(SOAPY_SDR_TX, SOAPY_SDR_CS16, channels, sargs));
    }
    std::cout << "radio init done!" << std::endl;
}

void RadioConfig::radioStart()
{
    long long frameTime(0);
    // send through the first radio for now
    //int beacon_ant = 1;
    int flags = 0;
    std::vector<unsigned> zeros(_cfg->sampsPerSymbol,0);
    std::vector<unsigned> beacon(_cfg->beacon.size()); 
    std::vector<unsigned> beacon_weights(_cfg->nAntennas); 
    for (int i = 0; i < _cfg->beacon.size(); i++)
        beacon[i] = (uint32_t)(_cfg->beacon[i].imag()) | ((uint32_t)(_cfg->beacon[i].real()) << 16);

    std::vector<std::string> _tddSched;
    _tddSched.resize(_cfg->framePeriod);
    for (int f = 0; f < _cfg->framePeriod; f++)
    {
        _tddSched[f] = _cfg->frames[f];
        for (int s =0; s < _cfg->frames[f].size(); s++)
        {
            char c = _cfg->frames[f].at(s);
            if (c == 'B')
                _tddSched[f].replace(s, 1, "P");
            else if (c == 'P')
                _tddSched[f].replace(s, 1, "R");
            else if (c == 'U')
                _tddSched[f].replace(s, 1, "R");
            else if (c == 'D')
                _tddSched[f].replace(s, 1, "T");
        }
    } 
#ifdef JSON
    json conf;
    conf["tdd_enabled"] = true;
    conf["trigger_out"] = false;
    conf["frames"] = _tddSched;
    conf["symbol_size"] = _cfg->sampsPerSymbol;
    std::string confString = conf.dump(); 
#else
    std::string confString ="{\"tdd_enabled\":true,\"trigger_out\":false,";
    confString +="\"symbol_size\":"+std::to_string(_cfg->sampsPerSymbol);
    confString +=",\"frames\":[";
    for (int f = 0; f < _cfg->framePeriod; f++)
        confString += (f == _cfg->framePeriod - 1) ? "\""+_tddSched[f]+"\"" : "\""+_tddSched[f]+"\",";
    confString +="]}";
    std::cout << confString << std::endl;
#endif
    for (int i = 0; i < this->_radioNum; i++)
    {
        baStn[i]->writeSetting("TX_SW_DELAY", "30"); // experimentally good value for dev front-end
        baStn[i]->writeSetting("TDD_MODE", "true");
        baStn[i]->writeSetting("TDD_CONFIG", confString);
        // write beacons to FPGA buffers
        if (_cfg->beacon_mode == "single" or _cfg->nAntennas == 1)
        {
            if (i*_cfg->nChannels == _cfg->beacon_ant)
                baStn[i]->writeRegisters("TX_RAM_A", 0, beacon);
            else if (_cfg->nChannels == 2 and i*2+1 == _cfg->beacon_ant)
                baStn[i]->writeRegisters("TX_RAM_B", 0, beacon);
            else 
            {
                baStn[i]->writeRegisters("TX_RAM_A", 0, zeros);
                baStn[i]->writeRegisters("TX_RAM_B", 0, zeros);
            }
        } 
        else // beamsweep
        {
            baStn[i]->writeRegisters("TX_RAM_A", 0, beacon);
            if (_cfg->nChannels == 2)
                baStn[i]->writeRegisters("TX_RAM_B", 0, beacon);
            int residue = int(pow(2,ceil(log2(_cfg->nAntennas))))-_cfg->nAntennas;
            printf("residue %d\n", residue);
            beacon_weights.assign(_cfg->beacon_weights.at(i*_cfg->nChannels).begin(), _cfg->beacon_weights.at(i*_cfg->nChannels).begin()+_cfg->nAntennas);
            baStn[i]->writeRegisters("TX_RAM_WGT_A", 0, beacon_weights);
            if (_cfg->nChannels == 2)
            {
                beacon_weights.assign(_cfg->beacon_weights[i*_cfg->nChannels+1].begin(), _cfg->beacon_weights[i*_cfg->nChannels+1].begin()+_cfg->nAntennas);
                baStn[i]->writeRegisters("TX_RAM_WGT_B", 0, beacon_weights);
            }
            baStn[i]->writeRegister("RFCORE", 156, _radioNum);
            baStn[i]->writeRegister("RFCORE", 160, 1); // enable beamsweeping
             
        }
        baStn[i]->activateStream(this->rxStreams[i], flags, 0);
    }
    if (hubs.size() == 0)
    {
        std::cout << "triggering first Iris ..." << std::endl;
        baStn[0]->writeSetting("SYNC_DELAYS", "");
        baStn[0]->writeSetting("TRIGGER_GEN", "");
    }
    else
    {
        std::cout << "triggering Hub ..." << std::endl;
        hubs[0]->writeSetting("SYNC_DELAYS", "");
        hubs[0]->writeSetting("TRIGGER_GEN", "");
    }
    std::cout << "radio start done!" << std::endl;
}

void RadioConfig::readSensors()
{
    for (int i = 0; i < this->_radioNum; i++)
    {
        std::cout << "TEMPs on Iris " << i << std::endl;
        std::cout << "ZYNQ_TEMP: " << baStn[i]->readSensor("ZYNQ_TEMP") << std::endl;
        std::cout << "LMS7_TEMP  : " << baStn[i]->readSensor("LMS7_TEMP") << std::endl;
        std::cout << "FE_TEMP  : " << baStn[i]->readSensor("FE_TEMP") << std::endl;
        std::cout << "TX0 TEMP  : " << baStn[i]->readSensor(SOAPY_SDR_TX, 0, "TEMP") << std::endl;
        std::cout << "TX1 TEMP  : " << baStn[i]->readSensor(SOAPY_SDR_TX, 1, "TEMP") << std::endl;
        std::cout << "RX0 TEMP  : " << baStn[i]->readSensor(SOAPY_SDR_RX, 0, "TEMP") << std::endl;
        std::cout << "RX1 TEMP  : " << baStn[i]->readSensor(SOAPY_SDR_RX, 1, "TEMP") << std::endl;
        std::cout << std::endl;
    }
}

void RadioConfig::radioStop()
{
    for (int i = 0; i < this->_radioNum; i++)
    {
        baStn[i]->writeSetting("TDD_MODE", "false");
        baStn[i]->writeRegister("IRIS30", 48, (1<<29)| 0x1);
        baStn[i]->writeRegister("IRIS30", 48, (1<<29));
        baStn[i]->writeRegister("IRIS30", 48, 0);
        // write schedule
	for (int j = 0; j < 16; j++) 
        {
            for(int k = 0; k < _cfg->symbolsPerFrame; k++) // symnum <= 256
            {
		baStn[i]->writeRegister("RFCORE", 136, j*256+k);
		baStn[i]->writeRegister("RFCORE", 140, 0);
            }
        }
        baStn[i]->closeStream(this->rxStreams[i]);
        baStn[i]->closeStream(this->txStreams[i]);
    }
}

void RadioConfig::radioTx(void ** buffs)
{
    int flags = 0;
    long long frameTime(0);
    for (int i = 0; i < this->_radioNum; i++)
    {
        baStn[i]->writeStream(this->txStreams[i], buffs, _cfg->sampsPerSymbol, flags, frameTime, 1000000);
    }
}

void RadioConfig::radioTx(int r /*radio id*/, void ** buffs)
{
    int flags = 0;
    long long frameTime(0);
    baStn[r]->writeStream(this->txStreams[r], buffs, _cfg->sampsPerSymbol, flags, frameTime, 1000000);
}

void RadioConfig::radioRx(void ** buffs)
{
    int flags = 0;
    long long frameTime(0);
    for (int i = 0; i < this->_radioNum; i++)
    {
        void **buff = buffs + (i * 2);
        baStn[i]->readStream(this->rxStreams[i], buff, _cfg->sampsPerSymbol, flags, frameTime, 1000000);
    }
}

void RadioConfig::radioRx(int r /*radio id*/, void ** buffs, long long & frameTime)
{
    int flags = 0;
    if (r < this->_radioNum)
    {
        long long frameTimeNs = 0;
        int ret = baStn[r]->readStream(this->rxStreams[r], buffs, _cfg->sampsPerSymbol, flags, frameTimeNs, 1000000);
        frameTime = frameTimeNs; //SoapySDR::timeNsToTicks(frameTimeNs, _rate);
        if (ret != _cfg->sampsPerSymbol)
            std::cout << "invalid return " << ret << " from radio " << r <<std::endl;
        return;
    }
    std::cout << "invalid radio id " << r << std::endl;
}

void RadioConfig::radioSched(std::vector<std::string> sched)
{
    // make sure we can pause the framer before we rewrite schedules to avoid
    // any race conditions, for now I set tdd mode to 0
#ifdef JSON
    json conf;
    conf["tdd_enabled"] = true;
    conf["trigger_out"] = false;
    conf["frames"] = sched;
    conf["symbol_size"] = _cfg->sampsPerSymbol;
    std::string confString = conf.dump(); 
#else
    std::string confString ="{\"tdd_enabled\":true,\"trigger_out\":false,";
    confString +="\"symbol_size\":"+std::to_string(_cfg->sampsPerSymbol);
    confString +=",\"frames\":[";
    for (int f = 0; f < sched.size(); f++)
        confString += (f == sched.size() - 1) ? "\""+sched[f]+"\"" : "\""+sched[f]+"\",";
    confString +="]}";
    std::cout << confString << std::endl;
#endif
    for (int i = 0; i < this->_radioNum; i++)
    {
        baStn[i]->writeSetting("TDD_CONFIG", confString);
    }
}
RadioConfig::~RadioConfig(){}

