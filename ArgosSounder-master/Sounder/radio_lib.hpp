/*
 Copyright (c) 2018-2019, Rice University 
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
 Author(s): Rahman Doost-Mohamamdy: doost@rice.edu
 
---------------------------------------------------------------------
 Initializes and Configures Radios in the massive-MIMO base station 
---------------------------------------------------------------------
*/

#include <iostream>
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Errors.hpp>
#include <SoapySDR/Time.hpp>
#include <cstdlib>
#include <cstddef>
#include <chrono>
#include <string>
#include <cstdint>
#include <complex>
#include <csignal>
#include "Config.hpp"
enum RadioType
{
 eNB=0,
 UE=1
};

class RadioConfig
{
public:
    RadioConfig(Config *cfg);
    void radioStart();
    void radioStop();
    void readSensors();
    void radioTx(void ** buffs);
    void radioRx(void ** buffs);
    void radioTx(int, void ** buffs);
    void radioRx(int, void ** buffs, long long & frameTime);
    void radioSched(std::vector<std::string> sched);
    ~RadioConfig();
private:
    Config *_cfg;
    std::vector<SoapySDR::Device *> hubs;
    std::vector<SoapySDR::Device *> baStn;
    std::vector<SoapySDR::Stream *> txStreams;
    std::vector<SoapySDR::Stream *> rxStreams;
    std::vector<std::complex<int16_t>> buff;
    int _radioNum;
    int _antennaNum;
    RadioType _radioType; 
};
