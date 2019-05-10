/*
 Copyright (c) 2018-2019, Rice University 
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
 Author(s): Rahman Doost-Mohamamdy: doost@rice.edu
 
---------------------------------------------------------------------
 Reads Configuration Parameters from file 
---------------------------------------------------------------------
*/


#include <algorithm>
#include <iostream>
#include <complex.h>
#include <stdio.h>  /* for fprintf */
#include <string.h> /* for memcpy */
#include <stdlib.h>
#include <unistd.h>
#include <vector>
#include <fstream>      // std::ifstream
#ifdef JSON
#include <nlohmann/json.hpp>
using json = nlohmann::json;
#endif

class Config
{
public:
    int sampsPerSymbol;
    int prefix;
    int postfix;
    int symbolsPerFrame;
    int pilotSymsPerFrame;
    int ulSymsPerFrame;
    int dlSymsPerFrame;
    
    std::string conf;
    std::string beacon_file;
    std::string pilot_file;
    std::string serial_file;
    std::string hub_file;
    std::vector<std::string> radio_ids;
    std::vector<std::string> hub_ids;
    std::vector<std::string> frames;
    std::vector<std::vector<size_t>> pilotSymbols;
    std::vector<std::vector<size_t>> ULSymbols;
    std::vector<std::vector<size_t>> DLSymbols;
    std::vector<std::complex<int16_t>> beacon;
    std::vector<std::vector<uint32_t>> beacon_weights;
    int beacon_ant;
    std::string beacon_mode;
    double freq;
    double txgainA;
    double rxgainA;
    double txgainB;
    double rxgainB;
    double rate;
    int framePeriod;
    int nCells;
    int nRadios;
    int nAntennas;
    int nUEs;
    int nChannels;
    const int maxFrame = 1 << 31;
    const int data_offset = sizeof(int) * 4;
    // header 4 int for: frame_id, subframe_id, cell_id, ant_id
    // ushort for: I/Q samples
    int getPackageLength() { return sizeof(int) * 4 + sizeof(ushort) * sampsPerSymbol * 2; }
    int getNumAntennas() { return nRadios*nChannels; }

    Config(std::string);
    ~Config();
private:
    static void loadData(char *, std::vector<std::complex<int16_t>> &, int);
    static void loadDevices(std::string, std::vector<std::string>&);
    static void printVector(std::vector<std::complex<int16_t>>&);
    static void loadTDDConfig(const std::string, std::string&);
    static std::vector<std::string> split(const std::string, char);
    static std::vector<std::vector<uint32_t>> getHadamard(int N);
};
