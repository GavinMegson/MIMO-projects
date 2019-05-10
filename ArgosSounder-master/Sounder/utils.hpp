/*
 Copyright (c) 2018-2019, Rice University 
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
 Author(s): Rahman Doost-Mohamamdy: doost@rice.edu
 
---------------------------------------------------------------------
 Utils functions  
---------------------------------------------------------------------
*/

#include <iostream>
#include <fstream>      // std::ifstream
#include <sstream>
#include <cstdlib>
#include <cstddef>
#include <chrono>
#include <string>
#include <cstdint>
#include <complex>
#include <csignal>
#include <thread>
#include <unistd.h>
#include <mutex>
#include <condition_variable>
#include <atomic>
#define JSON
#define CORR_THRESHOLD    0x4
#define CORR_RST          0x0
#define CORR_SCNT         0x8
#define CORR_CONF         60
#define RF_RST_REG        48
#define TDD_CONF_REG      120
#define SCH_ADDR_REG      136
#define SCH_MODE_REG      140
#define TX_GAIN_CTRL      88

typedef struct
{
    int id;
    int nsamps;
    int txSyms;
    int rxSyms;
    int txStartSym;
    unsigned txFrameDelta;
    double rate;
    SoapySDR::Device * device;
    SoapySDR::Stream * rxs;
    SoapySDR::Stream * txs;
    std::string data_file;
} dev_profile;

std::vector<uint32_t> cint16_to_uint32(std::vector<std::complex<int16_t>> in, bool conj, std::string order)
{
    std::vector<uint32_t> out (in.size(), 0);
    for (int i = 0; i < in.size(); i++)
    {
       uint16_t re = (uint16_t)in[i].real(); 
       uint16_t im = (uint16_t)(conj ? -in[i].imag() : in[i].imag());
       if (order == "IQ")
           out[i] = (uint32_t)re << 16 | im;
       else if (order == "QI")
           out[i] = (uint32_t)im << 16 | re;
    }
    return out;
}

std::vector<std::vector<size_t>> loadSymbols(std::vector<std::string> frames, char sym)
{
    std::vector<std::vector<size_t>> symId;
    int frameSize = frames.size();
    symId.resize(frameSize);
    for(int f = 0; f < frameSize; f++)
    {
        std::string fr = frames[f]; 
        for (int g = 0; g < fr.size(); g++)
        {
            if (fr[g] == sym){
                symId[f].push_back(g);
            }
        }
    }
    return symId;
}

void loadDevices(std::string filename, std::vector<std::string> &data)
{
   std::string line;
   std::ifstream myfile (filename, std::ifstream::in);
   if (myfile.is_open())
   {
     while ( getline (myfile,line) )
     {
       data.push_back(line);
       std::cout << line << '\n';
     }
     myfile.close();
   }
 
   else std::cout << "Unable to open file"; 
}

void loadData(const char* filename, std::vector<std::complex<int16_t>> &data, int samples)
{
    FILE* fp = fopen(filename,"r");
    data.resize(samples);
    float real, imag;
    for(int i = 0; i < samples; i++)
    {
    	fscanf(fp, "%f %f", &real, &imag);
        data[i] = std::complex<int16_t>(int16_t(real*32768*2), int16_t(imag*32768*2));
    }
    
    fclose(fp);
}

void loadData(const char* filename, std::vector<unsigned> &data, int samples)
{
    FILE* fp = fopen(filename,"r");
    data.resize(samples);
    unsigned sample;
    for(int i = 0; i < samples; i++)
    {
        fscanf(fp, "%u", &data[i]);
    }
    
    fclose(fp);
}

void loadTDDConfig(const std::string filename, std::string &jconfig)
{
    std::string line;
    std::ifstream configFile(filename);
    if (configFile.is_open())
    {
        while ( getline (configFile,line) )
        {
          jconfig += line;
        }
        configFile.close();
    }

    else std::cout << "Unable to open file"; 

}

std::vector<std::string> split(const std::string& s, char delimiter)
{
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    while (std::getline(tokenStream, token, delimiter))
    {
        tokens.push_back(token);
    }
    return tokens;
}


