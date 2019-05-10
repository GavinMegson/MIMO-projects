/*
 Copyright (c) 2018-2019, Rice University 
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
 Author(s): Rahman Doost-Mohamamdy: doost@rice.edu
 
---------------------------------------------------------------------
 Setup User Terminal(s) for channel sounding 
---------------------------------------------------------------------
*/

#include <SoapySDR/Device.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Errors.hpp>
#include <SoapySDR/Time.hpp>
#include "utils.hpp"

#ifdef JSON
#include <nlohmann/json.hpp>
using json = nlohmann::json;
#endif

static sig_atomic_t loopDone = false;
std::atomic_int thread_count(0);
std::mutex d_mutex;
std::condition_variable cond;

void sigIntHandler(const int)
{
    loopDone = true;
}

void txrx(void * context, void* data)
{
    dev_profile *profile = (dev_profile *)context;
    SoapySDR::Device * device = profile->device;
    SoapySDR::Stream * rxStream = profile->rxs;
    SoapySDR::Stream * txStream = profile->txs;
    int id = profile->id;
    int txSyms = profile->txSyms; 
    int rxSyms = profile->rxSyms; 
    int txStartSym = profile->txStartSym;
    double rate = profile->rate;
    unsigned txFrameDelta = profile->txFrameDelta; 
    int NUM_SAMPS = profile->nsamps;

    while(!d_mutex.try_lock()){}
    thread_count++;
    std::cout << "Thread " << id << ", txSyms " << txSyms << ", rxSyms " << rxSyms << ", txStartSym " << txStartSym << ", rate " << rate << ", txFrameDelta " << txFrameDelta << ", nsamps " << NUM_SAMPS << std::endl;
    d_mutex.unlock();
 
    std::vector<std::complex<int16_t>> buffs(NUM_SAMPS, 0);
    std::vector<void *> rxbuff(2);
    rxbuff[0] = buffs.data();
    rxbuff[1] = buffs.data();

    std::vector<void *> txbuff(2);
    txbuff[0] = data;
    txbuff[1] = data;

    device->activateStream(rxStream);
    device->activateStream(txStream);

    bool exitLoop = false;
    while (not exitLoop)
    {
        exitLoop = loopDone;

        // receiver loop
        long long rxTime(0);
        long long txTime(0);
        long long firstRxTime (0);
        bool receiveErrors = false;
        for (int i = 0; i < rxSyms; i++)
        {
            int flags(0);
            int r = device->readStream(rxStream, rxbuff.data(), NUM_SAMPS, flags, rxTime, 1000000);
            if (r == NUM_SAMPS)
            {
                if (i == 0) firstRxTime = rxTime;
            }
            else
            {
                std::cerr << "waiting for receive frames... " << std::endl;
                receiveErrors = true;
                break; 
            }
        }
        if (receiveErrors) continue; // just go to the next frame

        // transmit loop
        int flags = SOAPY_SDR_HAS_TIME;
        txTime = firstRxTime & 0xFFFFFFFF00000000;
        txTime += ((long long)txFrameDelta << 32); 
        txTime += ((long long)txStartSym << 16);
        //printf("rxTime %llx, txTime %llx \n", firstRxTime, txTime);
        if (exitLoop) flags |= SOAPY_SDR_END_BURST; //end burst on last iter
        bool transmitErrors = false;
        for (int i = 0; i < txSyms; i++)
        {
            if (i == txSyms - 1)  flags |= SOAPY_SDR_END_BURST;
            int r = device->writeStream(txStream, txbuff.data(), NUM_SAMPS, flags, txTime, 1000000);
            if (r == NUM_SAMPS)
            {
                txTime += 0x10000;
            }
            else
            {
                std::cerr << "unexpected writeStream error " << SoapySDR::errToStr(r) << std::endl;
                transmitErrors = true;
                //goto cleanup;
            }
        }

    }

    device->deactivateStream(rxStream);
    device->deactivateStream(txStream);

    device->closeStream(rxStream);
    device->closeStream(txStream);


    std::lock_guard<std::mutex> lock(d_mutex); 
    device->writeRegister("IRIS30", CORR_CONF, 0);
    std::cout << "device " << id << " T=" << std::hex << SoapySDR::timeNsToTicks(device->getHardwareTime(""), rate) << std::endl;
    for (int i = 0; i < 20; i++)
    {
        device->writeRegister("RFCORE", SCH_ADDR_REG, i);
        device->writeRegister("RFCORE", SCH_MODE_REG, 0);
    }
    device->writeSetting("TDD_MODE", "false");
    device->writeRegister("IRIS30", RF_RST_REG, (1<<29) | 1);
    device->writeRegister("IRIS30", RF_RST_REG, (1<<29));
    device->writeRegister("IRIS30", RF_RST_REG, 0);
    SoapySDR::Device::unmake(device);
    free(context);
    thread_count--;
    cond.notify_one();
}

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        std::cerr << "Usage: " << argv[0] << " CONFIG_FILE_NAME" << std::endl;
        return EXIT_FAILURE;
    }
    else if (strcmp(argv[1], "--help") == 0 or strcmp(argv[1], "-h") == 0)
    {
        std::cerr << "Usage: " << argv[0] << " CONFIG_FILE_NAME" << std::endl;
        return EXIT_SUCCESS;
    }

    std::string configfile(argv[1]);
    std::string config;
    loadTDDConfig(configfile, config);
    std::vector<std::string> serials;
    std::vector<std::string> frames;
#ifdef JSON
    const auto tddConf = json::parse(config);
    auto nChannels = tddConf.value("channels", 1);
    auto freq = tddConf.value("frequency", 3.6e9);
    auto txgain = tddConf.value("txgain", 20.0);
    auto fegain = tddConf.value("fegain", 0.0);
    auto rxgain = tddConf.value("rxgain", 20.0);
    auto rate = tddConf.value("rate", 5e6);
    auto sampsPerSymbol = tddConf.value("symbol_size", 0);
    auto prefix = tddConf.value("prefix", 0);
    auto postfix = tddConf.value("postfix", 0);
    auto txTimeDelta = tddConf.value("tx_delta", 4.0); // in miliseconds
    std::string coeffs_file = tddConf.value("coeffs", "coeffs.txt");
    std::string pilot_file = tddConf.value("pilot", "pilot.txt");
    std::string data_file = tddConf.value("txdata", "txdata.txt");
    auto jserials = tddConf.value("serials", json::array());
    for (int n = 0; n < jserials.size(); n++) serials.push_back(jserials.at(n).get<std::string>());
    auto jframes = tddConf.value("frames", json::array());
    for (int n = 0; n < jframes.size(); n++) frames.push_back(jframes.at(n).get<std::string>());
#else    
    std::string coeffs_file, pilot_file, data_file;
    unsigned nChannels, sampsPerSymbol, prefix, postfix;
    double freq, txgain, fegain, rxgain, rate, txTimeDelta;
    std::vector<std::string> paramlist = split(config, '/');
    std::cout << paramlist.size() << std::endl;
    for (int p = 0; p < paramlist.size(); p++)
    {
        std::vector<std::string> param = split(paramlist[p], '=');
        std::cout << param[0] << ":" << param[1] << std::endl;
        if (param[0] == "channels") nChannels =  std::stoul(param[1],nullptr, 0);
	else if (param[0] == "frequency") freq =  std::stod(param[1]);
	else if (param[0] == "txgain") txgain =  std::stod(param[1]);
	else if (param[0] == "fegain") fegain =  std::stod(param[1]);
	else if (param[0] == "rxgain") rxgain =  std::stod(param[1]);
	else if (param[0] == "rate") rate =  std::stod(param[1]);
	else if (param[0] == "tx_delta") txTimeDelta =  std::stod(param[1]);
	else if (param[0] == "symbol_size") sampsPerSymbol =  std::stoul(param[1],nullptr, 0);
        else if (param[0] == "prefix") prefix =  std::stoul(param[1],nullptr, 0);
        else if (param[0] == "postfix") postfix =  std::stoul(param[1],nullptr, 0);
        else if (param[0] == "coeffs") coeffs_file =  param[1];
        else if (param[0] == "pilot") pilot_file =  param[1];
        else if (param[0] == "txdata") data_file =  param[1];
        else if (param[0] == "serials") serials =  split(param[1], ',');
        else if (param[0] == "frames") frames =  split(param[1], ',');
    }
#endif

    std::vector<std::vector<size_t>> pilotSymbols = loadSymbols(frames, 'P');
    std::vector<std::vector<size_t>> ULSymbols = loadSymbols(frames, 'T');
    std::vector<std::vector<size_t>> DLSymbols = loadSymbols(frames, 'R');

    //std::vector<std::complex<int16_t>> coeffs_ci16(128,0);
    //loadData(coeffs_file.c_str(), coeffs_ci16, 128);

    std::vector<uint32_t> coeffs; //= cint16_to_uint32(coeffs_ci16, true, "QI");
    loadData(coeffs_file.c_str(), coeffs, 128);  
    //for (int i = 0; i < coeffs.size(); i++)
    //    std::cout << coeffs[i] << " ";
    //std::cout << std::endl;

    std::vector<std::complex<int16_t>> pre(prefix, 0);
    std::vector<std::complex<int16_t>> post(postfix, 0);

    std::vector<std::complex<int16_t>> data_ci16(sampsPerSymbol-prefix-postfix,0);
    loadData(data_file.c_str(), data_ci16, sampsPerSymbol-prefix-postfix);
    data_ci16.insert(data_ci16.begin(), pre.begin(), pre.end());
    data_ci16.insert(data_ci16.end(), post.begin(), post.end());

    std::vector<std::complex<int16_t>> pilot_ci16(sampsPerSymbol-prefix-postfix,0);
    loadData(pilot_file.c_str(), pilot_ci16, sampsPerSymbol-prefix-postfix);
    pilot_ci16.insert(pilot_ci16.begin(), pre.begin(), pre.end());
    pilot_ci16.insert(pilot_ci16.end(), post.begin(), post.end());

    std::vector<uint32_t> pilot;// = cint16_to_uint32(pilot_ci16, false, "QI");
    loadData(pilot_file.c_str(), pilot, sampsPerSymbol);  
    //for (int i = 0; i < pilot.size(); i++)
    //    std::cout << pilot[i] << " ";
    //std::cout << std::endl;

    int ueTrigOffset = prefix + 256 + postfix + 17 + prefix;
    int sf_start = ueTrigOffset/sampsPerSymbol;
    int sp_start = ueTrigOffset%sampsPerSymbol;

    //load channels
    std::vector<size_t> channels;
    if (nChannels == 1) channels = {0};
    else if (nChannels == 2) channels = {0, 1};
    else
    {
        std::cerr << "Error! Supported number of channels 1 or 2" << std::endl;
        return EXIT_FAILURE;
    }

    double frameTime = sampsPerSymbol*frames[0].size()*1e3/rate; // miliseconds
    unsigned frameTimeDelta = (unsigned)(std::ceil(txTimeDelta/frameTime)); 
    std::cout << "Frame time delta " << frameTimeDelta << std::endl;

    unsigned nRadios = serials.size();
    if (nRadios != frames.size())
    {
        std::cerr << " serials size does not match frames size!" << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << "setting samples rates to " << rate/1e6 << " Msps..." << std::endl;

    std::vector<SoapySDR::Device *> devs;
    std::vector<SoapySDR::Stream *> rxss;
    std::vector<SoapySDR::Stream *> txss;
    for(size_t n = 0; n < nRadios; n++)
    {
        auto device = SoapySDR::Device::make("serial="+serials.at(n));
        if (device == nullptr)
        {
            std::cerr << "No device!" << std::endl;
            return EXIT_FAILURE;
        }
        devs.push_back(device);
        SoapySDR::Kwargs info = device->getHardwareInfo();

        for (auto ch : channels)
        {
            device->setSampleRate(SOAPY_SDR_RX, ch, rate);
            device->setSampleRate(SOAPY_SDR_TX, ch, rate);

            //device->setFrequency(SOAPY_SDR_RX, ch, freq);  
            //device->setFrequency(SOAPY_SDR_TX, ch, freq); 
            device->setFrequency(SOAPY_SDR_RX, ch, "RF", freq-.75*rate);
            device->setFrequency(SOAPY_SDR_RX, ch, "BB", .75*rate);
            device->setFrequency(SOAPY_SDR_TX, ch, "RF", freq-.75*rate);
            device->setFrequency(SOAPY_SDR_TX, ch, "BB", .75*rate);
 
            if (info["frontend"].find("CBRS") != std::string::npos)
            {
                device->setGain(SOAPY_SDR_RX, ch, "ATTN", 0); //[-18,0]  
                device->setGain(SOAPY_SDR_RX, ch, "LNA1", 30); //[0,33]
                device->setGain(SOAPY_SDR_RX, ch, "LNA2", 17); //[0,17]
            }

            device->setGain(SOAPY_SDR_RX, ch, "LNA", rxgain);  //[0,30]
            device->setGain(SOAPY_SDR_RX, ch, "TIA", 0);  //[0,12]
            device->setGain(SOAPY_SDR_RX, ch, "PGA", 0);  //[-12,19]

            if (info["frontend"].find("CBRS") != std::string::npos)
            {
                device->setGain(SOAPY_SDR_TX, ch, "ATTN", 0);       //[-18,0] by 3
                device->setGain(SOAPY_SDR_TX, ch, "PA1", 15);       //[0|15]
                device->setGain(SOAPY_SDR_TX, ch, "PA2", fegain);   //[0|15]
                device->setGain(SOAPY_SDR_TX, ch, "PA3", 30);       //[0|30]
            }
            device->setGain(SOAPY_SDR_TX, ch, "IAMP", 12);          //[0,12] 
            if (fegain > 0 and txgain > 35) txgain = 35;
            device->setGain(SOAPY_SDR_TX, ch, "PAD", txgain);       //[0,52] 
        }

        for (auto ch : channels)
        {
            //device->writeSetting(SOAPY_SDR_RX, ch, "CALIBRATE", "SKLK");
            //device->writeSetting(SOAPY_SDR_TX, ch, "CALIBRATE", "");
            device->setDCOffsetMode(SOAPY_SDR_RX, ch, true);
        }

        if (nChannels == 1)
        {
            device->writeSetting(SOAPY_SDR_RX, 1, "ENABLE_CHANNEL", "false");
            device->writeSetting(SOAPY_SDR_TX, 1, "ENABLE_CHANNEL", "false");
            device->writeSetting("SPI_TDD_MODE", "SISO"); // a FPGA hack that bypasses the LDO issue
        } else
            device->writeSetting("SPI_TDD_MODE", "MIMO");

        device->writeRegister("IRIS30", RF_RST_REG, (1<<29) | 1);
        device->writeRegister("IRIS30", RF_RST_REG, (1<<29));
        device->writeRegister("IRIS30", RF_RST_REG, 0);

        std::cout << "create streams..." << std::endl;
        auto rxStream = device->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, channels);
        auto txStream = device->setupStream(SOAPY_SDR_TX, SOAPY_SDR_CS16, channels);
        rxss.push_back(rxStream);
        txss.push_back(txStream);

        device->writeRegister("IRIS30", CORR_CONF, 0x1);
        for (int i = 0; i < 128; i++)
            device->writeRegister("ARGCOE", i*4, 0);
        usleep(100000);
        device->writeRegister("ARGCOR", CORR_THRESHOLD, 128);
        device->writeRegister("ARGCOR", CORR_RST, 1);
        device->writeRegister("ARGCOR", CORR_RST, 0);
        for (int i = 0; i < 128; i++)
            device->writeRegister("ARGCOE", i*4, coeffs[i]);

#ifdef JSON
        json conf;
        conf["tdd_enabled"] = true;
        conf["trigger_out"] = true;
        conf["wait_trigger"] = true;
        conf["frames"] = json::array();
        conf["frames"].push_back(frames[n]);
        conf["symbol_size"] = sampsPerSymbol; 
	std::string confString = conf.dump();
#else
        std::string confString ="{\"tdd_enabled\":true,\"trigger_out\":false,\"wait_trigger\":true,";
        confString +="\"symbol_size\":"+std::to_string(sampsPerSymbol);
        confString +=",\"frames\":[\""+frames[n]+"\"]}";
	std::cout << confString << std::endl;
#endif
        device->writeSetting("TDD_CONFIG", confString);

        device->setHardwareTime(SoapySDR::ticksToTimeNs((sf_start << 16) | sp_start, rate), "TRIGGER");
        device->writeSetting("TX_SW_DELAY", "30"); // experimentally good value for dev front-end
        device->writeSetting("TDD_MODE", "true");
        // write beacons to FPGA buffers
        device->writeRegisters("TX_RAM_A", 0, pilot);
        if (nChannels == 2)
            device->writeRegisters("TX_RAM_B", 0, pilot); // no dual channel beacon for now
        device->writeRegister("IRIS30", CORR_CONF, 0x11);
        
    }

    std::cout << "Initializing Threads ..." << std::endl;
    std::vector<std::thread> threads(nRadios);
    for (size_t i = 0; i < nRadios; i++)
    {
        dev_profile *profile = (dev_profile *)malloc(sizeof(dev_profile));
        profile->id = i;
        profile->rate = rate;
        profile->nsamps = sampsPerSymbol;
        profile->txSyms = ULSymbols[i].size();
        profile->rxSyms = DLSymbols[i].size();
        profile->txStartSym = ULSymbols[i].size() > 0 ? ULSymbols[i][0] : 0;
        profile->txFrameDelta = frameTimeDelta;
        profile->device = devs[i];
        profile->rxs = rxss[i];
        profile->txs = txss[i];
        threads[i] = std::thread(txrx, (void*)profile, data_ci16.data()); 
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(i+1, &cpuset);
        int rc = pthread_setaffinity_np(threads[i].native_handle(),
                                        sizeof(cpu_set_t), &cpuset);
        if (rc != 0) {
          std::cerr << "Error calling pthread_setaffinity_np: " << rc << "\n";
        }
    }
    
    for (auto& t : threads){
        t.detach();
    }

    usleep(100000);
    std::cout << "Press Ctrl+C to stop..." << std::endl;
    signal(SIGINT, sigIntHandler);
    std::unique_lock<std::mutex> lock(d_mutex);
    cond.wait(lock, [](){ return thread_count == 0; });

}

