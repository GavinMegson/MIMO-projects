/*
 Copyright (c) 2018-2019, Rice University 
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
 Author(s): Rahman Doost-Mohamamdy: doost@rice.edu
 
---------------------------------------------------------------------
 main function 
---------------------------------------------------------------------
*/

#include "DataRecorder.hpp"
int main(int argc, char const *argv[])
{
    if(argc != 2)
    {
        std::cerr << "Usage: " << argv[0] << " CONFIG_FILE_NAME" << std::endl;
        return EXIT_FAILURE;
    }
    else if (strcmp(argv[1], "--help") == 0 or strcmp(argv[1], "-h") == 0)
    {
        std::cerr << "Usage: " << argv[0] << " CONFIG_FILE_NAME" << std::endl;
        return EXIT_SUCCESS;
    }

    Config *config = new Config(argv[1]);

    DataRecorder dr(config);
    time_t now = time(0);
    tm *ltm = localtime(&now);
    int bs_num = config->getNumAntennas(); 
    int ue_num = config->nUEs; 
    std::string filename = "logs/Argos-"+std::to_string(1900 + ltm->tm_year)+"-"+std::to_string(ltm->tm_mon)+"-"+std::to_string(ltm->tm_mday)+"-"+std::to_string(ltm->tm_hour)+"-"+std::to_string(ltm->tm_min)+"-"+std::to_string(ltm->tm_sec)+"_"+std::to_string(bs_num)+"x"+std::to_string(ue_num)+".hdf5";    
    if (dr.initHDF5(filename) < 0) return -1;
    dr.openHDF5();
    dr.start();

    return 0;
}
