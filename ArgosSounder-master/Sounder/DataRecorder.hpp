/*
 Copyright (c) 2018-2019 
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
 Author(s): Peiyao Zhao 
            Rahman Doost-Mohamamdy, doost@rice.edu
 
----------------------------------------------------------------------
 Record received frames from massive-mimo base station in HDF5 format
---------------------------------------------------------------------
*/

#include <unistd.h>
#include <memory>
#include <fcntl.h>
#include <system_error>
#include <pthread.h>
#include <queue>
#include <math.h>
#include <ctime>
#include <algorithm>
#include "cpu_attach.hpp"
#include "concurrentqueue.h"
#include "hdf5.h"
#include "H5Cpp.h"
#include "DataReceiver.hpp"

using std::cout;
using std::endl;
using namespace H5;
class DataRecorder
{
public:
    // TASK & SOCKET thread number 
    static const int TASK_THREAD_NUM = 1;
    static const int SOCKET_THREAD_NUM = 4; 
    // buffer length of each socket thread
    static const int SOCKET_BUFFER_FRAME_NUM = 80;
    // buffer length of recording part 
    static const int TASK_BUFFER_FRAME_NUM = 60;
    // dequeue bulk size, used to reduce the overhead of dequeue in main thread
    static const int dequeue_bulk_size = 5;

    DataRecorder(Config *cfg);
    ~DataRecorder();

    void start();
    herr_t initHDF5(std::string);
    void openHDF5();
    void closeHDF5();
    static void* taskThread(void* context);
    herr_t record(int tid, int offset);

    struct EventHandlerContext
    {
        DataRecorder* obj_ptr;
        int id;
    };

    inline int getUEId(int, int);
    inline int getUlSFIndex(int, int);
    inline int getDlSFIndex(int, int);
    inline bool isPilot(int, int);
    inline bool isData(int, int);

private:
    Config *cfg;
    std::unique_ptr<DataReceiver> receiver_;
    SocketBuffer socket_buffer_[SOCKET_THREAD_NUM];

    H5std_string hdf5name;
    H5std_string hdf5group;
    
    H5File *file;
    Group *group;
    DSetCreatPropList pilot_prop;
    DSetCreatPropList data_prop;

    DataSpace *pilot_filespace;
    DataSpace *data_filespace;

    DataSet   *pilot_dataset;
    DataSet   *data_dataset;
    
    int ndims;
    int cndims;
    
    hsize_t dims_pilot[5];
    hsize_t dims_data[5];
    hsize_t cdims_pilot[5];
    hsize_t cdims_data[5];
    
    int config_pilot_extent_step = 400;
    int config_data_extent_step = 400;
    int config_dump_data;
    int config_dump_data_size;
    int maxFrameNumber;
    int rx_thread_num;
    moodycamel::ConcurrentQueue<Event_data> task_queue_;
    moodycamel::ConcurrentQueue<Event_data> message_queue_;
    pthread_t task_threads[TASK_THREAD_NUM];

    EventHandlerContext context[TASK_THREAD_NUM];

    std::unique_ptr<moodycamel::ProducerToken> task_ptok[TASK_THREAD_NUM];

};

