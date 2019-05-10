/*
 Copyright (c) 2018-2019, Rice University 
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
 Author(s): Rahman Doost-Mohamamdy: doost@rice.edu

----------------------------------------------------------------------
 Record received frames from massive-mimo base station in HDF5 format
---------------------------------------------------------------------
*/

#include "DataRecorder.hpp"

DataRecorder::DataRecorder(Config *cfg)
{
    this->cfg = cfg;
    unsigned nCores = std::thread::hardware_concurrency();
    printf("number of CPU cores %d\n", nCores);
    rx_thread_num = nCores >= 2*SOCKET_THREAD_NUM and cfg->nRadios >= SOCKET_THREAD_NUM ? SOCKET_THREAD_NUM : 1; // FIXME: read number of cores and assing accordingly 
    printf("allocating %d cores to receive threads ... \n", rx_thread_num);
    task_queue_ = moodycamel::ConcurrentQueue<Event_data>(SOCKET_BUFFER_FRAME_NUM * cfg->symbolsPerFrame * cfg->getNumAntennas() * 36);
    message_queue_ = moodycamel::ConcurrentQueue<Event_data>(SOCKET_BUFFER_FRAME_NUM * cfg->symbolsPerFrame * cfg->getNumAntennas() * 36);
    
    // initialize socket buffer
    for(int i = 0; i < rx_thread_num; i++)
    {
        socket_buffer_[i].buffer.resize(cfg->getPackageLength() * cfg->getNumAntennas() * cfg->symbolsPerFrame * SOCKET_BUFFER_FRAME_NUM); // buffer SOCKET_BUFFER_FRAME_NUM entire frame
        socket_buffer_[i].buffer_status.resize(cfg->symbolsPerFrame * SOCKET_BUFFER_FRAME_NUM * cfg->getNumAntennas());
    }

    receiver_.reset(new DataReceiver(rx_thread_num, cfg, &message_queue_));

    // create task thread
    for(int i = 0; i < TASK_THREAD_NUM; i++)
    {
        context[i].obj_ptr = this;
        context[i].id = i;
        //printf("create thread %d\n", i);
        if(pthread_create( &task_threads[i], NULL, DataRecorder::taskThread, &context[i]) != 0)
        {
            perror("task thread create failed");
            exit(0);
        }
    }
}

herr_t DataRecorder::initHDF5(std::string hdf5)
{
    char tmpstr[1024];
    
    hdf5name = hdf5; 
    hdf5group = "/"; // make sure it starts with '/' 
    std::cout << "Set HDF5 File to " << hdf5name << " and group to " << hdf5group << std::endl;	

    // dataset dimension    
    dims_pilot[0] = MAX_FRAME_INC; //cfg->maxFrame; 
    dims_pilot[1] = cfg->nCells; 
    dims_pilot[2] = cfg->nUEs;
    dims_pilot[3] = cfg->getNumAntennas();
    dims_pilot[4] = 2 * cfg->sampsPerSymbol; // IQ
    hsize_t cdims[5] = {1, 1, 1, 1, 2 * cfg->sampsPerSymbol}; // pilot chunk size, FIXME: optimize size
    hsize_t max_dims_pilot[5] = {H5S_UNLIMITED, cfg->nCells, cfg->nUEs, cfg->getNumAntennas(), 2 * cfg->sampsPerSymbol};

    dims_data[0] = MAX_FRAME_INC; //cfg->maxFrame; 
    dims_data[1] = cfg->nCells; 
    dims_data[2] = cfg->ulSymsPerFrame;
    dims_data[3] = cfg->getNumAntennas();
    dims_data[4] = 2 * cfg->sampsPerSymbol; // IQ
    hsize_t cdims_data[5] = {1, 1, 1, 1, 2 * cfg->sampsPerSymbol}; // data chunk size, FIXME: optimize size
    hsize_t max_dims_data[5] = {H5S_UNLIMITED, cfg->nCells, cfg->ulSymsPerFrame, cfg->getNumAntennas(), 2 * cfg->sampsPerSymbol};

    try
    {
	Exception::dontPrint();

        file = new H5File(hdf5name, H5F_ACC_TRUNC);
        //group = new Group(file->createGroup("/Data"));
        file->createGroup("/Data");
        DataSpace *pilot_dataspace = new DataSpace(5, dims_pilot, max_dims_pilot);
        pilot_prop.setChunk(5, cdims);

        DataSet *pilot_dataset = new DataSet(file->createDataSet("/Data/Pilot_Samples", 
                                     PredType::STD_I16BE, *pilot_dataspace, pilot_prop) );

        hsize_t dims[1] = {1};
        //DataSpace attr_ds = DataSpace(H5S_SCALAR);
        DataSpace attr_ds = DataSpace (1, dims);

        int attr_data = cfg->sampsPerSymbol;
        Attribute att = pilot_dataset->createAttribute("SYMBOL_LEN", PredType::STD_I32BE, attr_ds);
        att.write(PredType::NATIVE_INT, &attr_data);

        attr_data = cfg->nCells; 
        att = pilot_dataset->createAttribute("NUM_CELLS", PredType::STD_I32BE, attr_ds);
        att.write(PredType::NATIVE_INT, &attr_data); 

        attr_data = cfg->nUEs;
        att = pilot_dataset->createAttribute("UE_NUM", PredType::STD_I32BE, attr_ds);
        att.write(PredType::NATIVE_INT, &attr_data); 

        attr_data = cfg->getNumAntennas();
        att = pilot_dataset->createAttribute("ANT_NUM", PredType::STD_I32BE, attr_ds);
        att.write(PredType::NATIVE_INT, &attr_data); 

        attr_data = cfg->nChannels;
        att = pilot_dataset->createAttribute("CH_PER_RADIO", PredType::STD_I32BE, attr_ds);
        att.write(PredType::NATIVE_INT, &attr_data); 

        attr_data = cfg->sampsPerSymbol-cfg->prefix-cfg->postfix;
        att = pilot_dataset->createAttribute("OFDM_LEN", PredType::STD_I32BE, attr_ds);
        att.write(PredType::NATIVE_INT, &attr_data); 

        attr_data = cfg->prefix;
        att = pilot_dataset->createAttribute("PREFIX_LEN", PredType::STD_I32BE, attr_ds);
        att.write(PredType::NATIVE_INT, &attr_data); 

        attr_data = cfg->postfix;
        att = pilot_dataset->createAttribute("POSTFIX_LEN", PredType::STD_I32BE, attr_ds);
        att.write(PredType::NATIVE_INT, &attr_data); 

        attr_data = cfg->symbolsPerFrame;
        att = pilot_dataset->createAttribute("FRAME_LEN", PredType::STD_I32BE, attr_ds);
        att.write(PredType::NATIVE_INT, &attr_data); 

        pilot_prop.close();
        pilot_dataspace->close();
        pilot_dataset->close();
        if(cfg->ulSymsPerFrame > 0)
        {
            DataSpace *data_dataspace = new DataSpace(5, dims_data, max_dims_data);
            data_prop.setChunk(5, cdims);

            DataSet *data_dataset = new DataSet(file->createDataSet("/Data/UplinkData", 
                                         PredType::STD_I16BE, *data_dataspace, data_prop) );

            att = data_dataset->createAttribute("UL_SYMS", PredType::STD_I32BE, attr_ds);
            att.write(PredType::NATIVE_INT, &(cfg->ulSymsPerFrame));
 
            data_prop.close();
            data_dataspace->close();
            data_dataset->close();
            config_dump_data = 1;
            delete data_dataspace;
            data_dataset->close();
        }
        else
        {
            config_dump_data_size = 0;
            config_dump_data = 0;	
        }
        //status = H5Gclose(group_id);
        //if (status < 0 ) return status;
        delete pilot_dataspace;
        pilot_dataset->close();
        file->close();
    }
    // catch failure caused by the H5File operations
    catch(FileIException error)
    {
	error.printError();
	return -1;
    }

    // catch failure caused by the DataSet operations
    catch(DataSetIException error)
    {
	error.printError();
	return -1;
    }

    // catch failure caused by the DataSpace operations
    catch(DataSpaceIException error)
    {
	error.printError();
	return -1;
    }
    maxFrameNumber = MAX_FRAME_INC;
    return 0;  // successfully terminated
}

void DataRecorder::openHDF5()
{
    file->openFile(hdf5name, H5F_ACC_RDWR);
    // Get Dataset for pilot and check the shape of it
    pilot_dataset = new DataSet(file->openDataSet("/Data/Pilot_Samples"));
 
    // Get the dataset's dataspace and creation property list.
    pilot_filespace = new DataSpace(pilot_dataset->getSpace());
    pilot_prop = pilot_dataset->getCreatePlist();

    // Get information to obtain memory dataspace.
    ndims = pilot_filespace->getSimpleExtentNdims();
    herr_t status_n = pilot_filespace->getSimpleExtentDims(dims_pilot);

    int cndims_pilot;
    if (H5D_CHUNKED == pilot_prop.getLayout())
        cndims_pilot = pilot_prop.getChunk(ndims, cdims_pilot);
    cout << "dim pilot chunk = " << cndims_pilot << endl;
    cout << "New Pilot Dataset Dimension " << ndims << "," << dims_pilot[0] << "," << dims_pilot[1] << "," << dims_pilot[2] << "," << dims_pilot[3] << "," << dims_pilot[4] << endl;
    pilot_filespace->close();
    // Get Dataset for DATA (If Enabled) and check the shape of it
    if((config_dump_data == 1))
    {
        data_dataset = new DataSet(file->openDataSet("/Data/UplinkData"));

        data_filespace = new DataSpace(data_dataset->getSpace());
        data_prop = data_dataset->getCreatePlist();
        ndims = data_filespace->getSimpleExtentNdims();
        status_n = data_filespace->getSimpleExtentDims(dims_data);

        int cndims_data;
        if (H5D_CHUNKED == data_prop.getLayout())
            cndims_data = data_prop.getChunk(ndims, cdims_data);
        cout << "dim data chunk = " << cndims_data << endl;;
        cout << "New Data Dataset Dimension " << ndims << "," << dims_data[0] << "," << dims_data[1] << "," << dims_data[2] << ","<< dims_data[3] << "," << dims_data[4] << endl;
        data_filespace->close();
    }
}

void DataRecorder::closeHDF5()
{
    int frameNumber = maxFrameNumber;

    // Resize Pilot Dataset
    dims_pilot[0] = frameNumber;
    pilot_dataset->extend(dims_pilot);
    pilot_prop.close();
    pilot_filespace->close();
    pilot_dataset->close();

    // Resize Data Dataset (If Needed)
    if(config_dump_data == 1)
    {
        dims_data[0] = frameNumber;
        data_dataset->extend(dims_data);
        data_prop.close();
        data_filespace->close();
        data_dataset->close();
    }


    file->close();

    cout << "Closing HDF5, " << frameNumber + 1 << " frames saved." << endl;
}

DataRecorder::~DataRecorder()
{
    this->closeHDF5();
}

void DataRecorder::start()
{
    // if ENABLE_CPU_ATTACH, attach main thread to core 0
#ifdef ENABLE_CPU_ATTACH
    if(stick_this_thread_to_core(0) != 0)
    {
        perror("stitch main thread to core 0 failed");
        exit(0);
    }
#endif
    // creare socket buffer and socket threads
    void* socket_buffer_ptrs[rx_thread_num];
    int* socket_buffer_status_ptrs[rx_thread_num];
    for(int i = 0; i < rx_thread_num; i++)
    {
        socket_buffer_ptrs[i] = socket_buffer_[i].buffer.data();
        socket_buffer_status_ptrs[i] = socket_buffer_[i].buffer_status.data();
    }
    std::vector<pthread_t> recv_thread = receiver_->startRecv(socket_buffer_ptrs, 
        socket_buffer_status_ptrs, socket_buffer_[0].buffer_status.size(), socket_buffer_[0].buffer.size(), 1);
    // for task_queue, main thread is producer, it is single-procuder & multiple consumer
    // for task queue
    moodycamel::ProducerToken ptok(task_queue_);
    // for message_queue, main thread is a consumer, it is multiple producers
    // & single consumer for message_queue
    moodycamel::ConsumerToken ctok(message_queue_);

    // counter for print log
    int miss_count = 0;
    int total_count = 0;

    Event_data events_list[dequeue_bulk_size];
    int ret = 0;
    while(true)
    {
        // get a bulk of events
        ret = message_queue_.try_dequeue_bulk(ctok, events_list, dequeue_bulk_size);
        total_count++;
        if(total_count == 1e7)
        {
            total_count = 0;
            miss_count = 0;
        }
        if(ret == 0)
        {
            miss_count++;
            continue;
        }
        // handle each event
        for(int bulk_count = 0; bulk_count < ret; bulk_count ++)
        {
            Event_data& event = events_list[bulk_count];

            // if EVENT_PACKAGE_RECEIVED, do crop
            if(event.event_type == EVENT_PACKAGE_RECEIVED)
            {
                int offset = event.data;
                Event_data do_crop_task;
                do_crop_task.event_type = TASK_CROP;
                do_crop_task.data = offset;
                if ( !task_queue_.try_enqueue(ptok, do_crop_task ) ) {
                    printf("need more memory\n");
                    if ( !task_queue_.enqueue(ptok, do_crop_task ) ) {
                        printf("crop task enqueue failed\n");
                        exit(0);
                    }
                }

                
            }
        }
    }
}

void* DataRecorder::taskThread(void* context)
{
    
    DataRecorder* obj_ptr = ((EventHandlerContext *)context)->obj_ptr;
    moodycamel::ConcurrentQueue<Event_data>* task_queue_ = &(obj_ptr->task_queue_);
    int tid = ((EventHandlerContext *)context)->id;
    printf("task thread %d starts\n", tid);
    
    // attach task threads to specific cores
#ifdef ENABLE_CPU_ATTACH2
    int offset_id = rx_thread_num + 1;
    int tar_core_id = tid + offset_id;
    if(tar_core_id >= 18)
        tar_core_id = (tar_core_id - 18) + 36;
    if(stick_this_thread_to_core(tar_core_id) != 0)
    {
        printf("stitch thread %d to core %d failed\n", tid, tar_core_id);
        exit(0);
    }
#endif

    obj_ptr->task_ptok[tid].reset(new moodycamel::ProducerToken(obj_ptr->message_queue_));

    int total_count = 0;
    int miss_count = 0;
    Event_data event;
    bool ret = false;
    while(true)
    {
        ret = task_queue_->try_dequeue(event);
        if(tid == 0)
            total_count++;
        if(tid == 0 && total_count == 1e6)
        {
            // print the task queue miss rate if required
            //printf("thread 0 task dequeue miss rate %f, queue length %d\n", (float)miss_count / total_count, task_queue_->size_approx());
            total_count = 0;
            miss_count = 0;
        }
        if(!ret)
        {
            if(tid == 0)
                miss_count++;
            continue;
        }

        // do different tasks according to task type
        if(event.event_type == TASK_CROP)
        {   
            obj_ptr->record(tid, event.data);
        }
    }
}

inline int DataRecorder::getUEId(int frame_id, int symbol_id)
{
    //return subframe_id-2; // fix
    std::vector<size_t>::iterator it;
    int fid = frame_id % cfg->framePeriod;
    it = find(cfg->pilotSymbols[fid].begin(), cfg->pilotSymbols[fid].end(), symbol_id);
    if (it != cfg->pilotSymbols[fid].end()) 
    {
#ifdef DEBUG1
        printf("getUEId(%d, %d) = %d\n",frame_id, symbol_id, it-cfg->pilotSymbols[fid].begin());
#endif
        return it-cfg->pilotSymbols[fid].begin();
    }else 
        return -1;
}

inline int DataRecorder::getUlSFIndex(int frame_id, int symbol_id)
{
    //return subframe_id-6; // fix
    std::vector<size_t>::iterator it;
    int fid = frame_id % cfg->framePeriod;
    it = find(cfg->ULSymbols[fid].begin(), cfg->ULSymbols[fid].end(), symbol_id);
    if (it != cfg->ULSymbols[fid].end()) 
    {
#ifdef DEBUG1
        printf("getUlSFIndexId(%d, %d) = %d\n",frame_id, symbol_id, it-cfg->ULSymbols[fid].begin());
#endif
        return it-cfg->ULSymbols[fid].begin();
    }else 
        return -1;
}

inline int DataRecorder::getDlSFIndex(int frame_id, int symbol_id)
{
    std::vector<size_t>::iterator it;
    int fid = frame_id % cfg->framePeriod;
    it = find(cfg->DLSymbols[fid].begin(), cfg->DLSymbols[fid].end(), symbol_id);
    if (it != cfg->DLSymbols[fid].end()) 
        return it-cfg->DLSymbols[fid].begin();
    else 
        return -1;
}

inline bool DataRecorder::isPilot(int frame_id, int symbol_id) 
{
    int fid = frame_id % cfg->framePeriod;
#ifdef DEBUG1
    printf("isPilot(%d, %d) = %c\n",frame_id, symbol_id, cfg->frames[fid].at(symbol_id));
#endif
    return cfg->frames[fid].at(symbol_id) == 'P' ? true : false;
} 
inline bool DataRecorder::isData(int frame_id, int symbol_id) 
{
    //return subframe_id == 6;
    int fid = frame_id % cfg->framePeriod;
#ifdef DEBUG1
    printf("isData(%d, %d) = %c\n",frame_id, symbol_id, cfg->frames[fid].at(symbol_id));
#endif
    return cfg->frames[fid].at(symbol_id) == 'U' ? true : false;
} 

// do Crop
herr_t DataRecorder::record(int tid, int offset)
{
    int buffer_frame_num = cfg->symbolsPerFrame * SOCKET_BUFFER_FRAME_NUM * cfg->getNumAntennas();
    int buffer_id = offset / buffer_frame_num;
    offset = offset - buffer_id * buffer_frame_num;
    // read info
    char* cur_ptr_buffer = socket_buffer_[buffer_id].buffer.data() + offset * cfg->getPackageLength();
    int ant_id, frame_id, symbol_id, cell_id;
    frame_id = *((int *)cur_ptr_buffer);
    symbol_id = *((int *)cur_ptr_buffer + 1);
    cell_id = *((int *)cur_ptr_buffer + 2);
    ant_id = *((int *)cur_ptr_buffer + 3);
#ifdef DEBUG1
    printf("record thread %d process frame_id %d, symbol_id %d, cell_id %d, ant_id %d\n", tid, frame_id, symbol_id, cell_id, ant_id);
            printf("record samples: %d %d %d %d %d %d %d %d ....\n",*((short *)cur_ptr_buffer+9), 
							   *((short *)cur_ptr_buffer+10),
                                                           *((short *)cur_ptr_buffer+11),
                                                           *((short *)cur_ptr_buffer+12),
                                                           *((short *)cur_ptr_buffer+13),
                                                           *((short *)cur_ptr_buffer+14),
                                                           *((short *)cur_ptr_buffer+15),
                                                           *((short *)cur_ptr_buffer+16)); 
#endif
    short* cur_ptr_buffer_short = (short*)(cur_ptr_buffer + sizeof(int) * 4);
    try
    {
	Exception::dontPrint();

        // Update the max frame number.
        // Note that the 'frameid' might be out of order.
        if(frame_id > maxFrameNumber)
        {
            // Open the hdf5 file if we haven't.
            closeHDF5();
            openHDF5();
            maxFrameNumber = maxFrameNumber + MAX_FRAME_INC;
        }
        hsize_t count[5];
        hsize_t hdfoffset[5];

        hdfoffset[0] = frame_id;
        hdfoffset[1] = 0;  // will change later after we use cell_id
        hdfoffset[2] = getUEId(frame_id, symbol_id);
        hdfoffset[3] = ant_id;
        hdfoffset[4] = 0;

        count[0] = 1; // frame
        count[1] = 1; // cell
        count[2] = 1; // ue num (subframe) 
        count[3] = 1; // ant 
        count[4] = dims_pilot[4]; // data size 

        if(isPilot(frame_id, symbol_id))
        {   
            assert(pilot_dataset >= 0);
            // Are we going to extend the dataset?
            if(frame_id >= dims_pilot[0])
            {
                dims_pilot[0] = dims_pilot[0] + config_pilot_extent_step; // 400 is a threshold.
                pilot_dataset->extend(dims_pilot);
                std::cout << "FrameId " << frame_id << ", (Pilot) Extent to " << dims_pilot[0] << " Frames" << std::endl;
            }
            // Select a hyperslab in extended portion of the dataset
            pilot_filespace = new DataSpace(pilot_dataset->getSpace());
            pilot_filespace->selectHyperslab(H5S_SELECT_SET, count, hdfoffset);

            // define memory space
            DataSpace *pilot_memspace = new DataSpace(5, count, NULL);
            pilot_dataset->write(cur_ptr_buffer_short, PredType::NATIVE_INT16, *pilot_memspace, *pilot_filespace);
            pilot_filespace->close();
            delete pilot_memspace;
        }
        else if(isData(frame_id, symbol_id)) 
        {
            
            assert(data_dataset >= 0);
            // Are we going to extend the dataset?
            if(frame_id >= dims_data[0])
            {
                dims_data[0] = dims_data[0] + config_data_extent_step;
                data_dataset->extend(dims_data);
                printf("(Data) Extent to %d Frames\n",dims_data[0]);
            }

            hdfoffset[2] = getUlSFIndex(frame_id, symbol_id);
            count[0] = 1;
            count[1] = 1;
            count[2] = 1;
            count[3] = 1;
            count[4] = dims_data[4];

            // Select a hyperslab in extended portion of the dataset
            data_filespace = new DataSpace(data_dataset->getSpace());
            data_filespace->selectHyperslab(H5S_SELECT_SET, count, hdfoffset);

            // define memory space
            DataSpace *data_memspace = new DataSpace(5, count, NULL);
            data_dataset->write(cur_ptr_buffer_short, PredType::NATIVE_INT16, *data_memspace, *data_filespace);
            delete data_memspace;
        }
    } 
    // catch failure caused by the H5File operations
    catch(FileIException error)
    {
	error.printError();
	return -1;
    }

    // catch failure caused by the DataSet operations
    catch(DataSetIException error)
    {
	error.printError();
        std::cout << "DataSet: Failed to record pilots from frame " << frame_id << " , UE " << getUEId(frame_id, symbol_id) << " antenna " << ant_id << " dims_pilot[4] " << dims_pilot[4] << std::endl;
        std::cout << "Dataset Dimension is " << ndims << "," << dims_pilot[0] << "," << dims_pilot[1] << "," << dims_pilot[2] << "," << dims_pilot[3] << "," << dims_pilot[4] << endl;
	return -1;
    }

    // catch failure caused by the DataSpace operations
    catch(DataSpaceIException error)
    {
	error.printError();
	return -1;
    }

    // after finish
    socket_buffer_[buffer_id].buffer_status[offset] = 0; // now empty
    return 0;
}

