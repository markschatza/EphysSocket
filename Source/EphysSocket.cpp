#ifdef _WIN32
#include <Windows.h>
#endif

#include "EphysSocket.h"
#include "EphysSocketEditor.h"
#include "SocketLSLBrainAmp.h"

using namespace EphysSocketNode;

DataThread* EphysSocket::createDataThread(SourceNode *sn)
{
    return new EphysSocket(sn);
}


EphysSocket::EphysSocket(SourceNode* sn) : DataThread(sn),
    port(DEFAULT_PORT),
    num_channels(DEFAULT_NUM_CHANNELS),
    num_samp(DEFAULT_NUM_SAMPLES),
    data_offset(DEFAULT_DATA_OFFSET), // not used??
    data_scale(DEFAULT_DATA_SCALE),
    sample_rate(DEFAULT_SAMPLE_RATE),
    cur_protocol(LSL)
{

    switch (cur_protocol) {
    case BONSAI:
        socket = new DatagramSocket();
        socket->bindToPort(port);
        connected = (socket->waitUntilReady(true, 500) == 1); // Try to automatically open, dont worry if it does not work


        // different for each one
        sourceBuffers.add(new DataBuffer(num_channels, 10000)); // start with 2 channels and automatically resize
        recvbuf = (uint16_t*)malloc(num_channels * num_samp * 2);
        convbuf = (float*)malloc(num_channels * num_samp * 4);
        ///
        break;
    case LSL:
        num_channels = 8;
        num_samp = 100;
        inlet = new LSLinlet(&sample_rate, &num_channels, num_samp);
        if (inlet->success) {
            connected = true;
        }
        // different for each one
        sourceBuffers.add(new DataBuffer(num_channels, 10000)); 
        recvbuf = (uint16_t*)malloc(num_channels * num_samp * 2);
        convbuf = (float*)malloc(num_channels * num_samp * sizeof(float));
    }
    
}

GenericEditor* EphysSocket::createEditor(SourceNode* sn)
{
    return new EphysSocketEditor(sn, this);
}



EphysSocket::~EphysSocket()
{
    // different for each one
    free(recvbuf);
    free(convbuf);
}


void EphysSocket::resizeChanSamp()
{
    // different for each one
    switch(cur_protocol) {
    case BONSAI:
        sourceBuffers[0]->resize(num_channels, 10000);
        recvbuf = (uint16_t*)realloc(recvbuf, num_channels * num_samp * 2);
        convbuf = (float*)realloc(convbuf, num_channels * num_samp * 4);
        timestamps.resize(num_samp);
        ttlEventWords.resize(num_samp);
        break;
    case LSL:
        sourceBuffers[0]->resize(num_channels, 10000);
        recvbuf = (uint16_t*)realloc(recvbuf, num_channels * num_samp * 2);
        convbuf = (float*)realloc(convbuf, num_channels * num_samp * sizeof(float));
        //timestamps.resize(num_samp);
        timestamps.resize(10000);
        ttlEventWords.resize(num_samp);
        break;
    }

}


void EphysSocket::setComProtocol(int protocol_index)
{
    cur_protocol = protocol_index;
    tryToConnect(); 
}


////////////
/////////// These are for other plugins to query the datathread (default OEPlugin functions)
/*
* Maybe not this one though
*/ 
int EphysSocket::getNumChannels() const
{
    return num_channels;
}

int EphysSocket::getNumDataOutputs(DataChannel::DataChannelTypes type, int subproc) const
{
    if (type == DataChannel::HEADSTAGE_CHANNEL)
        return num_channels;
    else
        return 0; 
}

/*
* Most likely need to update for some data types
*/
int EphysSocket::getNumTTLOutputs(int subproc) const
{
    switch (cur_protocol)
    {
    case BONSAI:
        return 0;
    case LSL:
        return 0;
    case RDA:
        return 0;
    }
    
    return 0; 
}

float EphysSocket::getSampleRate(int subproc) const
{
    return sample_rate;
}

float EphysSocket::getBitVolts (const DataChannel* ch) const
{
    return data_scale;
}


bool EphysSocket::foundInputSource()
{
    return connected;
}

bool EphysSocket::startAcquisition()
{
    // most likely different for each type
    resizeChanSamp();

    total_samples = 0;

    startTimer(5000);

    startThread();
    return true;
}

void  EphysSocket::tryToConnect()
{
    switch (cur_protocol)
    {
    case BONSAI: {
        // should always be the same
        socket->shutdown();
        socket = new DatagramSocket();
        bool bound = socket->bindToPort(port);
        if (bound)
        {
            std::cout << "Socket bound to port " << port << std::endl;
            connected = (socket->waitUntilReady(true, 500) == 1);
        }
        else {
            std::cout << "Could not bind socket to port " << port << std::endl;
        }


        if (connected)
        {
            std::cout << "Socket connected." << std::endl;

        }
        else {
            std::cout << "Socket failed to connect" << std::endl;
        }
        break;
    }
        
    case LSL: {
        
        connected = inlet->connectToStream(&sample_rate, &num_channels, num_samp);
    
        break;
    }
    case RDA: {
        // to fill
    
        break;
    }
    }
    
}

bool EphysSocket::stopAcquisition()
{
    // should always be the same
    if (isThreadRunning())
    {
        signalThreadShouldExit();
    }

    waitForThreadToExit(500);

    stopTimer();

    sourceBuffers[0]->clear();
    return true;
}

bool EphysSocket::updateBuffer()
{
    // need updateBuffer() function for each type
    // Basically setting sourceBuffers here. which is our data + timestamps + ttl events
    switch (cur_protocol)
    {
    case BONSAI: {
        int rc = socket->read(recvbuf, num_channels * num_samp * 2, true);

        if (rc == -1)
        {
            CoreServices::sendStatusMessage("Ephys Socket: Data shape mismatch");
            return false;
        }

        // Transpose because the chunkSize argument in addToBuffer does not seem to do anything
        if (transpose) {
            int k = 0;
            for (int i = 0; i < num_samp; i++) {
                for (int j = 0; j < num_channels; j++) {
                    convbuf[k++] = 0.195 * (float)(recvbuf[j * num_samp + i] - 32768);
                }
            }
        }
        else {
            for (int i = 0; i < num_samp * num_channels; i++)
                convbuf[i] = 0.195 * (float)(recvbuf[i] - 32768);
        }

        sourceBuffers[0]->addToBuffer(convbuf,
            &timestamps.getReference(0),
            &ttlEventWords.getReference(0),
            num_samp,
            1);

        total_samples += num_samp;
    }
        break;
    
    case LSL: {
        std::vector<std::vector<float>> recv_buf;
        std::vector<double> ts_buf;
        inlet->pullData(&recv_buf, &ts_buf);
        transpose = true;

        // Transpose because the chunkSize argument in addToBuffer does not seem to do anything
        if (transpose) { // think we always go here for LSL? 
            //std::cout << "consize: " << convbuf.size() << std::endl;
            int k = 0;
            for (int i = 0; i < num_samp; i++) {
                for (int j = 0; j < num_channels; j++) {
                    float curval = 0.195 * (float)(recv_buf[i][j]);
                    convbuf[k++] = curval; 
                    //std::cout << curval << std::endl;
                }
            }
        }
        else {
            for (int i = 0; i < num_samp * num_channels; i++) {
                //convbuf[i] = 0.195 * (float)(recvbuf[i] - 32768);
                convbuf[i] = (float)(recvbuf[i]);
                
            }
        }
        for (int i = 0; i < num_samp; i++) {
            timestamps.set(i, ts_buf[i]);
            std::cout << ts_buf[i] << std::endl;
        }

        //timestamps.set(0, ts_buf[0]);

        int sampswrit = sourceBuffers[0]->addToBuffer(convbuf,
            timestamps.getRawDataPointer(),
            ttlEventWords.getRawDataPointer(),
            num_samp,
            1);

        //std::cout << "sampswrite: " << sampswrit << std::endl;
        total_samples += num_samp; // if needed
    }
        break;
       
    }
    return true;
}

/*
* function used only for debugging
*/
void EphysSocket::timerCallback()
{
    
    //std::cout << "Expected samples: " << int(sample_rate * 5) << ", Actual samples: " << total_samples << std::endl;
    
    //relative_sample_rate = (sample_rate * 5) / float(total_samples);

    //total_samples = 0;
}

bool EphysSocket::usesCustomNames()
{
    return false;
}