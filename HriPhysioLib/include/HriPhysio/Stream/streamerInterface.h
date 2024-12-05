/* ================================================================================
 * Copyright: (C) 2021, SIRRL Social and Intelligent Robotics Research Laboratory, 
 *     University of Waterloo, All rights reserved.
 * 
 * Authors: 
 *     Austin Kothig <austin.kothig@uwaterloo.ca>
 * 
 * CopyPolicy: Released under the terms of the BSD 3-Clause License. 
 *     See the accompanying LICENSE file for details.
 * ================================================================================
 */

#ifndef HRI_PHYSIO_STREAM_STREAMER_INTERFACE_H
#define HRI_PHYSIO_STREAM_STREAMER_INTERFACE_H

#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "HriPhysio/helpers.h"

namespace hriPhysio {
    namespace Stream {
        class StreamerInterface;
    }
}

class hriPhysio::Stream::StreamerInterface {

protected:
    std::string name;
    std::string dtype;
    std::size_t frame_length;
    std::size_t num_channels;
    std::size_t sampling_rate;

    hriPhysio::varTag var;

    enum modeTag { NOTSET, SENDER, RECEIVER } mode;
    

public:
    StreamerInterface();

    ~StreamerInterface();

    void setName(const std::string name);
    void setDataType(const std::string dtype);
    void setFrameLength(const std::size_t frame_length);
    void setNumChannels(const std::size_t num_channels);
    void setSamplingRate(const std::size_t sampling_rate);

    std::string getName() const;
    std::string getDataType() const;
    std::size_t getFrameLength() const;
    std::size_t getNumChannels() const;
    std::size_t getSamplingRate() const;

    hriPhysio::varTag getVariableTag() const;

    virtual bool openInputStream() = 0;
    virtual bool openOutputStream() = 0;

    // General data streams.
    virtual void publish(const std::vector<hriPhysio::varType>&  buff, const std::vector<double>* timestamps = nullptr) = 0;
    virtual void receive(std::vector<hriPhysio::varType>& buff, std::vector<double>* timestamps = nullptr) = 0;
    
    // Special string stream.
    virtual void publish(const std::string&  buff, const double* timestamps = nullptr) = 0;
    virtual void receive(std::string& buff, double* timestamps = nullptr) = 0;

private:
    void tempfunc();

};

#endif /* HRI_PHYSIO_STREAM_STREAMER_INTERFACE_H */
