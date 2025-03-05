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

#ifndef HRI_PHYSIO_STREAM_LSL_STREAMER_H
#define HRI_PHYSIO_STREAM_LSL_STREAMER_H

#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "HriPhysio/Stream/lsl_cpp.h"

#include "HriPhysio/Stream/streamerInterface.h"

#include "HriPhysio/helpers.h"

namespace hriPhysio {
    namespace Stream {
        class LslStreamer;
    }
}

class hriPhysio::Stream::LslStreamer : public hriPhysio::Stream::StreamerInterface {

private:
    std::unique_ptr<lsl::stream_inlet> inlet;
    std::unique_ptr<lsl::stream_outlet> outlet;

public:
    LslStreamer();

    // ~LslStreamer();

    // Since LslStreamer is derived from StreamerInterface,
    // it should have a virtual destructor to ensure proper cleanup of derived class objects when deleted via a base class pointer.
    // (According to C++ guidelines C.35)
    virtual ~LslStreamer();

    // Add default implementations for special member functions to follow the Rule of Five.
    // (According to C++ guidelines C.21)
    LslStreamer(const LslStreamer&) = delete;
    LslStreamer& operator=(const LslStreamer&) = delete;
    LslStreamer(LslStreamer&&) = default;
    LslStreamer& operator=(LslStreamer&&) = default;

    lsl::channel_format_t getLslFormatType();

    // Explicitly mark overridden functions with override.
    // bool openInputStream();
    // bool openOutputStream();
    bool openInputStream() override;
    bool openOutputStream() override;

    // General data streams.
    // void publish(const std::vector<hriPhysio::varType>&  buff, const std::vector<double>* timestamps = nullptr);
    // void receive(std::vector<hriPhysio::varType>& buff, std::vector<double>* timestamps = nullptr);
    void publish(const std::vector<hriPhysio::varType>& buff, const std::vector<double>* timestamps = nullptr) override;
    void receive(std::vector<hriPhysio::varType>& buff, std::vector<double>* timestamps = nullptr) override;
    
    // Special string stream.
    // void publish(const std::string&  buff, const double* timestamps = nullptr);
    // void receive(std::string& buff, double* timestamps = nullptr);
    void publish(const std::string& buff, const double* timestamps = nullptr) override;
    void receive(std::string& buff, double* timestamps = nullptr) override;

private:
    template<typename T>
    void pushStream(const std::vector<hriPhysio::varType>&  buff, const std::vector<double>* timestamps);

    template<typename T>
    void pullStream(std::vector<hriPhysio::varType>& buff, std::vector<double>* timestamps);
    
};

#endif /* HRI_PHYSIO_STREAM_LSL_STREAMER_H */
