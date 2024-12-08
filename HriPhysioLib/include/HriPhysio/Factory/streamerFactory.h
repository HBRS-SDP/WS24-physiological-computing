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

#ifndef HRI_PHYSIO_FACTORY_STREAMER_FACTORY_H
#define HRI_PHYSIO_FACTORY_STREAMER_FACTORY_H

#include <string>

#include "HriPhysio/Stream/streamerInterface.h"
#include "HriPhysio/Stream/lslStreamer.h"

#ifdef WITH_ROS
#include <HriPhysio/Stream/ros/rosStreamer.h>
#endif

#include <HriPhysio/helpers.h>

namespace hriPhysio {
    namespace Factory {
        class StreamerFactory;
    }
}

class hriPhysio::Factory::StreamerFactory {
private:
    
public:
    StreamerFactory();
    ~StreamerFactory();

    hriPhysio::Stream::StreamerInterface* getStreamer(std::string streamerType);
};

#endif /* HRI_PHYSIO_FACTORY_STREAMER_FACTORY_H */
