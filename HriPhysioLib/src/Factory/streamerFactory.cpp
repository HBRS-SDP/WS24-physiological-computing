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

/* ================================================================================
 * Copyright: (C) 2024, Ayush Salunke, 
 *       Hochschule Bonn-Rhein-Sieg (H-BRS), All rights reserved.
 * 
 * Authors: Ayush Salunke ayush.salunke@smail.inf.h-brs.de
 * 
 * CopyPolicy: Released under the terms of the MIT License.
 *      See the accompanying LICENSE file for details.
 * ================================================================================
 */

#include <HriPhysio/Factory/streamerFactory.h>

using namespace hriPhysio::Factory;


StreamerFactory::StreamerFactory() {

}

StreamerFactory::~StreamerFactory() {

}

hriPhysio::Stream::StreamerInterface* StreamerFactory::getStreamer(std::string streamerType) {

    hriPhysio::toUpper(streamerType);
    if (streamerType == "") {
        std::cerr << "[WARNING] "
                  << "Stream Factory received no type!!" << std::endl;
        return nullptr;
    }

    // 'new' was allocating memory but passing the pointer to a non-owner type. Used 'std::make_unique<T>(...)' instead of 'new'.
    // (According to C++ guidelines I.11) 
    // Also we are using .release() to Return a Raw Pointer because we have to keep the raw pointer and still release the ownership from std::unique_ptr
    if (streamerType == "LSL") {
        // return new hriPhysio::Stream::LslStreamer();
        return std::make_unique<hriPhysio::Stream::LslStreamer>().release();
    }

    if (streamerType == "ROS") {
        #ifdef WITH_ROS
        // return new hriPhysio::Stream::RosStreamer();
        return std::make_unique<hriPhysio::Stream::RosStreamer>().release();
        #endif

        std::cerr << "[WARNING] " 
                  << "Stream Factory type ``" << streamerType
                  << "`` is not compiled!!" << std::endl;
        return nullptr;
    }

    if (streamerType == "YARP") {
        return nullptr;
    } 
    
    std::cerr << "[WARNING] "
              << "Stream Factory type ``" << streamerType
              << "`` is not defined!!" << std::endl;
    
    return nullptr;
}
