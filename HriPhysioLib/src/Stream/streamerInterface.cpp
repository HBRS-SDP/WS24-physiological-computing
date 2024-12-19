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

#include "HriPhysio/Stream/streamerInterface.h"

using namespace hriPhysio::Stream;


StreamerInterface::StreamerInterface() :
    name(""),
    dtype(""),
    frame_length(0),
    num_channels(0),
    mode(modeTag::NOTSET) {

}


StreamerInterface::~StreamerInterface() {
}


void StreamerInterface::setName(const std::string name) {
    this->name = name;
    return;
}


void StreamerInterface::setDataType(const std::string dtype) {
    
    std::string upperString(dtype);
    hriPhysio::toUpper(upperString);

    if (upperString == "") {
        return;
    } else if (upperString == "INT8" || upperString == "CHAR") {
        this->var = hriPhysio::varTag::CHAR;
    } else if (upperString == "INT16") {
        this->var = hriPhysio::varTag::INT16;
    } else if (upperString == "INT32") {
        this->var = hriPhysio::varTag::INT32;
    } else if (upperString == "INT64") {
        this->var = hriPhysio::varTag::INT64;
    } else if (upperString == "FLOAT"  || upperString == "FLOAT32") {
        this->var = hriPhysio::varTag::FLOAT;
    } else if (upperString == "DOUBLE" || upperString == "DOUBLE64" ) {
        this->var = hriPhysio::varTag::DOUBLE;
    } else if (upperString == "STRING") {
        this->var = hriPhysio::varTag::STRING;
    } else {
        //-- Does not match. TODO: Throw error?
        return;
    }
    
    //-- A tag matched. Set type.
    this->dtype = upperString;

    return;
}


void StreamerInterface::setFrameLength(const std::size_t frame_length) {
    this->frame_length = frame_length;
    return;
}


void StreamerInterface::setNumChannels(const std::size_t num_channels) {
    this->num_channels = num_channels;
    return;
}


void StreamerInterface::setSamplingRate(const std::size_t sampling_rate) {
    this->sampling_rate = sampling_rate;
    return;
}


std::string StreamerInterface::getName() const {
    return this->name;
}


std::string StreamerInterface::getDataType() const {
    return this->dtype;
}


std::size_t StreamerInterface::getFrameLength() const {
    return this->frame_length;
}


std::size_t StreamerInterface::getNumChannels() const {
    return this->num_channels;
}


std::size_t StreamerInterface::getSamplingRate() const {
    return this->sampling_rate;
}


hriPhysio::varTag StreamerInterface::getVariableTag() const {
    return this->var;
}
