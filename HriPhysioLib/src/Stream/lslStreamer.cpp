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

#include "HriPhysio/Stream/lslStreamer.h"

using namespace hriPhysio::Stream;


LslStreamer::LslStreamer() : 
    StreamerInterface() {

}


LslStreamer::~LslStreamer() {

    if (this->mode == modeTag::RECEIVER) {
        inlet->close_stream();
        inlet.reset();
    } else if (this->mode == modeTag::SENDER) {
        outlet.reset();
    }
}


lsl::channel_format_t LslStreamer::getLslFormatType() {

    lsl::channel_format_t cf_type;

    switch (this->var) {
    case hriPhysio::varTag::CHAR:
        cf_type = lsl::channel_format_t::cf_int8;
        break;
    case hriPhysio::varTag::INT16:
        cf_type = lsl::channel_format_t::cf_int16;
        break;
    case hriPhysio::varTag::INT32:
        cf_type = lsl::channel_format_t::cf_int32;
        break;
    case hriPhysio::varTag::INT64:
        cf_type = lsl::channel_format_t::cf_int64;
        break;
    case hriPhysio::varTag::FLOAT:
        cf_type = lsl::channel_format_t::cf_float32;
        break;
    case hriPhysio::varTag::DOUBLE:
        cf_type = lsl::channel_format_t::cf_double64;
        break;
    case hriPhysio::varTag::STRING:
        cf_type = lsl::channel_format_t::cf_string;
        break;
    default:
        break;
    }

    return cf_type;
}


bool LslStreamer::openInputStream() {

    //-- Set the current mode.
    if (this->mode != modeTag::NOTSET) {
        return false;
    }

    this->mode = modeTag::RECEIVER;

    try {

        //-- Create a new inlet from the given input name.
        inlet.reset(new lsl::stream_inlet(lsl::resolve_stream("name", this->name)[0]));

	} catch (std::exception& e) { std::cerr << "Got an exception: " << e.what() << std::endl; return false; }


    return true;
}


bool LslStreamer::openOutputStream() {
    
    //-- Set the current mode.
    if (this->mode != modeTag::NOTSET) {
        return false;
    }

    this->mode = modeTag::SENDER;

    try {

        //-- Create an info obj and open an outlet with it.
        lsl::stream_info info(
            /* name           = */ this->name,
            /* type           = */ "",
            /* channel_count  = */ this->num_channels,
            /* nominal_srate  = */ this->sampling_rate, // lsl::IRREGULAR_RATE -> 0.0.
            /* channel_format = */ this->getLslFormatType(),
            /* source_id      = */ this->name
        );

        outlet.reset(new lsl::stream_outlet(info, /*chunk_size=*/this->frame_length, /*max_buffered=*/this->frame_length*2));

    } catch (std::exception& e) { std::cerr << "Got an exception: " << e.what() << std::endl; return false; }


    return true;
}


void LslStreamer::publish(const std::vector<hriPhysio::varType>&  buff, const std::vector<double>* timestamps/*=nullptr*/) {

    std::cerr << "[LSL-OUT] Sending: " << this->dtype << " ";
    switch (this->var) {
    case hriPhysio::varTag::CHAR:
        this->pushStream<char>(buff, timestamps);
        break;
    case hriPhysio::varTag::INT16:
        this->pushStream<int16_t>(buff, timestamps);
        break;
    case hriPhysio::varTag::INT32:
        this->pushStream<int32_t>(buff, timestamps);
        break;
    case hriPhysio::varTag::INT64:
        this->pushStream<int64_t>(buff, timestamps);
        break;
    case hriPhysio::varTag::FLOAT:
        this->pushStream<float>(buff, timestamps);
        break;
    case hriPhysio::varTag::DOUBLE:
        std::cerr << "<double>" << std::endl;
        this->pushStream<double>(buff, timestamps);
        break;
    default:
        break;
    }
}


void LslStreamer::publish(const std::string& buff, const double* timestamps/*=nullptr*/) {

    //-- Push out the string.
    outlet->push_sample(&buff);

    return;
}


void LslStreamer::receive(std::vector<hriPhysio::varType>& buff, std::vector<double>* timestamps/*=nullptr*/) {

    std::cerr << "[LSL-IN] Receive: " << this->dtype << " ";

    switch (this->var) {
    case hriPhysio::varTag::CHAR:
        this->pullStream<char>(buff, timestamps);
        break;
    case hriPhysio::varTag::INT16:
        this->pullStream<int16_t>(buff, timestamps);
        break;
    case hriPhysio::varTag::INT32:
        this->pullStream<int32_t>(buff, timestamps);
        break;
    case hriPhysio::varTag::INT64:
        this->pullStream<int64_t>(buff, timestamps);
        break;
    case hriPhysio::varTag::FLOAT:
        this->pullStream<float>(buff, timestamps);
        break;
    case hriPhysio::varTag::DOUBLE:
        std::cerr << "<double>" << std::endl;
        this->pullStream<double>(buff, timestamps);
        break;
    default:
        break;
    }
}


void LslStreamer::receive(std::string& buff, double* timestamps/*=nullptr*/) {

    //-- Pull in the string.
    double ts = inlet->pull_sample(&buff, 1, 0.2);

    if (timestamps != nullptr) { *timestamps = ts; }

    return;
}


template<typename T>
void LslStreamer::pushStream(const std::vector<hriPhysio::varType>&  buff, const std::vector<double>* timestamps) {

    std::vector<T> samples(buff.size());

    //-- Copy the data into a temporary transfer.
    for (std::size_t idx = 0; idx < buff.size(); ++idx) {
        samples[idx] = std::get<T>( buff[idx] );
    }

    //-- Push a multiplexed chunk from a flat vector.
    outlet->push_chunk_multiplexed(samples);

    return;
}


template<typename T>
void LslStreamer::pullStream(std::vector<hriPhysio::varType>& buff, std::vector<double>* timestamps) {

    std::vector<T> samples;

    //-- Pull a multiplexed chunk into a flat vector.
    inlet->pull_chunk_multiplexed(samples, timestamps, 5.0);

    //-- Copy the data into the buffer.
    for (std::size_t idx = 0; idx < samples.size(); ++idx) {
        buff[idx] = samples[idx];
    }

    return;
}
