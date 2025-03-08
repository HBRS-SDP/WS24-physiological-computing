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

#include <HriPhysio/Stream/csvStreamer.h>

using namespace hriPhysio::Stream;


CsvStreamer::CsvStreamer() : 
    StreamerInterface() {

}

//Set Precision Constant (According to C++ guidelines)
constexpr int precisionConstant = 10; 


CsvStreamer::~CsvStreamer() {

    if (this->mode == modeTag::RECEIVER) {
        input.close();
    } else if (this->mode == modeTag::SENDER) {
        input.close();
        output.open(this->name);
    }
}

bool CsvStreamer::openInputStream() {
    
    //-- Set the current mode.
    if (this->mode != modeTag::NOTSET) {
        return false;
    }

    this->mode = modeTag::RECEIVER;

    try {

        //-- Open the specified file for reading from.
        input.open(this->name);

	} catch (std::exception& e) { std::cerr << "Got an exception: " << e.what() << std::endl; return false; }


    return true;
}


bool CsvStreamer::openOutputStream() {
    
    //-- Set the current mode.
    if (this->mode != modeTag::NOTSET) {
        return false;
    }

    this->mode = modeTag::SENDER;

    try {
        std::cerr << "Opening output file: " << this->name << std::endl;
        //-- Open the specified file for writing to.
        output.open(this->name);
        if (!output.is_open()) {
            std::cerr << "Failed to open output file: " << this->name << std::endl;
            return false;
        }

    } catch (std::exception& e) { std::cerr << "Got an exception: " << e.what() << std::endl; return false; }

    //-- Write the header information.
    output << "System Time" << "," << "Internal Time";

    for (std::size_t ch = 0; ch < this->num_channels; ++ch) {
        output << "," << "ch-" << ch;
    } 
    
    output << std::endl;


    return true;
}


void CsvStreamer::publish(const std::vector<hriPhysio::varType>&  buff, const std::vector<double>* timestamps/*=nullptr*/) {

    // Manual Changes
    std::cerr << "Publishing data..." << std::endl;

    if (!output.is_open()) {
        std::cerr << "Output stream is not open!" << std::endl;
        return;
    }
    // ---------------

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
        this->pushStream<double>(buff, timestamps);
        break;
    default:
        std::cerr << "Unsupported data type in publish()" << std::endl;
        break;
    }
}

void CsvStreamer::publish(const std::string& buff, const double* timestamps/*=nullptr*/) {

    //-- "System Time" 
    std::time_t t = std::time(nullptr);
    output << std::put_time(std::localtime(&t), "%Y/%m/%d_%H:%M:%S") << ",";

    //-- "Internal Time"
    if (timestamps == nullptr) {
        output << 0.0;
    } else {
        output << std::setprecision(precisionConstant) << (*timestamps);
    } 

    //-- Data.
    output << "," << "\"" << buff << "\"";

    //-- Move to the next line.
    output << std::endl;

    return;
}


void CsvStreamer::receive(std::vector<hriPhysio::varType>& buff, std::vector<double>* timestamps/*=nullptr*/) {

    std::cerr << "[CSV-IN] READING FROM: " << this->dtype << " ";

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
        this->pullStream<double>(buff, timestamps);
        break;
    default:
        break;
    }
}


void CsvStreamer::receive(std::string& buff, double* timestamps/*=nullptr*/) {
    
    return;
}


template<typename T>
void CsvStreamer::pushStream(const std::vector<hriPhysio::varType>&  buff, const std::vector<double>* timestamps) {

    //TODO: Should put in some error catching here for buff and timestamp.
    //      buff should logically be the length of timestamps by the num channels.

    std::time_t t = std::time(nullptr);

    std::size_t idx_buff = 0;
    std::size_t idx_time = 0;

    //std::cerr << "buff len: " << buff.size() << "  ts len: " << timestamps->size();
    //if (timestamps->size() == 0) { std::cerr << "!!!!!!!!!!!!!!!!!!!!!!!"; }
    //std::cerr << std::endl;

    while (idx_buff < buff.size()) {
        
        //-- "System Time" 
        output << std::put_time(std::localtime(&t), "%Y/%m/%d_%H:%M:%S") << ",";

        //-- "Internal Time"
        if (timestamps == nullptr) {
            output << 0.0;
        } else if (timestamps->size() == 0) {
            output << 0.0;
        } else {
            output << std::setprecision(precisionConstant) << timestamps->at(idx_time);
            ++idx_time;
        } 

        //-- "Channels"
        for (std::size_t ch = 0; ch < this->num_channels; ++ch) {
            output << "," << std::get<T>( buff[idx_buff + ch] );
        }
        idx_buff += this->num_channels;

        //-- Move to the next line.
        output << std::endl;
    }

    return;
}

// Changing the pullStream() because it is not working properly (Used ChatGPT for this!)

// template<typename T>
// void CsvStreamer::pullStream(std::vector<hriPhysio::varType>& buff, std::vector<double>* timestamps) {

//     // std::vector<T> samples;
//     std::string line;
//     bool skipHeader = true;

//     while (std::getline(input, line)) {
//         if (skipHeader) {
//             skipHeader = false; // Skip the first line (header)
//             continue;
//         }
//         std::cerr << "Processing line: " << line << std::endl;
//         // //-- Pull a multiplexed chunk into a flat vector.
//         // inlet->pull_chunk_multiplexed(samples, timestamps, 1.0);
        
//         // //-- Copy the data into the buffer.
//         // for (std::size_t idx = 0; idx < samples.size(); ++idx) {
//         //    buff[idx] = samples[idx];
//         // }
//     }
//     return;
// }

template<typename T>
void CsvStreamer::pullStream(std::vector<hriPhysio::varType>& buff, std::vector<double>* timestamps) {
    std::string line;
    bool skipHeader = true;

    while (std::getline(input, line)) {
        if (skipHeader) {
            skipHeader = false; // Skip the first line (header)
            continue;
        }

        std::cerr << "Processing line: " << line << std::endl;

        // Split the line by comma
        std::istringstream ss(line);
        std::string token;
        std::vector<std::string> tokens;

        while (std::getline(ss, token, ',')) {
            tokens.push_back(token);
        }

        // Assuming the format: System Time, Internal Time, ch-0, ch-1, ..., ch-n
        if (tokens.size() < (2 + this->num_channels)) {
            std::cerr << "Malformed line: " << line << std::endl;
            continue;
        }

        // Parse "Internal Time"
        if (timestamps != nullptr) {
            timestamps->push_back(std::stod(tokens[1])); // Convert to double
        }

        // Parse channel data
        for (std::size_t i = 2; i < tokens.size(); ++i) {
            T value = static_cast<T>(std::stod(tokens[i])); // Convert to appropriate type
            buff.push_back(value);
        }
    }
}
