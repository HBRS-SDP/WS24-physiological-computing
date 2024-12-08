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

#include <HriPhysio/Manager/physioManager.h>

using namespace hriPhysio::Manager;


PhysioManager::PhysioManager(hriPhysio::Stream::StreamerInterface* input, hriPhysio::Stream::StreamerInterface* output) : 
    stream_input(input),
    stream_output(output) {
    
}


PhysioManager::~PhysioManager() {
    delete stream_input;
    delete stream_output;
}


void PhysioManager::configure(const std::string yaml_file) {

    //-- Load the yaml file.
    YAML::Node config = YAML::LoadFile(yaml_file);

    //-- Parameters about the streamer.
    const std::string input_name  = config["input"].as<std::string>();
    const std::string output_name = config["output"].as<std::string>();

    //-- Parameters about the data.
    dtype          = config[ "dtype"          ].as<std::string>( /*default=*/ "int32");
    sampling_rate  = config[ "sampling_rate"  ].as<std::size_t>( /*default=*/ 20  );
    input_frame    = config[ "input_frame"    ].as<std::size_t>( /*default=*/ 10  );
    output_frame   = config[ "output_frame"   ].as<std::size_t>( /*default=*/ 20  );
    num_channels   = config[ "num_channels"   ].as<std::size_t>( /*default=*/ 1   );
    sample_overlap = config[ "sample_overlap" ].as<std::size_t>( /*default=*/ 0   );
    buffer_length  = config[ "buffer_length"  ].as<std::size_t>( /*default=*/ 100 );

    //-- Enable logging?
    log_data = config["log_data"].as<bool>( /*default=*/ false );
    log_name = config["log_name"].as<std::string>( /*default=*/ "");

    std::cerr << "[CONF] Load complete.\n";
    

    //-- Configure the intermediary buffer.
    buffer.resize(buffer_length * num_channels);
    timestamps.resize(buffer_length);


    //-- If streams are empty, exit.
    if (input_name == "" || output_name == "") {
        this->close();
        return;
    }


    //-- Configure the streams.
    stream_input->setName(input_name);
    stream_input->setDataType(dtype);
    stream_input->setFrameLength(input_frame);
    stream_input->setNumChannels(num_channels);
    stream_input->setSamplingRate(sampling_rate);

    stream_output->setName(output_name);
    stream_output->setDataType(dtype);
    stream_output->setFrameLength(output_frame);
    stream_output->setNumChannels(num_channels);
    stream_output->setSamplingRate(sampling_rate);

    if (log_data) {
        stream_logger.setName(log_name);
        stream_logger.setDataType(dtype);
        stream_logger.setFrameLength(input_frame);
        stream_logger.setNumChannels(num_channels);
        stream_logger.setSamplingRate(sampling_rate);
    }


    //-- Try opening the streams.
    if (!stream_input->openInputStream()) {
        std::cerr << "Could not open input stream.\n";
        this->close();
        return;
    }

    if (!stream_output->openOutputStream()) {
        std::cerr << "Could not open output stream.\n";
        this->close();
        return;
    }

    if (log_data && !stream_logger.openOutputStream()) {
        std::cerr << "Could not open logger stream.\n";
        this->close();
        return;
    }


    //-- Initialize the threads.
    this->threadInit();

    return;
}


void PhysioManager::interactive() {
    std::string str;
    while (this->getManagerRunning()) {
        std::cin >> str;
        std::cout << ">>> " << str << std::endl;

        if (str == "exit") {

            //-- Close the threads.
            this->close();
            
            break;
        }
    }
}


bool PhysioManager::threadInit() {

    //-- Initialize threads but don't start them yet.
    addThread(std::bind(&PhysioManager::inputLoop, this),  /*start=*/ false);
    addThread(std::bind(&PhysioManager::outputLoop, this), /*start=*/ false);

    return true;
}


void PhysioManager::inputLoop() {

    //-- Get the id for the current thread.
    const std::thread::id thread_id = std::this_thread::get_id();

    //-- Construct a vector for moving data between stream and the buffer.
    std::vector<hriPhysio::varType> transfer(input_frame * num_channels);
    std::vector<double>* stamps = new std::vector<double>(input_frame);

    //-- Loop until the manager stops running.
    while (this->getManagerRunning()) {
        
        //-- If this thread is active, run.
        if (this->getThreadStatus(thread_id)) {

            //-- Get data from the stream.
            stream_input->receive(transfer, stamps);

            if (transfer.size()) {
                
                //std::cerr << "[INPUT] ";
                //for (std::size_t idx = 0; idx < transfer.size(); idx++) {
                //    std::cerr << " [" << (*stamps)[idx] << "]: "; std::visit(hriPhysio::printVisitor(), transfer[idx]);
                //} std::cerr << std::endl;

                //-- Add the data to the buffer.
                this->buffer.enqueue(transfer.data(), transfer.size());
                this->timestamps.enqueue(stamps->data(), stamps->size());


                if (this->log_data) {
                    stream_logger.publish(transfer, stamps);
                }
            }

        } else {
            std::this_thread::sleep_for(
                std::chrono::duration<double>( 0.1 ) //seconds.
            );
        }
    }
}


void PhysioManager::outputLoop() {

    //-- Get the id for the current thread.
    const std::thread::id thread_id = std::this_thread::get_id();

    //-- Set a Const which is used a few times here.
    const std::size_t frame_length = output_frame * num_channels;

    //-- Construct a vector for moving data between the buffer and stream.
    std::vector<hriPhysio::varType> transfer(frame_length);

    //-- Loop until the manager stops running.
    while (this->getManagerRunning()) {
        
        //std::cerr << "[OUTPUT] " << buffer.size() << std::endl;

        //-- If this thread is active, run.
        if (this->getThreadStatus(thread_id) && buffer.size() >= frame_length) {

            //-- Get data from the buffer.
            buffer.dequeue(transfer.data(), frame_length, sample_overlap);

            //-- Write it out with the streamer.
            stream_output->publish(transfer);

            //std::cerr << "[OUTPUT] Sent output.\n";

        } else {
            std::this_thread::sleep_for(
                std::chrono::duration<double>( 0.01 ) //seconds.
            );
        }
    }
}
