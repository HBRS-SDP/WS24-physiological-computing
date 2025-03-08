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
 * Copyright: (C) 2024, Yash Somaiya, 
 *       Hochschule Bonn-Rhein-Sieg (H-BRS), All rights reserved.
 * 
 * Authors: Yash Somaiya yash.somaiya@smail.inf.h-brs.de
 * 
 * CopyPolicy: Released under the terms of the MIT License.
 *      See the accompanying LICENSE file for details.
 * ================================================================================
 */

#include <HriPhysio/Manager/robotManager.h>

using namespace hriPhysio::Manager;


RobotManager::RobotManager(hriPhysio::Social::RobotInterface* robot) : 
    robot(robot) {
    
}


RobotManager::~RobotManager() {
    delete robot;
}


void RobotManager::configure(int argc, char **argv) {
    
    //-- Configure the robot.
    robot->configure(argc, argv);


    //-- Init an argument parser.
    hriPhysio::ArgParser args(argc, argv);


    //-- Load the yaml file.
    const std::string &yaml_file = args.getCmdOption("--conf");
    YAML::Node config = YAML::LoadFile(yaml_file);

    //-- Parameters about the streamer.
    //const std::string input_name  = config["input"].as<std::string>();
    //const std::string output_name = config["output"].as<std::string>();

    //-- Parameters about the data.
    //dtype          = config[ "dtype"          ].as<std::string>( /*default=*/ "int32");
    //sampling_rate  = config[ "sampling_rate"  ].as<std::size_t>( /*default=*/ 20  );
    //input_frame    = config[ "input_frame"    ].as<std::size_t>( /*default=*/ 10  );
    //output_frame   = config[ "output_frame"   ].as<std::size_t>( /*default=*/ 20  );
    //num_channels   = config[ "num_channels"   ].as<std::size_t>( /*default=*/ 1   );
    //sample_overlap = config[ "sample_overlap" ].as<std::size_t>( /*default=*/ 0   );
    //buffer_length  = config[ "buffer_length"  ].as<std::size_t>( /*default=*/ 100 );


    //-- Enable logging?
    log_data = config["log_data"].as<bool>( /*default=*/ false );
    log_name = config["log_name"].as<std::string>( /*default=*/ "");

    std::cerr << "[CONF] Load complete.\n";
    
    //-- If streams are empty, exit.
    if (log_data && log_name == "") {
        this->close();
        return;
    }


    if (log_data) {

        robot_logger.setName(log_name);
        robot_logger.setDataType("STRING");
        robot_logger.setNumChannels(1);
    
        if (!robot_logger.openOutputStream()) {
            std::cerr << "Could not open logger stream.\n";
            this->close();
            return;
        }
    }


    //-- Initialize the threads.
    this->threadInit();

    start_time = std::chrono::system_clock::now();

    return;
}


void RobotManager::interactive() {
    
    std::string inp;
    while (this->getManagerRunning()) {
        
        //-- Get some input.
        std::cout << ">>> ";
        getline(std::cin, inp);

        //-- Process the request.
        this->process(inp);
    }
}


bool RobotManager::threadInit() {

    //-- Initialize threads but don't start them yet.
    addLoopThread(std::bind(&RobotManager::selfLoop, this),  /*period=*/ 0.01, /*start=*/ false);
    addLoopThread(std::bind(&RobotManager::inputLoop, this), /*period=*/ 0.01, /*start=*/ false);
    

    return true;
}


void RobotManager::process(const std::string& inp) {

    if (this->log_data) {
        //-- Log the data received.
        std::chrono::duration<double> now = std::chrono::system_clock::now() - start_time;
        double t = now.count();
        this->robot_logger.publish(inp, &t);
    }

    //-- Parse it up.
    std::vector< std::string > input = hriPhysio::parseString(inp);
    
    //-- If line was empty, skip to next input.
    if (input.size() == 0) { return; }
    
    //-- Get the command.
    std::string cmd = input[0];
    hriPhysio::toLower(cmd);
    if (cmd == "exit") {

        //-- Close the audio and video.
        robot->addAudioFile("exit");
        robot->addVideoFile("exit");

        //-- Close the threads.
        this->close();

    } else if (cmd == "set") {

        if(!this->setFunctions(input)) {
            std::cerr << "[ERROR] "
                        << "Unrecognized set input ``"
                        << hriPhysio::combineString(input, 1)
                        << "``" << std::endl;
        }

    } else if (cmd == "get") {

        if(!this->getFunctions(input)) {
            std::cerr << "[ERROR] "
                        << "Unrecognized get input ``"
                        << hriPhysio::combineString(input, 1)
                        << "``" << std::endl;
        }

    } else if (cmd == "help") {
        
    } else {
        std::cerr << "[ERROR] "
                    << "Unrecognized command ``"
                    << cmd << "``" << std::endl;
    }

    return;
}


bool RobotManager::setFunctions(const std::vector< std::string >& input) {

    if (input.size() < 2) { return false; }

    std::string func = input[1];
    hriPhysio::toLower(func);

    if (func == "state") {

        // set state {head,rightarm,leftarm,rightleg,leftleg} 0.0 0.0 [0.0]

        if (input.size() < 4) { return false; }

        std::string perph = input[2];
        hriPhysio::toLower(perph);

        std::vector< double > pos = hriPhysio::toVecDouble(input, 3);

        if (perph == "head") { 
            return robot->setPerphState(hriPhysio::Social::RobotInterface::peripheral::HEAD, pos);
        } else if (perph == "rightarm") {
            return robot->setPerphState(hriPhysio::Social::RobotInterface::peripheral::RIGHTARM, pos);
        } else if (perph == "leftarm") {
            return robot->setPerphState(hriPhysio::Social::RobotInterface::peripheral::LEFTARM, pos);
        } else if (perph == "rightleg") {
            return robot->setPerphState(hriPhysio::Social::RobotInterface::peripheral::RIGHTLEG, pos);
        } else if (perph == "leftleg") {
            return robot->setPerphState(hriPhysio::Social::RobotInterface::peripheral::LEFTLEG, pos);
        }
        
    } else if (func == "velocity") {

        //setPerphVelocity(const peripheral perph, const std::vector<double>& speed);

    } else if (func == "gesture") {

        // set gesture jump [1.5]

        if (input.size() == 3) {
            return robot->addGesture(input[2]);
        } else if (input.size() == 4) {
            return robot->addGesture(input[2], std::stod(input[3]));
        } else {
            return false;
        }

    } else if (func == "emotion") {
        
        // set emotion happy

        if (input.size() != 3) { return false; }
        
        return robot->setEmotionState(input[2]);


    } else if (func == "speech") {

        // set speech I am showing an example.

        if (input.size() < 3) { return false; }

        //-- Combine the string and send it out.
        std::string phrase = hriPhysio::combineString(input, 2);
        return robot->addSpeech(phrase);

    } else if (func == "speechConfig") {

        // set speechConfig 140 80

        if (input.size() < 3) { return false; }

        //-- Combine the string and send it out.
        std::string config = hriPhysio::combineString(input, 2);
        return robot->setSpeechConfig(config);

    } else if (func == "volume") {

        // set volume 42
        
        if (input.size() != 3) { return false; }

        return robot->setVolume(std::stod(input[2]));

    } else if (func == "audio") {

        // set audio temp.wav [1]
        
        if (input.size() == 3) {
            return robot->addAudioFile(input[2]);
        } else if (input.size() == 4) {
            return robot->addAudioFile(input[2], std::stoi(input[3]));
        } else {
            return false;
        }
    
    } else if (func == "video") {

        // set video temp.mp4
        
        if (input.size() != 3) { return false; }

        return robot->addVideoFile(input[2]);

    } else {
        return false;
    }

    return true;
}


bool RobotManager::getFunctions(const std::vector< std::string >& input) {

    std::string func = input[1];
    hriPhysio::toLower(func);

    if (func == "state") {
        //getPerphState(const peripheral perph, std::vector<double>& pos);
    
    } else if (func == "velocity") {
        //getPerphVelocity(const peripheral perph, std::vector<double>& speed);

    } else if (func == "emotion") {
        //getEmotionState(std::string& emotion);

    } else {
        return false;
    }

    return true;
}


void RobotManager::selfLoop() {

    //-- Call the robot's internal self loop.
    robot->robotLoop();

    return;
}

void RobotManager::inputLoop() {

    //-- Check to see if there is a command from the robot.
    std::string command;
    bool ret = robot->getRobotCommand(command);

    //-- If a command is given, process it.
    if (ret) { this->process(command); }

    return;
}
