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
 * Copyright: (C) 2024, Prachi Sheth,
 *     Hochschule Bonn-Rhein-Sieg (H-BRS), All rights reserved.
 * 
 * Author: 
 *     Prachi Sheth <prachi.sheth@smail.inf.h-brs.de>
 * 
 * CopyPolicy: Released under the terms of the MIT License.
 *     See the accompanying LICENSE file for details.
 * ================================================================================
 */

#include <HriPhysio/Social/robotInterface.h>

using namespace hriPhysio::Social;


RobotInterface::RobotInterface() :
    name("") {

}


RobotInterface::~RobotInterface() {
}


void RobotInterface::setName(const std::string& name) {
    this->name = name;
    return;
}


std::string RobotInterface::getName() const {
    return this->name;
}


void RobotInterface::robotLoop() {
    //-- empty.
    return;
}


bool RobotInterface::setPerphState(const peripheral perph, const std::vector<double>& pos) {
    warning("setPerphState");
    return false;
}


bool RobotInterface::getPerphState(const peripheral perph, std::vector<double>& pos) {
    warning("getPerphState");
    return false;
}


bool RobotInterface::setPerphVelocity(const peripheral perph, const std::vector<double>& speed) {
    warning("setPerphVelocity");
    return false;
}


bool RobotInterface::getPerphVelocity(const peripheral perph, std::vector<double>& speed) {
    warning("getPerphVelocity");
    return false;
}


bool RobotInterface::setEmotionState(const std::string& emotion) {
    warning("setEmotionState");
    return false;
}


bool RobotInterface::getEmotionState(std::string& emotion) {
    warning("getEmotionState");
    return false;
}


bool RobotInterface::addGesture(const std::string gesture, const double speed/*=1.0*/) {
    warning("addGesture");
    return false;
}


bool RobotInterface::addSpeech(const std::string phrase) {
    warning("addSpeech");
    return false;
}


bool RobotInterface::setSpeechConfig(const std::string config) {
    warning("setSpeechConfig");
    return false;
}

bool RobotInterface::setVolume(const double) {
    warning("setVolume");
    return false;
}


bool RobotInterface::addAudioFile(const std::string filename, const size_t channel/*=-1*/) {
    warning("addAudioFile");
    return false;
}

bool RobotInterface::addVideoFile(const std::string filename) {
    warning("addVideoFile");
    return false;
}

bool RobotInterface::getRobotCommand(std::string& command) {
    warning("getRobotCommand");
    return false;
}
