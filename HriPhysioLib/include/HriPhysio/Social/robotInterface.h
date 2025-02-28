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

#ifndef HRI_PHYSIO_SOCIAL_ROBOT_INTERFACE_H
#define HRI_PHYSIO_SOCIAL_ROBOT_INTERFACE_H

#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <HriPhysio/helpers.h>

namespace hriPhysio {
    namespace Social {
        class RobotInterface;
    }
}

class hriPhysio::Social::RobotInterface {

protected:
    std::string name;

public:
    RobotInterface();

    ~RobotInterface();

    void setName(const std::string& name);
    std::string getName() const;

    virtual bool configure(int argc, char **argv) = 0;

    virtual void robotLoop();
    
    enum peripheral { HEAD, RIGHTARM, LEFTARM, RIGHTLEG, LEFTLEG };
    
    virtual bool setPerphState(const peripheral perph, const std::vector<double>& pos);

    virtual bool getPerphState(const peripheral perph, std::vector<double>& pos);

    virtual bool setPerphVelocity(const peripheral perph, const std::vector<double>& speed);

    virtual bool getPerphVelocity(const peripheral perph, std::vector<double>& speed);

    virtual bool setEmotionState(const std::string& emotion);

    virtual bool getEmotionState(std::string& emotion);

    virtual bool addGesture(const std::string gesture, const double speed=1.0);

    virtual bool addSpeech(const std::string phrase);

    virtual bool setSpeechConfig(const std::string config);

    virtual bool setVolume(const double percent);

    virtual bool addAudioFile(const std::string filename, const size_t channel=-1);

    virtual bool addVideoFile(const std::string filename);

    virtual bool getRobotCommand(std::string& command);

private:
    void warning(std::string func) {
        std::cerr << "[DEBUG] " << "Function ``" << func << "`` has not been implemented!!" << std::endl;
    }

};

#endif /* HRI_PHYSIO_SOCIAL_ROBOT_INTERFACE_H */
