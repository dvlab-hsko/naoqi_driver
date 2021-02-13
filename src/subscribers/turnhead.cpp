/*
 * Copyright 2015 Aldebaran
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

/*
 * LOCAL includes
 */
#include "turnhead.hpp"

/*
 * ROS includes
 */
#include <std_msgs/Float32.h>

namespace naoqi
{
namespace subscriber
{

TurnheadSubscriber::TurnheadSubscriber( const std::string& name, const std::string& topic, const qi::SessionPtr& session ):
  BaseSubscriber( name, topic, session ),
  p_motion_( session->service("ALMotion") )
{}

void TurnheadSubscriber::reset( ros::NodeHandle& nh )
{
  sub_turnhead_ = nh.subscribe( topic_, 10, &TurnheadSubscriber::callback, this );
  is_initialized_ = true;
}

void TurnheadSubscriber::callback( const std_msgs::Float32::ConstPtr & inputRadius )
{
    
    float radius = float(inputRadius->data);

    // ensure motors are active
    p_motion_.async<void>(
    "setStiffnesses",
    1.0);
    
    //define basic vars
    float holdTime = 2.0f;
    float transTime = 2.0f;
    
    std::vector<std::string> names;
    names.push_back("HeadYaw");
    names.push_back("HeadPitch");
    
    std::vector<float> tempYawAngles;
    std::vector<float> tempPitchAngles;
    std::vector<std::vector<float>> angles;
    
    std::vector<float> tempYawTimes;
    std::vector<float> tempPitchTimes;
    std::vector<std::vector<float>> times;
    
    
    

    
    float countTime = 0.0f;
    
    tempYawAngles.push_back(0.0f);
    tempPitchAngles.push_back(0.0f);
    
    tempYawTimes.push_back(countTime);
    tempPitchTimes.push_back(countTime);
    
    
    // move head from forward to down
    
    countTime += transTime;
    
    tempYawAngles.push_back(0.0f);
    tempPitchAngles.push_back(-radius);
    
    tempYawTimes.push_back(countTime);
    tempPitchTimes.push_back(countTime);
    
    
    // hold position
    
    countTime += holdTime;
    
    tempYawAngles.push_back(0.0f);
    tempPitchAngles.push_back(-radius);
    
    tempYawTimes.push_back(countTime);
    tempPitchTimes.push_back(countTime);
    
    
    
    
    // move head from forward to left
    
    countTime += transTime;
    
    tempYawAngles.push_back(-radius);
    tempPitchAngles.push_back(0.0f);
    
    tempYawTimes.push_back(countTime);
    tempPitchTimes.push_back(countTime);
    
    
    // hold position
    
    countTime += holdTime;
    
    tempYawAngles.push_back(-radius);
    tempPitchAngles.push_back(0.0f);
    
    tempYawTimes.push_back(countTime);
    tempPitchTimes.push_back(countTime);
    
    
    
    
    // move head from forward to up
    
    countTime += transTime;
    
    tempYawAngles.push_back(0.0f);
    tempPitchAngles.push_back(radius);
    
    tempYawTimes.push_back(countTime);
    tempPitchTimes.push_back(countTime);
    
    
    // hold position
    
    countTime += holdTime;
    
    tempYawAngles.push_back(0.0f);
    tempPitchAngles.push_back(radius);
    
    tempYawTimes.push_back(countTime);
    tempPitchTimes.push_back(countTime);
    
    
    
    
    // move head from forward to right
    
    countTime += transTime;
    
    tempYawAngles.push_back(radius);
    tempPitchAngles.push_back(0.0f);
    
    tempYawTimes.push_back(countTime);
    tempPitchTimes.push_back(countTime);
    
    
    // hold position
    
    countTime += holdTime;
    
    tempYawAngles.push_back(radius);
    tempPitchAngles.push_back(0.0f);
    
    tempYawTimes.push_back(countTime);
    tempPitchTimes.push_back(countTime);
    
    
    
    
    
    // look forward (till next round)
    
    countTime += transTime;
    
    tempYawAngles.push_back(0.0f);
    tempPitchAngles.push_back(0.0f);
    
    tempYawTimes.push_back(countTime);
    tempPitchTimes.push_back(countTime);
    
    angles.push_back(tempYawAngles);
    angles.push_back(tempPitchAngles);
    times.push_back(tempYawTimes);
    times.push_back(tempPitchTimes);
    
    std::cout << "Movehead sending angleInterpolation to NaoQi with " << radius << " as radius" << std::endl;
    
    p_motion_.async<void>(
    "angleInterpolationBezier",
    names,
    times,
    angles);

}

} //publisher
} // naoqi
