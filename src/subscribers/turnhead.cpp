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
    float waittime = 3.0f;

    // ensure motors are active
    p_motion_.async<void>(
    "setStiffnesses",
    1.0);
    
    //define basic vars
    float fractionMaxSpeed  = 0.1f;
    std::vector<std::string> names;
    names.push_back("HeadYaw");
    names.push_back("HeadPitch");
    
    // look forward
    
    std::vector<float> angles;
    angles.push_back(0.0f);
    angles.push_back(0.0f);
    //float angles.push_back] = {0.0f, 0.0f};
    
    p_motion_.async<void>(
    "setAngles",
    names,
    angles,
    fractionMaxSpeed);
    
    //ros::Duration(waittime).sleep();
    
    
    
    // look down for 1 sec
    
    angles.clear();
    angles.push_back(0.0f);
    angles.push_back(-radius);
    
    p_motion_.async<void>(
    "setAngles",
    names,
    angles,
    fractionMaxSpeed);
    
    ros::Duration(waittime).sleep();
    
    
    
    // look left for 1 sec
    
    angles.clear();
    angles.push_back(-radius);
    angles.push_back(0.0f);
    
    p_motion_.async<void>(
    "setAngles",
    names,
    angles,
    fractionMaxSpeed);
    
    ros::Duration(waittime).sleep();
    
    
    
    // look up for 1 sec
    
    angles.clear();
    angles.push_back(0.0f);
    angles.push_back(radius);
    
    p_motion_.async<void>(
    "setAngles",
    names,
    angles,
    fractionMaxSpeed);
    
    ros::Duration(waittime).sleep();
    
    
    
    // look right for 1 sec
    
    angles.clear();
    angles.push_back(radius);
    angles.push_back(0.0f);
    
    p_motion_.async<void>(
    "setAngles",
    names,
    angles,
    fractionMaxSpeed);
    
    ros::Duration(waittime).sleep();
    
    
    
    // look forward (till next round)
    
    angles.clear();
    angles.push_back(0.0f);
    angles.push_back(0.0f);
    
    p_motion_.async<void>(
    "setAngles",
    names,
    angles,
    fractionMaxSpeed);

}

} //publisher
} // naoqi
