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


#ifndef TURNHEAD_SUBSCRIBER_HPP
#define TURNHEAD_SUBSCRIBER_HPP

/*
 * LOCAL includes
 */
#include "subscriber_base.hpp"

/*
 * ROS includes
 */
#include <ros/ros.h>


namespace naoqi
{
namespace subscriber
{

class TurnheadSubscriber: public BaseSubscriber<TurnheadSubscriber>
{
public:
  TurnheadSubscriber( const std::string& name, const std::string& topic, const qi::SessionPtr& session );
  ~TurnheadSubscriber(){}

  void reset( ros::NodeHandle& nh );
  void callback( const float radius );

private:
  qi::AnyObject p_motion_;
  ros::Subscriber sub_turnhead_;
}; // class Teleop

} // subscriber
}// naoqi
#endif
