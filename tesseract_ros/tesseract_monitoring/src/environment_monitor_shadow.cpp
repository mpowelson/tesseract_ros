/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan */

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ros/console.h>
#include <memory>

TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/core/utils.h>
#include <tesseract_monitoring/environment_monitor_shadow.h>

namespace tesseract_monitoring
{
EnvironmentMonitorShadow::EnvironmentMonitorShadow(std::string environment_update_topic, std::string robot_description)
  : pnh_("~")
{
  // Initial setup
  // TODO: Add init info to command history so that this isn't necessary anymore
  std::string urdf_xml_string, srdf_xml_string;
  if (!nh_.hasParam(robot_description))
  {
    ROS_ERROR("Failed to find parameter: %s", robot_description.c_str());
    return;
  }

  if (!nh_.hasParam(robot_description + "_semantic"))
  {
    ROS_ERROR("Failed to find parameter: %s", (robot_description + "_semantic").c_str());
    return;
  }

  nh_.getParam(robot_description, urdf_xml_string);
  nh_.getParam(robot_description + "_semantic", srdf_xml_string);

  tesseract_ = std::make_shared<tesseract::Tesseract>();
  tesseract_scene_graph::ResourceLocator::Ptr locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
  if (!tesseract_->init(urdf_xml_string, srdf_xml_string, locator))
    return;

  if (!tesseract_->isInitialized())
  {
    ROS_ERROR("Failed to initalize environment monitor shadow");
    return;
  }

  environment_monitor_subscriber_ =
      nh_.subscribe(environment_update_topic, 100, &EnvironmentMonitorShadow::callbackTesseractEnvDiff, this);
}

EnvironmentMonitorShadow::~EnvironmentMonitorShadow()
{
  current_state_monitor_.reset();
  tesseract_.reset();
}

const tesseract::Tesseract::Ptr& EnvironmentMonitorShadow::getTesseract()
{
  if (current_state_monitor_)
    updateEnvironmentWithCurrentState();
  return tesseract_;
}
tesseract::Tesseract::ConstPtr EnvironmentMonitorShadow::getTesseractConst()
{
  if (current_state_monitor_)
    updateEnvironmentWithCurrentState();
  return tesseract_;
}

bool EnvironmentMonitorShadow::updateEnvironment()
{
  // This should query the environment monitor via services to force an update
  ROS_ERROR("updateEnvironment has not been implemented yet.");

  if (current_state_monitor_)
    updateEnvironmentWithCurrentState();
  return false;
}

void EnvironmentMonitorShadow::callbackTesseractEnvDiff(const tesseract_msgs::TesseractStatePtr& state)
{
  update_mutex_.lock();
  if (!tesseract_rosutils::processMsg(*(tesseract_->getEnvironment()), *state))
  {
    ROS_ERROR("Invalid TesseractState message");
  }
  update_mutex_.unlock();

  return;
}

void EnvironmentMonitorShadow::startStateMonitor(const std::string& joint_states_topic)
{
  stopStateMonitor();
  if (tesseract_->getEnvironment())
  {
    if (!current_state_monitor_)
      current_state_monitor_.reset(
          new CurrentStateMonitor(tesseract_->getEnvironment(), tesseract_->getFwdKinematicsManager(), nh_));

    // Don't update on callback. Only update when requested.
    //    current_state_monitor_->addUpdateCallback(boost::bind(&EnvironmentMonitorShadow::onStateUpdate, this, _1));
    current_state_monitor_->startStateMonitor(joint_states_topic);
  }
  else
  {
    ROS_ERROR("Cannot monitor robot state because planning scene is not configured");
  }
}

void EnvironmentMonitorShadow::stopStateMonitor()
{
  if (current_state_monitor_)
    current_state_monitor_->stopStateMonitor();
}

void EnvironmentMonitorShadow::updateEnvironmentWithCurrentState()
{
  if (current_state_monitor_)
  {
    std::vector<std::string> missing;
    if (!current_state_monitor_->haveCompleteState(missing) &&
        (ros::Time::now() - current_state_monitor_->getMonitorStartTime()).toSec() > 1.0)
    {
      std::string missing_str = boost::algorithm::join(missing, ", ");
      ROS_WARN_THROTTLE(
          1, "The complete state of the robot is not yet known.  Missing %s", missing_str.c_str());
    }

    {
      update_mutex_.lock();
      tesseract_->getEnvironment()->setState(current_state_monitor_->getCurrentState()->joints);
      update_mutex_.unlock();
    }
  }
  else
    ROS_ERROR_THROTTLE(1, "State monitor is not active. Unable to set the planning scene state");
}

}  // namespace tesseract_monitoring
