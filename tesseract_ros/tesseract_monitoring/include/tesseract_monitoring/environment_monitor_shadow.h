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

#ifndef TESSERACT_MONITORING_ENVIRONMENT_MONITOR_MIRROR_H
#define TESSERACT_MONITORING_ENVIRONMENT_MONITOR_MIRROR_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ros/ros.h>
#include <memory>
#include <mutex>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/tesseract.h>
#include <tesseract_monitoring/current_state_monitor.h>
#include <tesseract_monitoring/environment_monitor.h>

namespace tesseract_monitoring
{
/**
 * @brief TesseractMonitor
 * Connects to an EnvironmentMonitor and updates whenever it is updated*/
class EnvironmentMonitorShadow
{
public:
  using Ptr = std::shared_ptr<EnvironmentMonitorShadow>;
  using ConstPtr = std::shared_ptr<const EnvironmentMonitorShadow>;

  EnvironmentMonitorShadow(std::string environment_update_topic = "/tesseract_monitoring_environment/"
                                                                  "monitored_tesseract",
                           std::string robot_description = "robot_description");

  ~EnvironmentMonitorShadow();
  EnvironmentMonitorShadow(const EnvironmentMonitorShadow&) = delete;
  EnvironmentMonitorShadow& operator=(const EnvironmentMonitorShadow&) = delete;
  EnvironmentMonitorShadow(EnvironmentMonitorShadow&&) = delete;
  EnvironmentMonitorShadow& operator=(EnvironmentMonitorShadow&&) = delete;

  const tesseract::Tesseract::Ptr& getTesseract();
  tesseract::Tesseract::ConstPtr getTesseractConst();

  /**
   * @brief Queries the Environment monitor for updates rather than waiting on the published topic
   * @return
   */
  bool updateEnvironment();

  /** @brief Update the scene using the monitored state. */
  void updateEnvironmentWithCurrentState();

  void startStateMonitor(const std::string& joint_states_topic);

  void stopStateMonitor();

protected:
  void callbackTesseractEnvDiff(const tesseract_msgs::TesseractStatePtr& state);

  tesseract::Tesseract::Ptr tesseract_;
  boost::mutex update_mutex_;  /// mutex for updating the environment
  // Lock for state_update_pending_ and dt_state_update_
  std::mutex state_pending_mutex_;

  ros::NodeHandle pnh_;
  ros::NodeHandle nh_;

  // include a current state monitor
  CurrentStateMonitorPtr current_state_monitor_;

  ros::Subscriber environment_monitor_subscriber_;
};
using EnvironmentMonitorShadowPtr = std::shared_ptr<EnvironmentMonitorShadow>;
using EnvironmentMonitorShadowConstPtr = std::shared_ptr<const EnvironmentMonitorShadow>;

}  // namespace tesseract_monitoring

#endif
