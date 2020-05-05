/**
 * @file interactive_transform.h
 * @brief Helper for an interactive marker that provides and Eigen transform
 *
 * @author Matthew Powelson
 * @date May 4, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef TESSERACT_ROSUTILS_INTERACTIVE_TRANSFORM_H
#define TESSERACT_ROSUTILS_INTERACTIVE_TRANSFORM_H

#include <tesseract_common/macros.h>

TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <Eigen/Eigen>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_rosutils {

class InteractiveTransform
{
public:
  using Ptr = std::shared_ptr<InteractiveTransform>;
  using ConstPtr = std::shared_ptr<const InteractiveTransform>;

  InteractiveTransform(const ros::NodeHandle& nh, std::string base_frame);
  ~InteractiveTransform() = default;

  Eigen::Isometry3d getEigenTransform();

  void setTransform(Eigen::Isometry3d transform);

private:
  ros::NodeHandle nh_;
  interactive_markers::InteractiveMarkerServer server_;
  visualization_msgs::InteractiveMarker interactive_marker_;
  visualization_msgs::Marker visual_marker_;
};
}


#endif
