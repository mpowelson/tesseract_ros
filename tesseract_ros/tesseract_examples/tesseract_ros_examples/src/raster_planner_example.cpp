/**
 * @file pick_and_place_example.cpp
 * @brief Pick and place implementation
 *
 * @author Levi Armstrong
 * @date July 22, 2019
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
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
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <jsoncpp/json/json.h>
#include <ros/ros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_ros_examples/raster_planner_example.h>
#include <tesseract_motion_planners/trajopt/config/trajopt_planner_config.h>
#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>
#include <tesseract_rosutils/plotting.h>
#include <tesseract_rosutils/utils.h>
#include <tesseract_msgs/ModifyEnvironment.h>
#include <tesseract_msgs/GetEnvironmentChanges.h>
#include <trajopt/plot_callback.hpp>
#include <trajopt/file_write_callback.hpp>
#include <trajopt/problem_description.hpp>
#include <trajopt_utils/config.hpp>
#include <trajopt_utils/logging.hpp>

using namespace trajopt;
using namespace tesseract;
using namespace tesseract_environment;
using namespace tesseract_kinematics;
using namespace tesseract_scene_graph;
using namespace tesseract_collision;
using namespace tesseract_rosutils;

const std::string ROBOT_DESCRIPTION_PARAM = "robot_description"; /**< Default ROS parameter for robot description */
const std::string ROBOT_SEMANTIC_PARAM = "robot_description_semantic"; /**< Default ROS parameter for robot
                                                                          description */
const std::string GET_ENVIRONMENT_CHANGES_SERVICE = "get_tesseract_changes_rviz";
const std::string MODIFY_ENVIRONMENT_SERVICE = "modify_tesseract_rviz";
const bool ENABLE_TIME_COST = false;
const bool ENABLE_VELOCITY_COST = false;
const double OFFSET = 0.005;

namespace tesseract_ros_examples
{

inline CompositeInstruction rasterPlannerExampleProgram()
{
  CompositeInstruction program;
  // Start Joint Position for the program
  Waypoint wp1 = JointWaypoint(Eigen::VectorXd::Zero(7));
  MoveInstruction start_instruction(wp1, MoveInstructionType::START);
  program.setStartInstruction(start_instruction);

  // Define raster poses
  Waypoint wp2 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(1, -1, 1));
  Waypoint wp3 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(1, -0.4, 1));
  Waypoint wp4 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(1, -0.2, 1));
  Waypoint wp5 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(1, 0.0, 1));
  Waypoint wp6 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(1, 0.2, 1));
  Waypoint wp7 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(1, 0.4, 1));

  // Define raster move instruction
  PlanInstruction plan_c1(wp3, PlanInstructionType::LINEAR, "RASTER");
  PlanInstruction plan_c2(wp4, PlanInstructionType::LINEAR, "RASTER");
  PlanInstruction plan_c3(wp5, PlanInstructionType::LINEAR, "RASTER");
  PlanInstruction plan_c4(wp6, PlanInstructionType::LINEAR, "RASTER");
  PlanInstruction plan_c5(wp7, PlanInstructionType::LINEAR, "RASTER");

  PlanInstruction plan_f0(wp2, PlanInstructionType::FREESPACE, "FREESPACE");
  plan_f0.setDescription("from_start_plan");
  CompositeInstruction from_start;
  from_start.setDescription("from_start");
  from_start.push_back(plan_f0);
  program.push_back(from_start);

  {
    CompositeInstruction raster_segment;
    raster_segment.setDescription("raster_segment");
    CompositeInstruction raster;
    raster.setDescription("raster");
    raster.push_back(plan_c1);
    raster.push_back(plan_c2);
    raster.push_back(plan_c3);
    raster.push_back(plan_c4);
    raster.push_back(plan_c5);
    raster_segment.push_back(raster);
    program.push_back(raster_segment);
  }

  {
    PlanInstruction plan_f1(wp2, PlanInstructionType::FREESPACE, "FREESPACE");
    plan_f1.setDescription("transition_from_end_plan");
    CompositeInstruction transition_from_end;
    transition_from_end.setDescription("transition_from_end");
    transition_from_end.push_back(plan_f1);
    CompositeInstruction transition_from_start;
    transition_from_start.setDescription("transition_from_start");
    transition_from_start.push_back(plan_f1);

    CompositeInstruction transitions("DEFAULT", CompositeInstructionOrder::UNORDERED);
    transitions.setDescription("transitions");
    transitions.push_back(transition_from_start);
    transitions.push_back(transition_from_end);
    program.push_back(transitions);
  }

  {
    CompositeInstruction raster_segment;
    raster_segment.setDescription("raster_segment");
    CompositeInstruction raster;
    raster.setDescription("raster");
    raster.push_back(plan_c1);
    raster.push_back(plan_c2);
    raster.push_back(plan_c3);
    raster.push_back(plan_c4);
    raster.push_back(plan_c5);
    raster_segment.push_back(raster);
    program.push_back(raster_segment);
  }

  {
    PlanInstruction plan_f1(wp2, PlanInstructionType::FREESPACE, "FREESPACE");
    plan_f1.setDescription("transition_from_end_plan");
    CompositeInstruction transition_from_end;
    transition_from_end.setDescription("transition_from_end");
    transition_from_end.push_back(plan_f1);
    CompositeInstruction transition_from_start;
    transition_from_start.setDescription("transition_from_start");
    transition_from_start.push_back(plan_f1);

    CompositeInstruction transitions("DEFAULT", CompositeInstructionOrder::UNORDERED);
    transitions.setDescription("transitions");
    transitions.push_back(transition_from_start);
    transitions.push_back(transition_from_end);
    program.push_back(transitions);
  }

  {
    CompositeInstruction raster_segment;
    raster_segment.setDescription("raster_segment");
    CompositeInstruction raster;
    raster.setDescription("raster");
    raster.push_back(plan_c1);
    raster.push_back(plan_c2);
    raster.push_back(plan_c3);
    raster.push_back(plan_c4);
    raster.push_back(plan_c5);
    raster_segment.push_back(raster);
    program.push_back(raster_segment);
  }

  PlanInstruction plan_f2(wp2, PlanInstructionType::FREESPACE, "FREESPACE");
  plan_f2.setDescription("to_end_plan");
  CompositeInstruction to_end;
  to_end.setDescription("to_end");
  to_end.push_back(plan_f2);
  program.push_back(to_end);

  return program;
}

bool RasterPlannerExample::run()
{
  // Set Log Level
  util::gLogLevel = util::LevelError;

  /////////////
  /// SETUP ///
  /////////////

  // Pull ROS params
  std::string urdf_xml_string, srdf_xml_string, box_parent_link;
  double box_side, box_x, box_y;
  nh_.getParam(ROBOT_DESCRIPTION_PARAM, urdf_xml_string);
  nh_.getParam(ROBOT_SEMANTIC_PARAM, srdf_xml_string);

  // Initialize the environment
  ResourceLocator::Ptr locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
  if (!tesseract_->init(urdf_xml_string, srdf_xml_string, locator))
    return false;

  // Create plotting tool
  tesseract_rosutils::ROSPlottingPtr plotter =
      std::make_shared<tesseract_rosutils::ROSPlotting>(tesseract_->getEnvironment()->getSceneGraph()->getRoot());

  if (rviz_)
  {
    // These are used to keep visualization updated
    modify_env_rviz_ = nh_.serviceClient<tesseract_msgs::ModifyEnvironment>(MODIFY_ENVIRONMENT_SERVICE, false);
    get_env_changes_rviz_ =
        nh_.serviceClient<tesseract_msgs::GetEnvironmentChanges>(GET_ENVIRONMENT_CHANGES_SERVICE, false);

    // Check RViz to make sure nothing has changed
    if (!checkRviz())
      return false;
  }
  sleep(20);
  // Set the initial state of the robot
  std::unordered_map<std::string, double> joint_states;
  joint_states["iiwa_joint_1"] = 0.0;
  joint_states["iiwa_joint_2"] = 0.0;
  joint_states["iiwa_joint_3"] = 0.0;
  joint_states["iiwa_joint_4"] = -1.57;
  joint_states["iiwa_joint_5"] = 0.0;
  joint_states["iiwa_joint_6"] = 0.0;
  joint_states["iiwa_joint_7"] = 0.0;
  tesseract_->getEnvironment()->setState(joint_states);

  if (rviz_)
  {
    // Now update rviz environment
    if (!sendRvizChanges(0))
      return false;
  }

  if (rviz_)
  {
    ROS_ERROR("Press enter to continue");
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  }

  // Choose the manipulator and end effector link
  std::string manip = "manipulator";
  std::string end_effector = "iiwa_link_ee";



  // Plot the resulting trajectory
//  if (plotting_)
//    plotter->plotTrajectory(pick_prob->GetKin()->getJointNames(),
//                            planning_response.joint_trajectory.trajectory.leftCols(
//                                static_cast<long>(pick_prob->GetKin()->getJointNames().size())));

//  std::cout << planning_response.joint_trajectory.trajectory << '\n';

  ROS_INFO("Done");
  return true;
}
}  // namespace tesseract_ros_examples
