#include <tesseract_rosutils/interactive_transform.h>

using namespace tesseract_rosutils;

void processFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  ROS_INFO_STREAM( feedback->marker_name << " is now at "
      << feedback->pose.position.x << ", " << feedback->pose.position.y
      << ", " << feedback->pose.position.z );
}

  InteractiveTransform::InteractiveTransform(const ros::NodeHandle& nh, std::string base_frame)
      : nh_(nh), server_("interactive_transform","",false)
  {
    // create an interactive marker for our server
    {
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = base_frame;
    int_marker.header.stamp = ros::Time::now();
    int_marker.name = "interactive_transform";
    int_marker.description = "6 DOF marker for getting an Eigen Isometry3d";
    interactive_marker_ = int_marker;
    }

    // create a grey box marker
    {
    visualization_msgs::Marker box_marker;
    box_marker.type = visualization_msgs::Marker::SPHERE;
    box_marker.scale.x = 0.1;
    box_marker.scale.y = 0.1;
    box_marker.scale.z = 0.1;
    box_marker.color.r = 0.5;
    box_marker.color.g = 0.5;
    box_marker.color.b = 0.5;
    box_marker.color.a = 1.0;
    visual_marker_ = box_marker;
    }

    // create a non-interactive control which contains the box
    visualization_msgs::InteractiveMarkerControl control;
    control.always_visible = true;
    control.markers.push_back( visual_marker_ );

    // add the control to the interactive marker
    interactive_marker_.controls.push_back( control );

    // create a control which will move the box
    // this control does not contain any markers,
    // which will cause RViz to insert two arrows
    visualization_msgs::InteractiveMarkerControl rotate_control;
    rotate_control.name = "move_6dof";
    rotate_control.interaction_mode =
        visualization_msgs::InteractiveMarkerControl::NONE;

    // add the control to the interactive marker
    interactive_marker_.controls.push_back(rotate_control);

    // add the interactive marker to our collection &
    // tell the server to call processFeedback() when feedback arrives for it
    server_.insert(interactive_marker_, &processFeedback);

    // 'commit' changes and send to all clients
    server_.applyChanges();
  }
