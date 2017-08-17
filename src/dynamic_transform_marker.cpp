#include <dynamic_transform_publisher/dynamic_transform_marker.h>

DynamicMarkerControl::DynamicMarkerControl(const CallbackType &callback, const dynamic_transform_publisher::TFConfig &init_config)
    : server_(ros::this_node::getName())
    , callback_(callback)
    , config_(init_config){

    ros::NodeHandle nh("~");
    marker_.header.frame_id = config_.frame_id;
    marker_.name = config_.child_frame_id;
    marker_.description = config_.child_frame_id;
    marker_.pose.position.x = config_.x;
    marker_.pose.position.y = config_.y;
    marker_.pose.position.z = config_.z;
    marker_.pose.orientation.x = config_.qx;
    marker_.pose.orientation.y = config_.qy;
    marker_.pose.orientation.z = config_.qz;
    marker_.pose.orientation.w = config_.qw;
    marker_.scale = nh.param<double>("marker_scale", 0.1);

    visualization_msgs::InteractiveMarkerControl marker_control;
    marker_control.name="rotate_x";
    marker_control.orientation.w = 1;
    marker_control.orientation.x = 1;
    marker_control.orientation.y = 0;
    marker_control.orientation.z = 0;
    marker_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    marker_.controls.push_back(marker_control);
    marker_control.name="rotate_y";
    marker_control.orientation.w = 1;
    marker_control.orientation.x = 0;
    marker_control.orientation.y = 0;
    marker_control.orientation.z = 1;
    marker_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    marker_.controls.push_back(marker_control);
    marker_control.name="rotate_z";
    marker_control.orientation.w = 1;
    marker_control.orientation.x = 0;
    marker_control.orientation.y = 1;
    marker_control.orientation.z = 0;
    marker_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    marker_.controls.push_back(marker_control);
    marker_control.name="move_x";
    marker_control.orientation.w = 1;
    marker_control.orientation.x = 1;
    marker_control.orientation.y = 0;
    marker_control.orientation.z = 0;
    marker_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    marker_.controls.push_back(marker_control);
    marker_control.name="move_y";
    marker_control.orientation.w = 1;
    marker_control.orientation.x = 0;
    marker_control.orientation.y = 0;
    marker_control.orientation.z = 1;
    marker_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    marker_.controls.push_back(marker_control);
    marker_control.name="move_z";
    marker_control.orientation.w = 1;
    marker_control.orientation.x = 0;
    marker_control.orientation.y = 1;
    marker_control.orientation.z = 0;
    marker_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    marker_.controls.push_back(marker_control);

    server_.insert(marker_, boost::bind(&DynamicMarkerControl::processFeedback, this, _1));
    server_.applyChanges();
}

void DynamicMarkerControl::updatePose(const geometry_msgs::Pose &pose)
{
    server_.setPose(marker_.name, pose);
    server_.applyChanges();
}

void DynamicMarkerControl::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    if(pose_history_.size() == 0 || feedback->event_type == feedback->MOUSE_DOWN)
    {
        ROS_DEBUG("Adding pose to history");
        addToHistory(current_pose_);
    }
    //! update dynamic transform
    tf2::fromMsg(feedback->pose, current_pose_);
    ROS_INFO_STREAM("new transform: (" <<current_pose_.getOrigin().x() << ", " << current_pose_.getOrigin().y() << ", " << current_pose_.getOrigin().z() << ") "
                    << "("<<current_pose_.getRotation().x() << ", " << current_pose_.getRotation().y() << ", " <<current_pose_.getRotation().z() << ", " << current_pose_.getRotation().w() << ")");

    tfToConfig(current_pose_, config_);

    callback_(config_);
}

void DynamicMarkerControl::addToHistory(tf2::Transform pose)
{
    pose_history_.push_back(pose);

    while (pose_history_.size() > 100)
    {
        pose_history_.pop_front();
    }
}

void DynamicMarkerControl::tfToConfig(const tf2::Transform &pose, dynamic_transform_publisher::TFConfig &config)
{
    config.x = pose.getOrigin().x();
    config.y = pose.getOrigin().y();
    config.z = pose.getOrigin().z();
    config.qx = pose.getRotation().getX();
    config.qy = pose.getRotation().getY();
    config.qz = pose.getRotation().getZ();
    config.qw = pose.getRotation().getW();

    //! update rpy from quaternion
    tf2::Quaternion q(config.qx, config.qy, config.qz, config.qw);
    tf2::Matrix3x3(q).getRPY(config.roll, config.pitch, config.yaw);
}

