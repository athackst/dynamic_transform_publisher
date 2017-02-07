#include <dynamic_transform_publisher/dynamic_transform_marker.h>

DynamicMarkerControl::DynamicMarkerControl(const boost::shared_ptr<ReconfigureServer> &server, const dynamic_transform_publisher::TFConfig &config)
    : server_(ros::this_node::getName())
    , reconfigure_server_(server)
    , config_(config){
    //visualization_msgs::InteractiveMarker marker;
    marker_.header.frame_id = config.frame_id;
    marker_.name = config.child_frame_id;
    marker_.description = config.child_frame_id;
    marker_.pose.position.x = config.x;
    marker_.pose.position.y = config.y;
    marker_.pose.position.z = config.z;
    marker_.pose.orientation.x = config.qx;
    marker_.pose.orientation.y = config.qy;
    marker_.pose.orientation.z = config.qz;
    marker_.pose.orientation.w = config.qw;
    marker_.scale = 0.1;

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

void DynamicMarkerControl::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    if(pose_history_.size() == 0 || feedback->event_type == feedback->MOUSE_DOWN)
    {
        ROS_DEBUG("Adding pose to history");
        addToHistory(current_pose_);
    }
    //! update dynamic transform
    tf::poseMsgToTF(feedback->pose, current_pose_);
    ROS_DEBUG_STREAM("new transform: (" <<current_pose_.getOrigin().x() << ", " << current_pose_.getOrigin().y() << ", " << current_pose_.getOrigin().z() << ") "
                    << "("<<current_pose_.getRotation().x() << ", " << current_pose_.getRotation().y() << ", " <<current_pose_.getRotation().z() << ", " << current_pose_.getRotation().w() << ")");

    tfToConfig(current_pose_, config_);

    reconfigure_server_->updateConfig(config_);
}

void DynamicMarkerControl::addToHistory(tf::Transform pose)
{
    pose_history_.push_back(pose);

    while (pose_history_.size() > 100)
    {
        pose_history_.pop_front();
    }
}

void DynamicMarkerControl::tfToConfig(const tf::Transform &pose, dynamic_transform_publisher::TFConfig &config)
{
    config.x = pose.getOrigin().x();
    config.y = pose.getOrigin().y();
    config.z = pose.getOrigin().z();
    config.qx = pose.getRotation().getX();
    config.qy = pose.getRotation().getY();
    config.qz = pose.getRotation().getZ();
    config.qw = pose.getRotation().getW();
}

