#include <dynamic_transform_publisher/dynamic_transform_broadcaster.h>

using namespace tf2_ros;

DynamicTransformBroadcaster::DynamicTransformBroadcaster(ros::NodeHandle nh)
{
    server_.reset(new ReconfigureServer(config_mutex_, nh));

    update_timer_ = nh.createTimer(ros::Duration(0), &DynamicTransformBroadcaster::send, this, false, false);
}


void DynamicTransformBroadcaster::init(double x, double y, double z, double roll, double pitch, double yaw, std::string frame_id, std::string child_frame_id, double period)
{
    update_timer_.stop();
    dynamic_transform_publisher::TFConfig config;
    config.x = x;
    config.y = y;
    config.z = z;
    config.use_rpy = true;
    config.roll = roll;
    config.pitch = pitch;
    config.yaw = yaw;
    config.frame_id = frame_id;
    config.child_frame_id = child_frame_id;
    config.period = period;

    update(config);
}

void DynamicTransformBroadcaster::init(double x, double y, double z, double qx, double qy, double qz, double qw, std::string frame_id, std::string child_frame_id, double period)
{
    update_timer_.stop();
    dynamic_transform_publisher::TFConfig config;
    config.x = x;
    config.y = y;
    config.z = z;
    config.use_rpy = false;
    config.qx = qx;
    config.qy = qy;
    config.qz = qz;
    config.qw = qw;
    config.frame_id = frame_id;
    config.child_frame_id = child_frame_id;
    config.period = period;

    update(config);
}

void DynamicTransformBroadcaster::reconfigure_cb(dynamic_transform_publisher::TFConfig &config, uint32_t level)
{
    update_timer_.stop();

    if(config.use_rpy)
    {
        config.groups.rpy.state = true;
        config.groups.quaternion.state = false;
        tf2::Quaternion q;
        q.setRPY(config.roll, config.pitch, config.yaw);
        //! update quaternion from rpy
        config.qx = q.getX();
        config.qy = q.getY();
        config.qz = q.getZ();
        config.qw = q.getW();

    }
    else
    {
        config.groups.rpy.state = false;
        config.groups.quaternion.state = true;

        //! update rpy from quaternion
        tf2::Quaternion q(config.qx, config.qy, config.qz, config.qw);
        tf2::Matrix3x3(q).getRPY(config.roll, config.pitch, config.yaw);
    }

    update_timer_.setPeriod(ros::Duration(config.period/1000));

    config_ = config;

    ROS_INFO_STREAM("Updating transform for "
                     << "\n frame_id: "<<config_.frame_id
                     << "\n child_frame_id: "<<config_.child_frame_id
                     << "\n to: ["<<config_.x<<","<<config_.y<<","<<config_.z<<"]");

    if(config_.frame_id != "" && config_.child_frame_id != "" && config_.frame_id != config_.child_frame_id)
    {
        if(!marker_)
        {
            ROS_INFO_STREAM("Created interactive marker.");
            marker_.reset(new DynamicMarkerControl(boost::bind(&DynamicTransformBroadcaster::update, this, _1), config_));
        }
        marker_->updatePose(configToPose(config_));
    }

    update_timer_.start();
}

void DynamicTransformBroadcaster::update(const dynamic_transform_publisher::TFConfig &config)
{
    // update the gui
    server_->updateConfig(config);
    boost::recursive_mutex::scoped_lock lock(config_mutex_);
    // update the config file
    config_ = config;
}

geometry_msgs::Pose DynamicTransformBroadcaster::configToPose(const dynamic_transform_publisher::TFConfig &config)
{
    geometry_msgs::Pose pose;
    pose.position.x = config.x;
    pose.position.y = config.y;
    pose.position.z = config.z;
    pose.orientation.x = config.qx;
    pose.orientation.y = config.qy;
    pose.orientation.z = config.qz;
    pose.orientation.w = config.qw;
    return pose;
}

geometry_msgs::TransformStamped DynamicTransformBroadcaster::configToTransform(const dynamic_transform_publisher::TFConfig &config)
{
    geometry_msgs::TransformStamped transform;
    transform.transform.translation.x = config.x;
    transform.transform.translation.y = config.y;
    transform.transform.translation.z = config.z;
    transform.transform.rotation.x = config.qx;
    transform.transform.rotation.y = config.qy;
    transform.transform.rotation.z = config.qz;
    transform.transform.rotation.w = config.qw;
    transform.header.frame_id = config.frame_id;
    transform.header.stamp = ros::Time::now();
    transform.child_frame_id = config.child_frame_id;

    return transform;
}

void DynamicTransformBroadcaster::send(const ros::TimerEvent &e)
{
    boost::recursive_mutex::scoped_lock lock(config_mutex_);
    if(config_.frame_id == "" || config_.child_frame_id == "")
    {
        ROS_WARN_STREAM_ONCE("No transforms will be published until frames are set. "
                         << "\n frame_id: " << config_.frame_id
                         <<"\n child_frame_id: " << config_.child_frame_id);
    }
    else if(config_.frame_id == config_.child_frame_id)
    {
        ROS_WARN_STREAM_ONCE("frame_id and child_frame_id cannot be the same. "
                         << "\n frame_id: " << config_.frame_id
                         <<"\n child_frame_id: " << config_.child_frame_id);
    }
    else
    {
        br_.sendTransform(configToTransform(config_));
    }
}

void DynamicTransformBroadcaster::start()
{
    ReconfigureServer::CallbackType f = boost::bind(&DynamicTransformBroadcaster::reconfigure_cb, this, _1, _2);
    server_->setCallback(f);
}
