#include <dynamic_transform_publisher/dynamic_tf.h>

DynamicTransform::DynamicTransform(ros::NodeHandle nh)
{
    server.reset(new ReconfigureServer(config_mutex, nh));
    ReconfigureServer::CallbackType f = boost::bind(&DynamicTransform::configureCB, this, _1, _2);
    server->setCallback(f);
}


void DynamicTransform::set(double x, double y, double z, double roll, double pitch, double yaw, std::string frame_id, std::string child_frame_id)
{
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

    server->updateConfig(config);
}

void DynamicTransform::set(double x, double y, double z, double qx, double qy, double qz, double qw, std::string frame_id, std::string child_frame_id)
{
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

    server->updateConfig(config);
}

void DynamicTransform::configureCB(dynamic_transform_publisher::TFConfig &config, uint32_t level)
{
    geometry_msgs::TransformStamped t;
    t.transform.translation.x = config.x;
    t.transform.translation.y = config.y;
    t.transform.translation.z = config.z;

    if(config.use_rpy)
    {
        config.groups.rpy.state = true;
        config.groups.quaternion.state = false;

        tf::Quaternion q;
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
        tf::Quaternion q(config.qx, config.qy, config.qz, config.qw);
        tf::Matrix3x3(q).getRPY(config.roll, config.pitch, config.yaw);
    }

    t.transform.rotation.x = config.qx;
    t.transform.rotation.y = config.qy;
    t.transform.rotation.z = config.qz;
    t.transform.rotation.w = config.qw;
    t.header.frame_id = config.frame_id;
    t.child_frame_id = config.child_frame_id;
    t.header.stamp = ros::Time::now();

    if(config.frame_id == "" || config.child_frame_id == "")
    {
        ROS_WARN_STREAM("No transforms will be published until frames are set. "
                         << "\n frame_id: " << config.frame_id
                         <<"\n child_frame_id: " << config.child_frame_id);
    }
    else if(config.frame_id == config.child_frame_id)
    {
        ROS_WARN_STREAM("frame_id and child_frame_id cannot be the same. "
                         << "\n frame_id: " << config.frame_id
                         <<"\n child_frame_id: " << config.child_frame_id);
    }
    else
    {
        std::vector<geometry_msgs::TransformStamped> transforms;
        transforms.push_back(t);
        br.sendTransform(transforms);
    }
}
