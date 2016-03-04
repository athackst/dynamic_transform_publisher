#include <dynamic_transform_publisher/dynamic_tf2.h>

DynamicTF2::DynamicTF2(ros::NodeHandle nh)
{
    server.reset(new ReconfigureServer(config_mutex, nh));

    update_timer = nh.createTimer(ros::Duration(0), &DynamicTF2::update, this, false, false);
}


void DynamicTF2::set(double x, double y, double z, double roll, double pitch, double yaw, std::string frame_id, std::string child_frame_id, double period)
{
    update_timer.stop();
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

    server->updateConfig(config);
}

void DynamicTF2::set(double x, double y, double z, double qx, double qy, double qz, double qw, std::string frame_id, std::string child_frame_id, double period)
{
    update_timer.stop();
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

    server->updateConfig(config);
}

void DynamicTF2::configureCB(dynamic_transform_publisher::TFConfig &config, uint32_t level)
{
    update_timer.stop();
    transform.transform.translation.x = config.x;
    transform.transform.translation.y = config.y;
    transform.transform.translation.z = config.z;

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

    transform.transform.rotation.x = config.qx;
    transform.transform.rotation.y = config.qy;
    transform.transform.rotation.z = config.qz;
    transform.transform.rotation.w = config.qw;
    transform.header.frame_id = config.frame_id;
    transform.child_frame_id = config.child_frame_id;

    update_timer.setPeriod(ros::Duration(config.period/1000));

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
        ROS_INFO_STREAM("Updating transform for "
                         << "\n frame_id: "<<config.frame_id
                         << "\n child_frame_id: "<<config.child_frame_id );
        update_timer.start();
    }
}

void DynamicTF2::update(const ros::TimerEvent &e)
{
    boost::recursive_mutex::scoped_lock lock(config_mutex);
    transform.header.stamp = ros::Time::now();
    br.sendTransform(transform);
}

void DynamicTF2::start()
{
    ReconfigureServer::CallbackType f = boost::bind(&DynamicTF2::configureCB, this, _1, _2);
    server->setCallback(f);
}
