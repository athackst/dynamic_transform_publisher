#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf/tf.h>
#include <dynamic_reconfigure/server.h>
#include <dynamic_transform_publisher/TFConfig.h>

class DynamicTransformPublisher
{
public:
    DynamicTransformPublisher() {}
    virtual ~DynamicTransformPublisher() {}

    void configureCB(dynamic_transform_publisher::TFConfig &config, uint32_t level)
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

        if(config.frame_id != "" && config.child_frame_id != "" && config.frame_id != config.child_frame_id)
        {
            tf2_ros::StaticTransformBroadcaster br;
            std::vector<geometry_msgs::TransformStamped> transforms;
            transforms.push_back(t);
            br.sendTransform(transforms);
        }
    }

private:
    tf2_ros::StaticTransformBroadcaster br;

};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "dynamic_transform_publisher");
    ros::NodeHandle nh("~");

    DynamicTransformPublisher tf_sender;

    dynamic_reconfigure::Server<dynamic_transform_publisher::TFConfig> server;
    dynamic_reconfigure::Server<dynamic_transform_publisher::TFConfig>::CallbackType f;

    f = boost::bind(&DynamicTransformPublisher::configureCB, &tf_sender, _1, _2);
    server.setCallback(f);
    ros::spin();

    return 0;
};
