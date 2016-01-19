/**************************************************************************
** Copyright (c) 2014 United States Government as represented by the
** National Aeronotics and Space Administration.  All Rights Reserved
**
** Author: Allison Thackston
** Created: 28 Oct 2014
**
** Developed jointly by NASA/JSC and Oceaneering Space Systems
**
** Licensed under the NASA Open Source Agreement v1.3 (the "License");
** you may not use this file except in compliance with the License.
** You may obtain a copy of the License at
**
**     http://opensource.org/licenses/NASA-1.3
**
** Unless required by applicable law or agreed to in writing, software
** distributed under the License is distributed on an "AS IS" BASIS,
** WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
** See the License for the specific language governing permissions and
** limitations under the License.
**************************************************************************/
#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <dynamic_reconfigure/server.h>
#include <dynamic_transform_publisher/TFConfig.h>

class DynamicTransformSender
{
public:
    DynamicTransformSender()
        : nh("~")
    {
        tfStamped.resize(1, geometry_msgs::TransformStamped());
    }
    virtual ~DynamicTransformSender() {}

    void set(geometry_msgs::Transform tf_frame, std::string parent_frame, std::string child_frame)
    {
        tfStamped[0].header.stamp = ros::Time::now();
        tfStamped[0].transform = tf_frame;
        tfStamped[0].child_frame_id = child_frame;
        tfStamped[0].header.frame_id = parent_frame;

        br.sendTransform(tfStamped);
    }

    void configureCB(dynamic_transform_publisher::TFConfig &config, uint32_t level)
    {
        std::cout<<"configing"<<std::endl;
        if(config.use_rpy)
        {
            config.groups.rpy.state = true;
            config.groups.quaternion.state = false;
        }
        else
        {
            config.groups.rpy.state = false;
            config.groups.quaternion.state = true;
        }\

    }
/*
    void send(ros::Time time)
    {
        if (parentFrame != "" && childFrame != "")
        {
            br.sendTransform(tf::StampedTransform(transform, time, parentFrame, childFrame));
        }
    }

    bool update(dynamic_transform_publisher::UpdateRequest &request, dynamic_transform_publisher::UpdateResponse &response)
    {
        tf::transformMsgToTF(request.transform, transform);
        parentFrame = request.parent_frame;
        childFrame = request.child_frame;
        return true;
    }

    bool reset(std_srvs::EmptyRequest &request, std_srvs::EmptyResponse &response)
    {
        fromParams();
        return true;
    }

    bool save(std_srvs::EmptyRequest &request, std_srvs::EmptyResponse &response)
    {
        toParams();
        return true;
    }

    void fromParams()
    {
        double x, y, z, qx, qy, qz, qw;
        nh.param<std::string>("frame_id", parentFrame, "");
        ROS_INFO("parent_frame: %s", parentFrame.c_str());
        nh.param<std::string>("child_frame", childFrame, "");
        ROS_INFO("child_frame: %s", childFrame.c_str());
        nh.param<double>("x", x, 0);
        ROS_INFO("x: %f", x);
        nh.param<double>("y", y, 0);
        ROS_INFO("y: %f", y);
        nh.param<double>("z", z, 0);
        ROS_INFO("z: %f", z);
        transform.setOrigin(tf::Vector3(x, y, z));
        nh.param<double>("qx", qx, 0);
        ROS_INFO("qx: %f", qx);
        nh.param<double>("qy", qy, 0);
        ROS_INFO("qy: %f", qy);
        nh.param<double>("qz", qz, 0);
        ROS_INFO("qz: %f", qz);
        nh.param<double>("qw", qw, 1);
        ROS_INFO("qw: %f", qw);
        transform.setRotation(tf::Quaternion(qx, qy, qz, qw));
    }

    void toParams()
    {
        nh.setParam("frame_id", parentFrame);
        nh.setParam("child_frame", childFrame);
        tf::Vector3 origin = transform.getOrigin();
        nh.setParam("x", origin.x());
        nh.setParam("y", origin.y());
        nh.setParam("z", origin.z());

        tf::Quaternion rot = transform.getRotation();
        nh.setParam("qx", rot.getX());
        nh.setParam("qy", rot.getY());
        nh.setParam("qz", rot.getZ());
        nh.setParam("qw", rot.getW());
    }
*/
private:
    ros::NodeHandle nh;
    tf2_ros::StaticTransformBroadcaster br;
    std::vector<geometry_msgs::TransformStamped> tfStamped;

};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "dynamic_tf_publisher");
    ros::NodeHandle nh("~");

    DynamicTransformSender tf_sender;

    dynamic_reconfigure::Server<dynamic_transform_publisher::TFConfig> server;
    dynamic_reconfigure::Server<dynamic_transform_publisher::TFConfig>::CallbackType f;

    f = boost::bind(&DynamicTransformSender::configureCB, &tf_sender, _1, _2);
    server.setCallback(f);
    ros::spin();
/*
    ros::Duration sleeper;

    if(argc == 11)
    {
        sleeper = ros::Duration(atof(argv[10])/1000.0);

        tf::Transform transform = tf::Transform(tf::Quaternion(atof(argv[4]), atof(argv[5]), atof(argv[6]), atof(argv[7])),
                                                tf::Vector3(atof(argv[1]), atof(argv[2]), atof(argv[3])));

        tf_sender.set(transform, argv[8], argv[9]);
    }
    else if(argc == 10)
    {
        sleeper = ros::Duration(atof(argv[9])/1000.0);

        tf::Quaternion q;
        q.setRPY(atof(argv[4]), atof(argv[5]), atof(argv[6]));
        tf::Transform transform = tf::Transform(q, tf::Vector3(atof(argv[1]), atof(argv[2]), atof(argv[3])));
        tf_sender.set(transform, argv[7], argv[8]);
    }
    else if (argc == 1)
    {

        double rate;
        nh.param<double>("rate", rate, 30);
        sleeper = ros::Duration(1/rate);
        tf_sender.fromParams();
    }
    else
    {
        printf("A command line utility for manually sending a transform.\n");
        printf("It will periodicaly republish the given transform. \n");
        printf("Usage: static_transform_publisher x y z yaw pitch roll frame_id child_frame_id  period(milliseconds) \n");
        printf("OR \n");
        printf("Usage: static_transform_publisher x y z qx qy qz qw frame_id child_frame_id  period(milliseconds) \n");
        printf("OR \n");
        printf("Usage: static_transform_publisher \n");
        printf("  with x, y, z, qx, qy, qz, qw, frame_id, child_frame set by parameter. \n");
        printf("\nThis transform is the transform of the coordinate frame from frame_id into the coordinate frame \n");
        printf("of the child_frame_id.  \n");
        ROS_ERROR("static_transform_publisher exited due to not having the right number of arguments");
        return -1;
    }

    while(nh.ok())
    {
        tf_sender.send(ros::Time::now() + sleeper);
        sleeper.sleep();
        ros::spinOnce();
    }
*/
    return 0;
};
