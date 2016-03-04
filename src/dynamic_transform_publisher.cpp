#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <dynamic_transform_publisher/dynamic_transform_broadcaster.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dynamic_transform_publisher", ros::init_options::AnonymousName);
    ros::NodeHandle nh("~");

    tf2_ros::DynamicTransformBroadcaster tf_sender(nh);

    if(argc == 11)
    {
      if (strcmp(argv[8], argv[9]) == 0)
        ROS_FATAL("target_frame and source frame are the same (%s, %s) this cannot work", argv[8], argv[9]);

      tf_sender.init(atof(argv[1]), atof(argv[2]), atof(argv[3]), atof(argv[4]), atof(argv[5]), atof(argv[6]), atof(argv[7]), argv[8], argv[9], atof(argv[10]));
    }
    else if (argc == 10)
    {
      if (strcmp(argv[7], argv[8]) == 0)
        ROS_FATAL("target_frame and source frame are the same (%s, %s) this cannot work", argv[7], argv[8]);

      tf_sender.init(atof(argv[1]), atof(argv[2]), atof(argv[3]), atof(argv[4]), atof(argv[5]), atof(argv[6]), argv[7], argv[8], atof(argv[9]));
    }
    else if(argc==1)
    {
        ROS_INFO("using parameters");
    }
    else
    {
        printf("A command line utility for manually sending a transform.\n");
        printf("It will periodicaly republish the given transform. \n");
        printf("This transform is the transform of the coordinate frame from frame_id into the coordinate frame \n");
        printf("of the child_frame_id.\n");
        printf("If no arguments are given, the publisher will use the parameter server. \n\n");
        printf("Usage: dynamic_transform_publisher x y z yaw pitch roll frame_id child_frame_id  period(milliseconds) \n");
        printf("OR \n");
        printf("Usage: dynamic_transform_publisher x y z qx qy qz qw frame_id child_frame_id  period(milliseconds) \n");
        printf("OR \n");
        printf("Usage: dynamic_transform_publisher\n");

        ROS_ERROR("dynamic_transform_publisher exited due to not having the right number of arguments");
        return -1;
    }

    tf_sender.start();

    ros::spin();

    return 0;
};
