#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <dynamic_reconfigure/server.h>
#include <dynamic_transform_publisher/TFConfig.h>

class DynamicTF2
{
public:
    DynamicTF2(ros::NodeHandle nh);
    virtual ~DynamicTF2() {}

    void set(double x, double y, double z, double roll, double pitch, double yaw, std::string frame_id, std::string child_frame_id, double period);

    void set(double x, double y, double z, double qx, double qy, double qz, double qw, std::string frame_id, std::string child_frame_id, double period);

    void configureCB(dynamic_transform_publisher::TFConfig &config, uint32_t level);

    void update(const ros::TimerEvent &e);

    void start();

private:
    tf2_ros::TransformBroadcaster br;

    boost::recursive_mutex config_mutex;
    typedef dynamic_reconfigure::Server<dynamic_transform_publisher::TFConfig> ReconfigureServer;
    boost::shared_ptr<ReconfigureServer> server;

    ros::Timer update_timer;
    geometry_msgs::TransformStamped transform;
};
