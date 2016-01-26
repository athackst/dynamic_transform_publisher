#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf/tf.h>
#include <dynamic_reconfigure/server.h>
#include <dynamic_transform_publisher/TFConfig.h>

class DynamicTransform
{
public:
    DynamicTransform(ros::NodeHandle nh);
    virtual ~DynamicTransform() {}

    void set(double x, double y, double z, double roll, double pitch, double yaw, std::string frame_id, std::string child_frame_id);

    void set(double x, double y, double z, double qx, double qy, double qz, double qw, std::string frame_id, std::string child_frame_id);

    void configureCB(dynamic_transform_publisher::TFConfig &config, uint32_t level);

private:
    tf2_ros::StaticTransformBroadcaster br;

    boost::recursive_mutex config_mutex;
    typedef dynamic_reconfigure::Server<dynamic_transform_publisher::TFConfig> ReconfigureServer;
    boost::shared_ptr<ReconfigureServer> server;

};
