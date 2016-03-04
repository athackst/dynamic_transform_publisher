#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <dynamic_reconfigure/server.h>
#include <dynamic_transform_publisher/TFConfig.h>

namespace tf2_ros
{

class DynamicTransformBroadcaster
{
public:
    DynamicTransformBroadcaster(ros::NodeHandle nh);
    virtual ~DynamicTransformBroadcaster() {}

    /** @brief Initializes the transform to x, y, z roll, pitch, yaw
     *  @note  init(..) needs to be called before start()
     *  @param x The x translation of the tf
     *  @param y The y translation of the tf
     *  @param z The z translation of the tf
     *  @param roll The rotation about x
     *  @param pitch The rotation about y
     *  @param yaw The rotation about z
     *  @param frame_id The parent frame name
     *  @param child_frame_id The name of the transform to publihs
     *  @param period The period (in ms) to send the TransformData
     **/
    void init(double x, double y, double z, double roll, double pitch, double yaw, std::string frame_id, std::string child_frame_id, double period);

    /** @overload
     **/
    void init(double x, double y, double z, double qx, double qy, double qz, double qw, std::string frame_id, std::string child_frame_id, double period);

    /** @brief Starts the dynamic_reconfigure server
     **/
    void start();

    /** @brief Updates the transform/properties of the transform
     **/
    void update(dynamic_transform_publisher::TFConfig &config, uint32_t level=0);

    /** @brief Publishes the transform
     **/
    void send(const ros::TimerEvent &e = ros::TimerEvent());

protected:
    tf2_ros::TransformBroadcaster br;

    boost::recursive_mutex config_mutex;
    typedef dynamic_reconfigure::Server<dynamic_transform_publisher::TFConfig> ReconfigureServer;
    boost::shared_ptr<ReconfigureServer> server;

    ros::Timer update_timer;
    geometry_msgs::TransformStamped transform;
};

} // end namespace tf2_ros
