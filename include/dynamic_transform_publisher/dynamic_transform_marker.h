#ifndef DYNAMIC_MARKER_CONTROL_H_
#define DYNAMIC_MARKER_CONTROL_H_

#include <deque>

#include <ros/ros.h>

#include <dynamic_transform_publisher/TFConfig.h>
#include <dynamic_reconfigure/server.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class DynamicMarkerControl {
private:
    interactive_markers::InteractiveMarkerServer server_;
    visualization_msgs::InteractiveMarker marker_;

    typedef boost::function<void(const dynamic_transform_publisher::TFConfig &)> CallbackType;
    CallbackType callback_;

    tf2::Transform current_pose_;

    std::deque<tf2::Transform> pose_history_;

public:
    DynamicMarkerControl(const CallbackType &callback, const dynamic_transform_publisher::TFConfig &init_config);

    virtual ~DynamicMarkerControl()  {}

    void updatePose(const geometry_msgs::Pose &pose);

    void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    void addToHistory(tf2::Transform pose);
    void tfToConfig(const tf2::Transform &pose, dynamic_transform_publisher::TFConfig &config);

    dynamic_transform_publisher::TFConfig config_;
};

#endif // DYNAMIC_MARKER_CONTROL_H_
