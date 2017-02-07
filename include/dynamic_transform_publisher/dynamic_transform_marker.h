#ifndef DYNAMIC_MARKER_CONTROL_H_
#define DYNAMIC_MARKER_CONTROL_H_

#include <deque>
#include <dynamic_transform_publisher/TFConfig.h>
#include <dynamic_reconfigure/server.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

class DynamicMarkerControl {
private:
    interactive_markers::InteractiveMarkerServer server_;
    visualization_msgs::InteractiveMarker marker_;

    typedef dynamic_reconfigure::Server<dynamic_transform_publisher::TFConfig> ReconfigureServer;
    boost::shared_ptr<ReconfigureServer> reconfigure_server_;

    tf::Transform current_pose_;

    std::deque<tf::Transform> pose_history_;

public:
    DynamicMarkerControl(const boost::shared_ptr<ReconfigureServer> &server, const dynamic_transform_publisher::TFConfig &config);

    virtual ~DynamicMarkerControl()  {}

    void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    void addToHistory(tf::Transform pose);
    void tfToConfig(const tf::Transform &pose, dynamic_transform_publisher::TFConfig &config);

    dynamic_transform_publisher::TFConfig config_;
};

#endif // DYNAMIC_MARKER_CONTROL_H_
