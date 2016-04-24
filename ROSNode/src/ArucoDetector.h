#ifndef ROS_ARUCO_ARUCODETECTOR_H
#define ROS_ARUCO_ARUCODETECTOR_H

#include <string>
#include <vector>

#include <opencv2/core/core.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>

#include <aruco/aruco.h>

namespace ros_aruco
{
class ArucoDetector
{
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Subscriber camera_info_sub_;
    ros::Publisher pose_pub_;
    std::string target_encoding_;

    aruco::MarkerDetector marker_detector_;
    aruco::CameraParameters camera_parameters_;
    bool camera_parameters_set_;
    float marker_size_;
    bool detected_;

public:
    ArucoDetector();
    std::vector<aruco::Marker> detect(const cv::Mat image);
    bool setCameraParameters(const cv::Mat& cameraMatrix,
                             const cv::Mat& distortionCoefficients, const cv::Size& size);

    void detectorCb(const sensor_msgs::ImageConstPtr& msg);
    void cameraInfoCb(const sensor_msgs::CameraInfoConstPtr& msg);
};
}
#endif
