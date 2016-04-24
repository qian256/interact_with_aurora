#include <ArucoDetector.h>

#include <string>
#include <vector>

#include <opencv2/core/core.hpp>

#include <ros/ros.h>
#include <ros/assert.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Pose.h>

#include <aruco/aruco.h>

namespace std
{
string to_string(const string& s) {
    return s;
}
}

namespace ros_aruco
{

ArucoDetector::ArucoDetector()
    : nh_("~"), it_(nh_)
{
    marker_size_ = 0.1;
    detected_ = false;

    nh_.param("target_encoding", target_encoding_, std::string());

    // Subscribe to input video feed and publish output video feed
    camera_info_sub_ = nh_.subscribe("/updated/camera_info", 10,
                                     &ArucoDetector::cameraInfoCb, this);
    image_sub_ = it_.subscribe("/image_raw", 10,
                               &ArucoDetector::detectorCb, this);
    pose_pub_ = nh_.advertise<geometry_msgs::Pose>("/aruco/pose", 100);
}


bool ArucoDetector::setCameraParameters(const cv::Mat& cameraMatrix,
                                        const cv::Mat& distortionCoefficients,
                                        const cv::Size& size)
{
    if (distortionCoefficients.size().area() != 5
            || cameraMatrix.size().area() != 9)
        return false;
    try
    {
        camera_parameters_ = aruco::CameraParameters(cameraMatrix, distortionCoefficients, size);
    }
    catch (cv::Exception e)
    {
        return false;
    }

    camera_parameters_set_ = true;
    ROS_INFO("Camera parameters are set!");
    return true;
}


std::vector<aruco::Marker> ArucoDetector::detect(cv::Mat image)
{
    std::vector<aruco::Marker> markers;
    if (camera_parameters_set_) {
        marker_detector_.detect(image, markers, camera_parameters_, marker_size_);
    }
    return markers;
}


void ArucoDetector::cameraInfoCb(const sensor_msgs::CameraInfoConstPtr& msg)
{
    cv::Mat cameraMatrix(cv::Size(3,3), CV_64F, (void*) msg->K.elems);
    cv::Mat distortionCoefficients(msg->D, true);
    cv::Size size(msg->width, msg->height);
    ROS_ASSERT_MSG(msg->distortion_model == "plumb_bob",
                   "Only cameras with a 'plumb_bob' distortion model are supported");
    if (setCameraParameters(cameraMatrix, distortionCoefficients, size))
        camera_info_sub_.shutdown();
}


void ArucoDetector::detectorCb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, target_encoding_);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    geometry_msgs::Pose pose_msg;

    std::vector<aruco::Marker> detected_markers = detect(cv_ptr->image);
    if (detected_markers.empty()){
        detected_ = false;

        pose_msg.position.x = 0;
        pose_msg.position.y = 0;
        pose_msg.position.z = 0;

        pose_msg.orientation.w = 0;
        pose_msg.orientation.x = 0;
        pose_msg.orientation.y = 0;
        pose_msg.orientation.z = 0;

    }
    else{
        detected_ = true;
        std::cout << "=================================" << std::endl;
        for (aruco::Marker m : detected_markers) {
            m.calculateExtrinsics(marker_size_, camera_parameters_);
            double position[3];
            double orientation[4];

            m.OgreGetPoseParameters(position, orientation);
            double modelview[16];

            m.glGetModelViewMatrix(modelview);

            std::cout << "ModelView:" << std::endl;
            std::cout << "[[" << modelview[0] << ", " << modelview[4] << ", " << modelview[8] << ", " << modelview[12] << "]," << std::endl
                    << "["  << modelview[1] << ", " << modelview[5] << ", " << modelview[9] << ", " << modelview[13] << "]," << std::endl
                    << "["  << modelview[2] << ", " << modelview[6] << ", " << modelview[10] << ", " << modelview[14] << "]," << std::endl
                    << "[" << modelview[3] << ", " << modelview[7] << ", " << modelview[11] << ", " << modelview[15] << "]]" << std::endl;

            std::cout << "Position:" << std::endl;
            std::cout << position[0] << ", " << position[1] << ", " << position[2] << std::endl;

            std::cout << "Quaternion:" << std::endl;
            std::cout << orientation[0] << ", " << orientation[1] << ", " << orientation[2] << ", " << orientation[3] << std::endl;

            pose_msg.position.x = position[0];
            pose_msg.position.y = position[1];
            pose_msg.position.z = position[2];

            pose_msg.orientation.w = orientation[0];
            pose_msg.orientation.x = orientation[1];
            pose_msg.orientation.y = orientation[2];
            pose_msg.orientation.z = orientation[3];
        }
        pose_pub_.publish(pose_msg);
    }
}

} // namespace ros_aruco


int main(int argc, char** argv)
{
    ros::init(argc, argv, "aruco_detector");
    ros_aruco::ArucoDetector ic;
    ros::spin();
    return EXIT_SUCCESS;
}
