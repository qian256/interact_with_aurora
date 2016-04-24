#ifndef _convertCoord_h
#define _convertCoord_h

// input output stream library
#include <iostream>
// ros library
#include <ros/ros.h>
// Pose
#include <geometry_msgs/Pose.h>
// image transport
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
// opencv library
#include <opencv2/opencv.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
// stl vector
#include <vector>
// math
#include <math.h>
// tf
#include <tf/transform_datatypes.h>

struct globeCoord {
    bool isN, isE;
    float latitude, longitude;
    float signedLat, signedLon;
};

class convertCoord
{
public:
    convertCoord();
    ~convertCoord();
    ros::Rate* loop_rate;

private:
    ros::NodeHandle nh;
    image_transport::ImageTransport *it;
    image_transport::Publisher pub_converted_image;
    ros::Subscriber sub_pose_camera;

    cv::Mat map;
    cv::Mat* cameraPosition;
    cv::Mat* cameraIntrinsic;

    cv::Mat* imagePlane2Globe(cv::Mat& cameraPosition, cv::Mat& cameraIntrinsic, cv::Mat& globeMap);

    void cameraPoseCallback(const geometry_msgs::Pose::ConstPtr& msg);

    globeCoord card2globe(float x, float y, float z);
};

#endif // _convertCoord_h

