#include "convertCoord.h"

convertCoord::convertCoord()
{
    it = new image_transport::ImageTransport(nh);
    pub_converted_image = it->advertise("/moverio/view", 1);
    sub_pose_camera = nh.subscribe("/aruco/pose", 1, &convertCoord::cameraPoseCallback, this);

    cameraPosition = new cv::Mat(4,4,CV_32FC1);
    cameraIntrinsic = new cv::Mat(4,1,CV_32FC1);

    cameraIntrinsic->at<float>(0) = 320;
    cameraIntrinsic->at<float>(1) = 240;
    cameraIntrinsic->at<float>(2) = 1400;
    cameraIntrinsic->at<float>(3) = 1400;

    map = cv::imread("map64751.jpg");
    map.convertTo(map, CV_8UC3);

    loop_rate = new ros::Rate(25);
}

convertCoord::~convertCoord()
{
    delete it;
    delete loop_rate;
}

void convertCoord::cameraPoseCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
    // std::cout << "test" << std::endl;

    double yaw = 0.0, pitch = 0.0, roll = 0.0;
    tf::Quaternion q;
    tf::quaternionMsgToTF(msg->orientation, q);
    tf::Matrix3x3 m(q);
    m.getEulerYPR(yaw, pitch, roll);
    m.setEulerYPR(yaw, 0.0, 0.0);
    double rotation[12];
    m.getOpenGLSubMatrix(rotation);
    cameraPosition->at<float>(0, 0) = rotation[0];
    cameraPosition->at<float>(0, 1) = rotation[1];
    cameraPosition->at<float>(0, 2) = rotation[2];
    cameraPosition->at<float>(1, 0) = rotation[4];
    cameraPosition->at<float>(1, 1) = rotation[5];
    cameraPosition->at<float>(1, 2) = rotation[6];
    cameraPosition->at<float>(2, 0) = rotation[8];
    cameraPosition->at<float>(2, 1) = rotation[9];
    cameraPosition->at<float>(2, 2) = rotation[10];
    cameraPosition->at<float>(0, 3) = 0;
    cameraPosition->at<float>(1, 3) = 0;
    cameraPosition->at<float>(2, 3) = -0.1;//sqrt(msg->position.z*msg->position.z + msg->position.x*msg->position.x + msg->position.y*msg->position.y);

    cv::Mat* image = imagePlane2Globe(*cameraPosition, *cameraIntrinsic, map);
    cv::imshow("a", *image);
    cv::waitKey(0);
    sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", *image).toImageMsg();
    pub_converted_image.publish(img_msg);
    delete image;

    std::cout << q.getX() << " "
    	      << q.getY() << " "
    	      << q.getZ() << " "
    	      << q.w() << std::endl;
    for (int i = 0; i < 12; i++)
    {
    	std::cout << *(rotation+i) << " ";
    }
    std::cout << std::endl;
    std::cout << msg->position.x << " " << msg->position.y << " " << msg->position.z << std::endl;
}

cv::Mat* convertCoord::imagePlane2Globe(cv::Mat& cameraPosition, cv::Mat& cameraIntrinsic, cv::Mat& globeMap)
{
    cv::Mat *image = new cv::Mat(480, 640, CV_8UC3);

    // Get intrinsic and exttric parameters
    float t0 = cameraPosition.at<float>(0, 3);
    float t1 = cameraPosition.at<float>(1, 3);
    float t2 = cameraPosition.at<float>(2, 3);
    float r00 = cameraPosition.at<float>(0, 0);
    float r01 = cameraPosition.at<float>(0, 1);
    float r02 = cameraPosition.at<float>(0, 2);
    float r10 = cameraPosition.at<float>(1, 0);
    float r11 = cameraPosition.at<float>(1, 1);
    float r12 = cameraPosition.at<float>(1, 2);
    float r20 = cameraPosition.at<float>(2, 0);
    float r21 = cameraPosition.at<float>(2, 1);
    float r22 = cameraPosition.at<float>(2, 2);
    float cx = cameraIntrinsic.at<float>(0);
    float cy = cameraIntrinsic.at<float>(1);
    float fx = cameraIntrinsic.at<float>(2);
    float fy = cameraIntrinsic.at<float>(3);
    float r = 0.05f;

    float stepX = 0.f;
    float stepY = 0.f;

    stepX = 360.f / globeMap.cols;
    stepY = 180.f / globeMap.rows;

    // std::cout << t0  << " "
    // 	      << t1  << " "
    // 	      << t2  << " "
    // 	      << r00 << " "
    // 	      << r01 << " "
    // 	      << r02 << " "
    // 	      << r10 << " "
    // 	      << r11 << " "
    // 	      << r12 << " "
    // 	      << r20 << " "
    // 	      << r21 << " "
    // 	      << r22 << " "
    // 	      << cx  << " "
    // 	      << cy  << " "
    // 	      << fx  << " "
    // 	      << fy  << " " << std::endl;


    //Find the projection region - assume the globe with radius 1 and centered at(0, 0, 0)
    //Find the intersection of the globe and the line from camera center passing the image pixel
    for (int uy = 0; uy < image->rows; ++uy) {
        for (int ux = 0; ux < image->cols; ++ux) {
            float XOverZ = (ux - cx) / fx;
            float YOverZ = (uy - cy) / fy;
            // x ^ 2 + y ^ 2 + z ^ 2 = r^2
            // (r00 - XOverZ*r20) * x + (r01 - XOverZ*r21) * y + (r02 - XOverZ*r22) * z = XOverZ*t2 - t0
            // (r10 - YOverZ*r20) * x + (r11 - YOverZ*r21) * y + (r12 - YOverZ*r22) * z = YOverZ*t2 - t1
            // Let them be
            // x ^ 2 + y ^ 2 + z ^ 2 = r^2
            // a00*x + a01*y + a02*z = b0
            // a10*x + a11*y + a12*z = b1
            // x = (b0 - a01*y - a02*z) / a00
            float a00 = r00 - XOverZ * r20;
            float a01 = r01 - XOverZ * r21;
            float a02 = r02 - XOverZ * r22;
            float b0 = XOverZ * t2 - t0;
            float a10 = r10 - YOverZ * r20;
            float a11 = r11 - YOverZ * r21;
            float a12 = r12 - YOverZ * r22;
            float b1 = YOverZ * t2 - t1;
            // simplified, we have
            // (a01^2/a00^2 + 1)*y^2 + (a02^2/a00^2 + 1)*z^2 + 2(a01*a02/a00^2)*y*z - 2(a01*b0/a00^2)*y - 2(a02*b0/a00^2)*z = r^2 - b0^2/a00^2
            // (a11 - a10*a01/a00)*y + (a12 - a10*a02/a00)*z = b1 - a10*b0/a00
            // Let them be
            // c00*y ^ 2 + c01*z ^ 2 + c02*yz + c03*y + c04*z = d0
            // c10*y + c11*z = d1
            // y = (d1 - c11*z) / c10
            float c00 = a01 / a00*a01 / a00 + 1;
            float c01 = a02 / a00*a02 / a00 + 1;
            float c02 = 2 * a01 / a00*a02 / a00;
            float c03 = -2 * a01 / a00*b0 / a00;
            float c04 = -2 * a02 / a00*b0 / a00;
            float d0 = r * r - b0 / a00*b0 / a00;
            float c10 = a11 - a10 / a00*a01;
            float c11 = a12 - a10 / a00*a02;
            float d1 = b1 - a10 / a00*b0;
            // simplified, we have
            // e0*z ^ 2 + e1*z + e2 = 0
            // z = e1 + -sqrt(e1 ^ 2 - 4 * e0*e2) / 2 * e0
            float e0 = c00 / c10*c11 / c10*c11 + c01 - c02 / c10*c11 + c04;
            float e1 = c02*d1 / c10 - 2 * d1 / c10*c11 / c10*c00 + c03 / c10*(d1 - c11);
            float e2 = c00 / c10*d1 / c10*d1 - d0;
            float delta = e1 * e1 - 4 * e0 * e2;
            if (delta < 0) {
                image->at<cv::Vec3b>(uy, ux)[0] = 0;
                image->at<cv::Vec3b>(uy, ux)[1] = 0;
                image->at<cv::Vec3b>(uy, ux)[2] = 0;
            }
            else {
                float z0 = (-e1 + std::sqrt(delta)) / 2 / e0;
                float z1 = (-e1 - std::sqrt(delta)) / 2 / e0;
                float y0 = (d1 - c11*z0) / c10;
                float y1 = (d1 - c11*z1) / c10;
                float x0 = (b0 - a01*y0 - a02*z0) / a00;
                float x1 = (b0 - a01*y1 - a02*z1) / a00;
                float distance0 = (x0 - t0) * (x0 - t0) + (y0 - t1) * (y0 - t1) + (z0 - t2) * (z0 - t2);
                float distance1 = (x1 - t0) * (x1 - t0) + (y1 - t1) * (y1 - t1) + (z1 - t2) * (z1 - t2);
                globeCoord g;
                if (distance0 < distance1){
                    g = card2globe(x0, y0, z0);
                    if (x0*x0 + y0*y0 + z0*z0 < r*r) {
                        image->at<cv::Vec3b>(uy, ux)[0] = 0;
                        image->at<cv::Vec3b>(uy, ux)[1] = 0;
                        image->at<cv::Vec3b>(uy, ux)[2] = 0;
                        continue;
                    }
                }
                else {
                    g = card2globe(x1, y1, z1);
                    if (x1*x1 + y1*y1 + z1*z1 < r*r) {
                        image->at<cv::Vec3b>(uy, ux)[0] = 0;
                        image->at<cv::Vec3b>(uy, ux)[1] = 0;
                        image->at<cv::Vec3b>(uy, ux)[2] = 0;
                        continue;
                    }
                }

    		// Need to check how to get values from the mapping...
    		int y = ceilf((g.signedLat + 90) / stepY);
    		// Make sure it is within boundary
    		y = (y < 0) ? 0 : y;
    		y = (y >= globeMap.rows) ? globeMap.rows - 1 : y;
    		int x = ceilf((g.signedLon + 180) / stepX);
    		x = (x < 0) ? 0 : x;
    		x = (x >= globeMap.cols) ? globeMap.cols - 1 : x;
    		image->at<cv::Vec3b>(uy, ux) = globeMap.at<cv::Vec3b>(y, x);

            }
        }
    }

    return image;
}

globeCoord convertCoord::card2globe(float x, float y, float z) {
    float r = std::sqrt(x * x + y * y + z * z);
    globeCoord gCoord;

    gCoord.isN = z >= 0;
    gCoord.isE = y >= 0;
    gCoord.latitude = asinf(fabs(z) / r) * 180.f / 3.14159f;
    gCoord.longitude = (gCoord.isE ? 1 : -1) * atan2f(y, x) * 180.f / 3.14159f;
    gCoord.signedLat = (gCoord.isN ? 1 : -1) * gCoord.latitude;
    gCoord.signedLon = (gCoord.isE ? 1 : -1) * gCoord.longitude;

    // std::cout << x << " "
    // 	      << y << " "
    // 	      << z << " "
    // 	      << gCoord.isN << " "
    // 	      << gCoord.isE << " "
    // 	      << gCoord.latitude << " "
    // 	      << gCoord.longitude << " "
    // 	      << gCoord.signedLat << " "
    // 	      << gCoord.signedLon << std::endl;
    // int i;
    // std::cin >> i;

    return gCoord;
}
