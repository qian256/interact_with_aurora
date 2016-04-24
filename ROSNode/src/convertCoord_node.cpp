// ros library
#include <ros/ros.h>
// my controller header file
#include "convertCoord.h"
// output to file
#include <fstream>

// #include <opencv2\core\core.hpp>
// #include <opencv2\imgproc\imgproc.hpp>
// #include <opencv2\highgui\highgui.hpp>


/*ux = 127
uy = 127
cx = 127
cy = 127
fx = 1
fy = 1
r00 = 1
r01 = 0
r02 = 0
r10 = 0
r11 = 1
r12 = 0
r20 = 0
r21 = 0
r22 = 1
t0 = 0
t1 = 0
t2 = -2*/
#define TEST

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "convertCoord_node");

    convertCoord converter;

    ros::spin();
    converter.loop_rate->sleep();

    return 0;
}

