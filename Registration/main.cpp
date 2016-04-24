#include "convertCoord.h"
#include <opencv2\core\core.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <iostream>
#include <fstream>
#include <vector>

#define TEST
std::vector<std::string> files = {
    "64754"
    /*"64751", "64752", "64754", "64755", "64756", "64757", 
    "64758", "64759", "64760", "64761", "64762", "64763",
    "64764", "64765", "64766"*/
};

//#define SHIFT_LATS
#define SHIFT_LONS
//#define ROTATE

#define GENDATA

int main() {
    cv::Mat cameraPosition(4,4,CV_32FC1);
    cv::Mat cameraIntrinsic(4,1,CV_32FC1);

    std::ifstream iFile("D:\\Dropbox\\NASA2016\\matrix.txt");
#ifdef GENDATA
    for (int i = 0; i < 361; ++i) {

        iFile >>
            cameraPosition.at<float>(0, 0) >>
            cameraPosition.at<float>(0, 1) >>
            cameraPosition.at<float>(0, 2) >>
            cameraPosition.at<float>(1, 0) >>
            cameraPosition.at<float>(1, 1) >>
            cameraPosition.at<float>(1, 2) >>
            cameraPosition.at<float>(2, 0) >>
            cameraPosition.at<float>(2, 1) >>
            cameraPosition.at<float>(2, 2);
        /*iFile >>
            cameraPosition.at<float>(0, 0) >>
            cameraPosition.at<float>(1, 0) >>
            cameraPosition.at<float>(2, 0) >>
            cameraPosition.at<float>(0, 1) >>
            cameraPosition.at<float>(1, 1) >>
            cameraPosition.at<float>(2, 1) >>
            cameraPosition.at<float>(0, 2) >>
            cameraPosition.at<float>(1, 2) >>
            cameraPosition.at<float>(2, 2);*/
        std::string dummy;
        std::getline(iFile, dummy);
#else
        cameraPosition.at<float>(0, 0) = 1.f;
        cameraPosition.at<float>(0, 1) = 0.f;
        cameraPosition.at<float>(0, 2) = 0.f;
        cameraPosition.at<float>(1, 0) = 0.f;
        cameraPosition.at<float>(1, 1) = 1.f;
        cameraPosition.at<float>(1, 2) = 0.f;
        cameraPosition.at<float>(2, 0) = 0.f;
        cameraPosition.at<float>(2, 1) = 0.f;
        cameraPosition.at<float>(2, 2) = 1.f;
        /*cameraPosition.at<float>(0, 0) = -1.f;
        cameraPosition.at<float>(0, 1) = 0.f;
        cameraPosition.at<float>(0, 2) = 0.;
        cameraPosition.at<float>(1, 0) = 0.f;
        cameraPosition.at<float>(1, 1) = -1.f;
        cameraPosition.at<float>(1, 2) = 0.f;
        cameraPosition.at<float>(2, 0) = 0.f;
        cameraPosition.at<float>(2, 1) = 0.f;
        cameraPosition.at<float>(2, 2) = 1.f;*/
#endif
        cameraPosition.at<float>(0, 3) = 0.f;
        cameraPosition.at<float>(1, 3) = 0.f;
        cameraPosition.at<float>(2, 3) = 0.2f;

        //[[-0.0261007, 0.0433044, 0.998721, -0.0123315],
        //    [-0.542355, -0.839855, 0.022242, 0.0330893],
        //    [-0.839744, 0.541081, -0.0454072, -0.349108],
        //    [0, 0, 0, 1]]


        cameraIntrinsic.at<float>(0) = 320;
        cameraIntrinsic.at<float>(1) = 240;
        cameraIntrinsic.at<float>(2) = 1400;
        cameraIntrinsic.at<float>(3) = 1400;

        convertCoord converter;
        //cv::Mat* image = converter.imagePlane2Globe(cameraPosition, cameraIntrinsic);

        //// Display all values
        //for (int y = 0; y < image->cols; ++y) {
        //    for (int x = 0; x < image->rows; ++x) {
        //        std::cout << "(" << y << ", " << x << "): " << image->at<cv::Vec3f>(y, x)[0] << ", " << image->at<cv::Vec3f>(y, x)[1] << ", " << image->at<cv::Vec3f>(y, x)[2] << std::endl;
        //    }
        //    std::cout << std::endl;
        //}
#ifdef TEST
        //cv::Mat input = cv::imread("D:\\Dropbox\\NASA2016\\coordCoord\\lena_color.jpg");
        //cv::Mat input = cv::imread("D:\\Dropbox\\NASA2016\\coordCoord\\cropped_image.JPG");
        for (auto fi : files) {

            cv::Mat input = cv::imread("D:\\Dropbox\\NASA2016\\coordCoord\\map"+fi+".jpg");
            input.convertTo(input, CV_8UC3);

            // 414x816 - create map
            cv::Mat map(input.rows, input.cols, CV_8UC3);

            for (int y = 0; y < input.rows; ++y) {
                for (int x = 0; x < input.cols; ++x) {
                    map.at<cv::Vec3b>(y, x) = input.at<cv::Vec3b>(y, x);
                }
            }

            cv::Mat* image2 = converter.imagePlane2Globe(cameraPosition, cameraIntrinsic, &map);

            //cv::imshow("myimage", image2->t());
            //cv::waitKey(0);
#ifdef GENDATA
            cv::imwrite("D:\\Dropbox\\NASA2016\\coordCoord\\mapResult" + fi + "_" + std::to_string(i) + ".jpg", image2->t());
#else
            cv::imwrite("D:\\Dropbox\\NASA2016\\coordCoord\\mapResult" + fi + ".jpg", image2->t());
#endif
        }
#else
        for (auto fi : files) {
            std::ifstream iRGBFile("E:\\img_lat_lon\\img" + fi);
            std::ifstream iLatsFile("E:\\img_lat_lon\\lats" + fi);
            std::ifstream iLonsFile("E:\\img_lat_lon\\lons" + fi);
            std::vector<cv::Vec3d> RGBValues;
            std::vector<float> latsValues, lonsValues;
#ifdef SHIFT_LATS
            float maxLatsValue = -90.f, minLatsValue = 90.f;
#endif
#ifdef SHIFT_LONS
            float maxLonsValue = -180.f, minLonsValue = 180.f;
#endif
            if (iRGBFile.good() && iLatsFile.good() && iLonsFile.good()) {
                while (!iRGBFile.eof()) {
                    std::string R, G, B, Lat, Lon;
                    std::getline(iRGBFile, R, ',');
                    std::getline(iRGBFile, G, ',');
                    std::getline(iRGBFile, B, ',');
                    std::getline(iLatsFile, Lat, ',');
                    std::getline(iLonsFile, Lon, ',');
                    //std::cout << R << ", " << G << ", " << B << ", " << Lat << ", " << Lon << "." << std::endl;
                    cv::Vec3d temp;
                    temp[0] = std::atoi(R.c_str());
                    temp[1] = std::atoi(G.c_str());
                    temp[2] = std::atoi(B.c_str());
                    RGBValues.push_back(temp);
                    latsValues.push_back(std::atof(Lat.c_str()));
#ifdef SHIFT_LATS
                    if (latsValues.back() > maxLatsValue)
                        maxLatsValue = latsValues.back();
                    if (latsValues.back() < minLatsValue)
                        minLatsValue = latsValues.back();
#endif
                    lonsValues.push_back(std::atof(Lon.c_str()));
#ifdef SHIFT_LONS
                    if (lonsValues.back() > maxLonsValue)
                        maxLonsValue = lonsValues.back();
                    if (lonsValues.back() < minLonsValue)
                        minLonsValue = lonsValues.back();
#endif
                }
            }
            else {
                exit(1);
            }
#ifdef SHIFT_LATS
            float halfLatsSize = std::abs(maxLatsValue - minLatsValue) / 2.f;
#endif
#ifdef SHIFT_LONS
            float halfLonsSize = std::abs(maxLonsValue - minLonsValue) / 2.f;
#endif
            cv::Mat map = cv::imread("D:\\Dropbox\\NASA2016\\coordCoord\\oy8Od82.jpg");
            map.convertTo(map, CV_8UC3);
            //cv::resize(map, map, cv::Size(), 6, 6);
            // create cv::Mat map from input data
            //cv::Mat map(480, 640, CV_8UC3);

            float stepX = 361.f / map.cols;
            float stepY = 181.f / map.rows;

            for (unsigned int i = 0; i < RGBValues.size(); ++i) {
                float lats = latsValues[i];
                float lons = lonsValues[i];
#ifdef ROTATE
                float angle = -23.f / 180.f * 3.14f;
                /*float nLats = std::cosf(angle) * lats - std::sinf(angle) * lons;
                float nLons = std::sinf(angle) * lats + std::cosf(angle) * lons;*/
                float nLats = std::sinf(angle) * lons + std::cosf(angle) * lats;
                float nLons = std::cosf(angle) * lons - std::sinf(angle) * lats;
                lats = nLats;
                lons = nLons;
                if (lats > 90.) lats -= 90.f;
                if (lats < -90.f) lats += 90.f;
                if (lons > 180.f) lons -= 180.f;
                if (lons < -180.f) lons += 180.f;
#endif
#ifdef SHIFT_LATS
                lats += halfLatsSize;
                if (lats > maxLatsValue)
                    lats -= 2.f * halfLatsSize;
                if (lats > 90.) lats -= 90.f;
                if (lats < -90.f) lats += 90.f;
#endif
#ifdef SHIFT_LONS
                lons += halfLonsSize;
                if (lons > maxLonsValue)
                    lons -= 2.0f*halfLonsSize;
                if (lons > 180.f) lons -= 180.f;
                if (lons < -180.f) lons += 180.f;
#endif

                cv::Vec3d temp = RGBValues[i];
                if (temp[0] == 0 && temp[1] == 0 && temp[2] == 0) continue;
                int y = ceil((lats + 90) / stepY);
                // Make sure it is within boundary
                y = (y < 0) ? 0 : y;
                y = (y >= map.rows) ? map.rows - 1 : y;
                int x = ceil((lons + 180) / stepX);
                x = (x < 0) ? 0 : x;
                x = (x >= map.cols) ? map.cols - 1 : x;
                map.at<cv::Vec3b>(y, x) = temp;
            }

            // Erosion???

            //cv::imshow("myimage", map);
            //cv::waitKey(0);

            cv::imwrite("D:\\Dropbox\\NASA2016\\coordCoord\\map" + fi + ".jpg", map);
        }
#endif
#ifdef GENDATA
    }
#endif
}