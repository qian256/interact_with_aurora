#pragma once

namespace cv {
    class Mat;
}

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

    cv::Mat* imagePlane2Globe(cv::Mat& cameraPosition, cv::Mat& cameraIntrinsic, cv::Mat* globeMap = nullptr);

private:
    globeCoord card2globe(float x, float y, float z);
};

