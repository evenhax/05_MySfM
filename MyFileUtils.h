//
// Created by nannan on 2019/10/20.
//

#ifndef ZARANTSFM_MYFILEUTILS_H
#define ZARANTSFM_MYFILEUTILS_H

#include "SfMCommon.h"
#include <string>
#include <boost/filesystem.hpp>
#include <iostream>
#include <boost/algorithm/string.hpp>
#include <MacTypes.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <map>
#include <chrono>

using namespace cv;
using namespace std;

class MyFileUtils {
public:
    /**
     * Set the directory with images to perform the SfM operation on.
     * Image file with extensions "jpg" and "png" will be used.
     * @return true on success.
     */
    bool setImagesDirectory(const std::string &directoryPath);

    std::vector<cv::Mat> returnImgs();

    int returnNumImgs();
private:
    std::vector<std::string> mImageFilenames;
    std::vector<cv::Mat> mImages;



};


#endif //ZARANTSFM_MYFILEUTILS_H
