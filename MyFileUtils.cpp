//
// Created by nannan on 2019/10/20.
//
#include "MyFileUtils.h"
#include "SfMCommon.h"

using namespace boost::filesystem;
using namespace std;

bool MyFileUtils::setImagesDirectory(const std::string &directoryPath) {

    //define the path and ensure its existence
    path dirPath(directoryPath);
    if (not exists(dirPath) or not is_directory(dirPath)) {
        cerr << "Cannot open directory: " << directoryPath << endl;
        return false;
    }

    //fetch all imgs with excepted extension and store their path in mImageFilenames
    for (directory_entry &x : directory_iterator(dirPath)) {
        std::string extension = x.path().extension().string();
        boost::algorithm::to_lower(extension);
        if (extension == ".jpg" or extension == ".png") {
            mImageFilenames.push_back(x.path().string());
        }
    }

    if (mImageFilenames.size() <= 0) {
        cerr << "Unable to find valid files in images directory (\"" << directoryPath << "\")." << endl;
        return false;
    }

//    if (mConsoleDebugLevel <= LOG_DEBUG) {
//        cout << "Found " << mImageFilenames.size() << " image files in directory." << endl;
//    }

//read all images (default downscale factor=1,i.e.,keep the original size) into mImages
    for (auto &imageFilename : mImageFilenames) {
        mImages.push_back(cv::imread(imageFilename));

        if (mDownscaleFactor != 1.0) {
            cv::resize(mImages.back(), mImages.back(), cv::Size(), mDownscaleFactor, mDownscaleFactor);
        }

        if (mImages.back().empty()) {
            cerr << "Unable to read image from file: " << imageFilename << endl;
            return false;
        }
    }

    return true;
}

std::vector<cv::Mat> MyFileUtils::returnImgs() {
    return mImages;
}

int MyFileUtils::returnNumImgs() {

    int i = mImages.size();
    return i;
}