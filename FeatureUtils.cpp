//
// Created by nannan on 2019/10/21.
//


#include "FeatureUtils.h"
using namespace std;

FeatureUtils::FeatureUtils() {
    // initialize detector and extractor
    mDetector = cv::ORB::create(5000);
}

Features FeatureUtils::extractSingleImgKeys(const cv::Mat &image) {
    Features features;
    mDetector->detectAndCompute(image, cv::noArray(), features.keyPoints, features.descriptors);
    KeyPointsToPoints(features.keyPoints, features.points);
    return features;
}

void FeatureUtils::extractMultiImgKeys(std::vector<cv::Mat> mImages_) {
    mImageFeatures.resize(mImages_.size());
    for (size_t i = 0; i < mImages_.size(); i++) {
        mImageFeatures[i] = extractSingleImgKeys(mImages_[i]);

//        if (mConsoleDebugLevel <= LOG_DEBUG) {
//            cout << "Image " << i << ": " << mImageFeatures[i].keyPoints.size() << " keypoints" << endl;
//        }
    }
    cout<<"extractMultiImgKeys Success"<<endl;
}

std::vector<Features> FeatureUtils::returnMImageFeatures() {
    return mImageFeatures;
}
