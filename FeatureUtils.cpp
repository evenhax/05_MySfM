//
// Created by nannan on 2019/10/21.
//


#include "FeatureUtils.h"
using namespace std;
FeatureUtils::FeatureUtils() {
    mDetector = cv::ORB::create(5000);
}


Features FeatureUtils::extractSingleImgKeys(const cv::Mat &image) {

    Features features;
    mDetector->detectAndCompute(image, cv::noArray(), features.keyPoints, features.descriptors);
    KeyPointsToPoints(features.keyPoints, features.points);
    return features;
}

std::vector<Features> FeatureUtils::extractMultiImgKeys(ImgsVect aImgsVect) {
    FeaturesVect aFeaturesVect;
    aFeaturesVect.resize(aImgsVect.size());
    for (size_t i = 0; i < aImgsVect.size(); i++) {
        aFeaturesVect[i] = extractSingleImgKeys(aImgsVect[i]);

//        if (mConsoleDebugLevel <= LOG_DEBUG) {
//            cout << "Image " << i << ": " << mImageFeatures[i].keyPoints.size() << " keypoints" << endl;
//        }
    }
    cout<<"extractMultiImgKeys Success"<<endl;
    return aFeaturesVect;
}


