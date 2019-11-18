//
// Created by nannan on 2019/10/21.
//


#include <opencv/cv.hpp>
#include "FeatureUtils.h"
using namespace std;
using namespace cv;
FeatureUtils::FeatureUtils() {
    mDetector = ORB::create(6000);
}


Features FeatureUtils::extractSingleImgKeys(const Mat &image, size_t ImgNo) {

    Features features;
    Mat storeAndShow;
    string title="feature"+to_string(ImgNo)+".jpg";
    mDetector->detectAndCompute(image, noArray(), features.keyPoints, features.descriptors);
    drawKeypoints(image,features.keyPoints,storeAndShow);
    //imshow(title,storeAndShow);
    //waitKey(0);
    imwrite(featureOutputPath+"/"+title,storeAndShow);
    cout<<"The imgs with features have been stored"<<endl;
    KeyPointsToPoints(features.keyPoints, features.points);
    return features;
}

FeaturesVect FeatureUtils::extractMultiImgKeys(ImgsVect aImgsVect) {
    FeaturesVect aFeaturesVect;
    aFeaturesVect.resize(aImgsVect.size());
    for (size_t i = 0; i < aImgsVect.size(); i++) {
        aFeaturesVect[i] = extractSingleImgKeys(aImgsVect[i],i);
//        if (mConsoleDebugLevel <= LOG_DEBUG) {
//            cout << "Image " << i << ": " << mImageFeatures[i].keyPoints.size() << " keypoints" << endl;
//        }
    }
    cout<<"extractMultiImgKeys Success"<<endl;
    return aFeaturesVect;
}


