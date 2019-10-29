/**
 * Aiming at
 */
#ifndef ZARANTSFM_FEATUREUTILS_H
#define ZARANTSFM_FEATUREUTILS_H

#include "SfMCommon.h"
#include <opencv2/features2d.hpp>

class FeatureUtils {

public:
    FeatureUtils();

    Features extractSingleImgKeys(const cv::Mat &image);

    FeaturesVect extractMultiImgKeys(ImgsVect aImgsVect);

//    std::vector<Features> returnMImageFeatures();
private:
    cv::Ptr<cv::Feature2D> mDetector;
};


#endif //ZARANTSFM_FEATUREUTILS_H
