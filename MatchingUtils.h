//
// Created by nannan on 2019/10/21.
//

#ifndef ZARANTSFM_MATCHINGUTILS_H
#define ZARANTSFM_MATCHINGUTILS_H

#include "SfMCommon.h"
#include "FeatureUtils.h"

class MatchingUtils {



public:

    /**
 * Get the features for left and right images after keeping only the matched features and aligning them.
 * Alignment: i-th feature in left is a match to i-th feature in right.
 * @param leftFeatures       Left image features.
 * @param rightFeatures      Right image features.
 * @param matches            Matching over the features.
 * @param alignedLeft        Output: aligned left features.
 * @param alignedRight       Output: aligned right features.
 * @param leftBackReference  Output: back reference from aligned index to original index
 * @param rightBackReference Output: back reference from aligned index to original index
 */
    static void GetAlignedPointsFromMatch(const Features &leftFeatures,
                                                 const Features &rightFeatures,
                                                 const Matching &matches,
                                                 Features &alignedLeft,
                                                 Features &alignedRight,
                                                 std::vector<int> &leftBackReference,
                                                 std::vector<int> &rightBackReference);

/**
 * Get the features for left and right images after keeping only the matched features and aligning them.
 * Alignment: i-th feature in left is a match to i-th feature in right.
 * @param leftFeatures  Left image features.
 * @param rightFeatures Right image features.
 * @param matches       Matching over the features.
 * @param alignedLeft   Output: aligned left features.
 * @param alignedRight  Output: aligned right features.
 */
    static void GetAlignedPointsFromMatch(const Features &leftFeatures,
                                          const Features &rightFeatures,
                                          const Matching &matches,
                                          Features &alignedLeft,
                                          Features &alignedRight);

/**
 * Get a Matching for an aligned set: i -> i
 * @param size size of maching vector
 * @return aligned matching.
 */
    Matching GetAlignedMatching(size_t size);

    static Matching matchFeatures(
            const Features &featuresLeft,
            const Features &featuresRight);

    void createFeatureMatchMatrix(int imgNum_, std::vector<Features> mImageFeatures_);


private:


    cv::Ptr<cv::DescriptorMatcher> mMatcher;
    typedef std::vector<std::vector<Matching> > MatchMatrix;
    MatchMatrix mFeatureMatchMatrix;

};


#endif //ZARANTSFM_MATCHINGUTILS_H
