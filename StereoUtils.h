//
// Created by nannan on 2019/10/21.
//

#ifndef ZARANTSFM_STEREOUTILS_H
#define ZARANTSFM_STEREOUTILS_H

#include "SfMCommon.h"
#include "MatchingUtils.h"
#include <iostream>
#include <exception>

using namespace std;


class StereoUtils {
public:

    StereoUtils();
    /**
     * Find the amount of inlier points in a homography between 2 views.
     * @param left      Left image features
     * @param right     Right image features
     * @param matches   Matching between the features
     * @return number of inliers.
     */
    static int findHomographyInliers(
            const Features &left,
            const Features &right,
            const Matching &matches);

    /**
     * Find camera matrices (3x4 poses) from stereo point matching.
     * @param intrinsics      Camera intrinsics (assuming both cameras have the same parameters)
     * @param featureMatching Matching between left and right features
     * @param featuresLeft    Features in left image
     * @param featuresRight   Features in right image
     * @param prunedMatches   Output: matching after pruning using essential matrix
     * @param Pleft           Output: left image matrix (3x4)
     * @param Pright          Output: right image matrix (3x4)
     * @return true on success.
     */
    static bool findCameraMatricesFromMatch(
            const Intrinsics &intrinsics,
            const Matching &featureMatching,
            const Features &featuresLeft,
            const Features &featuresRight,
            Matching &prunedMatches,
            cv::Matx34f &Pleft,
            cv::Matx34f &Pright);

    /**
     * Triangulate (recover 3D locations) from point matching.
     * @param imagePair     Indices of left and right views
     * @param leftFeatures  Left image features
     * @param rightFeatures Right image features
     * @param Pleft         Left camera matrix
     * @param Pright        Right camera matrix
     * @param pointCloud    Output: point cloud with image associations
     * @return true on success.
     */
    static bool triangulateViews(
            const Intrinsics &intrinsics,
            const ImagePair imagePair,
            const Matching &matches,
            const Features &leftFeatures,
            const Features &rightFeatures,
            const cv::Matx34f &Pleft,
            const cv::Matx34f &Pright,
            PointCloud &pointCloud
    );

    /**
     * Find the camera location based on 2D 3D point correspondence using PnP.
     * @param intrinsics camera intrinsics
     * @param match      a 2d-3d point matching
     * @param cameraPose Output: camera pose
     * @return true on success.
     */
    static bool findCameraPoseFrom2D3DMatch(
            const Intrinsics &intrinsics,
            const Image2D3DMatch &match,
            cv::Matx34f &cameraPose
    );
};


#endif //ZARANTSFM_STEREOUTILS_H
