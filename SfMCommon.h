//
// Created by nannan on 2019/10/28.
//
# include <string>
#include <iostream>
#include <opencv2/core/types.hpp>
#include <opencv2/core/mat.hpp>
#include <map>
#include <opencv2/features2d.hpp>

#ifndef MYSFM02_MYCOMMON_H
#define MYSFM02_MYCOMMON_H

using namespace std;


/**
 * parameters need to be defined by users
 */
const float mDownscaleFactor=1.0;//used by fileutils
const double NN_MATCH_RATIO = 1.0f; // Nearest-neighbour matching ratio,used by matchingutils,1.0f initial
const double RANSAC_THRESHOLD_PNP = 10.0f; // original 10.0f, RANSAC inlier threshold,used in StereoUtils
const double RANSAC_THRESHOLD_H=10.0f;//original 10.0f, RANSAC inlier threshold ,used in StereoUtils,findHomography
const float MIN_REPROJECTION_ERROR = 10.0; // Maximum 10-pixel allowed re-projection error,used in StereoUtils
//0.2 initial.Minimal ratio of inliers-to-total number of points for computing camera pose,used in StereoUtils
const float POSE_INLIERS_MINIMAL_RATIO = 0.15;//0.15 initial
//0.2 initial.Minimal ratio of inliers-to-total number of points for computing camera pose,used in pipe
const float POSE_INLIERS_MINIMAL_RATIO_INITIAL=0.2;
const int findHInliers_match_size=4; //original is 4, used in stereoUtils
const int   MIN_POINT_COUNT_FOR_HOMOGRAPHY         = 100;//used in pipe, 100 original
const float MERGE_CLOUD_POINT_MIN_MATCH_DISTANCE   = 0.01;//used in pipe
const float MERGE_CLOUD_FEATURE_MIN_MATCH_DISTANCE = 20.0;//used in pipe

//E = findEssentialMat(alignedLeft.points, alignedRight.points, focal, pp, cv::RANSAC, 0.999, 1.0, mask); in StereoUtils


const string totalPath="/Users/nannan/CLionProjects/MySfM02/";
const string imageSourcePath=totalPath+"pump02/";
const string featureOutputPath=totalPath+"feature_out/";
const string matchingOutputPath=totalPath+"match_out/";

/**
 * Basic structure and functions
 */
///0. Used in MyFileUtils
typedef std::vector<std::string> ImgNamesVect;
typedef std::vector<cv::Mat> ImgsVect;


///1. Used in Features.h/.cpp

typedef std::vector<cv::KeyPoint> Keypoints;
typedef std::vector<cv::Point2f> Points2f;

struct Features {
    Keypoints keyPoints;
    Points2f points;
    cv::Mat descriptors;
};


typedef std::vector<Features> FeaturesVect;

/**
 * Convert Keypoints to Points2f
 * @param kps keypoints
 * @param ps  points
 */
void KeyPointsToPoints(const Keypoints &kps, Points2f &ps);

/**
 * Convert Points2f to Keypoints.
 * Note: distance on Keypoint will be set to 1.0.
 * @param ps  Points
 * @param kps Keypoints
 */
void PointsToKeyPoints(const Points2f &ps, Keypoints &kps);


///2. Used in MatchingUtils.h/cpp

typedef std::vector<cv::DMatch> Matching;
struct ImagePair {
    size_t left, right;
};

/////////////////cv::Ptr<cv::DescriptorMatcher> mMatcher;
typedef std::vector<std::vector<Matching> > MatchMatrix;


///3. used in StereoUtils
struct Point3DInMap {
    // 3D point.
    cv::Point3f p;

    // A mapping from image index to 2D point index in that image's list of features.
    std::map<int, int> originatingViews;
};
typedef std::vector<cv::Point3f> Points3f;
typedef std::vector<Point3DInMap> PointCloud;
//intrisic parameter of the camera
struct Intrinsics {
    cv::Mat K;
    cv::Mat Kinv;
    cv::Mat distortion;
};

struct Image2D3DMatch {
    Points2f points2D;
    Points3f points3D;
};


///Rotational element in a 3x4 matrix
const cv::Rect ROT(0, 0, 3, 3);

///Translational element in a 3x4 matrix
const cv::Rect TRA(3, 0, 1, 3);

typedef cv::Matx34f Pose;

///4. used for pipe


#endif //MYSFM02_MYCOMMON_H
