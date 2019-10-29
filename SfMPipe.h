//
// Created by nannan on 2019/10/21.
//

#ifndef ZARANTSFM_SFMPIPE_H
#define ZARANTSFM_SFMPIPE_H

#include "SfMCommon.h"
#include "MyFileUtils.h"
#include "FeatureUtils.h"
#include "MatchingUtils.h"
#include "StereoUtils.h"
#include "BAUtils.h"

#include <opencv2/features2d.hpp>

#include <string>
#include <vector>
#include <map>
#include <set>

enum ErrorCode {
    OKAY = 0,
    ERROR
};

class SfMPipe {

    typedef std::map<int, Image2D3DMatch> Images2D3DMatches;

public:
    SfMPipe();

    bool mySfMSet();

    /**
 * This is the main function of this class. Start here.
 * Run the SfM operation:
 *  - Extract and match image features.
 *  - Find a baseline triangulation.
 *  - Sequentially add more views to the cloud.
 * @return error code.
 */
    ErrorCode runSfM();

    void saveCloudAndCamerasToPLY(const std::string &filename);


    /**
     * Find the best two views and perform an initial triangulation from their feature matching.
     */
    void findBaselineTriangulation();

    /**
     * Run a bundle adjuster on the current reconstruction.
     */
    void adjustCurrentBundle();

    /**
     * Sort the image pairs for the initial baseline triangulation based on the number of homography-inliers
     * @return scoring of views-pairs
     */
    std::map<float, ImagePair> sortViewsForBaseline();

    /**
     * Add more views from the set to the 3D point cloud
     */
    void addMoreViewsToReconstruction();

    /**
     * For all remaining images to process, find the set of 2D points that correlate to 3D points in the current cloud.
     * This is done by scanning the 3D cloud and checking the originating 2D views of each 3D point to see if they
     * match 2D features in the new views.
     * @return 2D-3D matching from the image features to the cloud
     */
    Images2D3DMatches find2D3DMatches();

    /**
     * Merge the given point cloud into the existing reconstruction, by merging 3D points from multiple views.
     * @param cloud to merge
     * @return number of new points added
     */
    void mergeNewPointCloud(const PointCloud &cloud);

private:

    MyFileUtils myFileTool;
    FeatureUtils myFeatureTool;
    MatchingUtils myMatchTool;
    StereoUtils myStereoTool;
    BAUtils myBATool;

    ImgsVect myImgs;
    FeaturesVect myFeatureVects;



    Intrinsics mIntrinsics;
    std::vector<cv::Matx34f> mCameraPoses;
    std::set<int> mDoneViews;
    std::set<int> mGoodViews;
    PointCloud mReconstructionCloud;


};


#endif //ZARANTSFM_SFMPIPE_H
