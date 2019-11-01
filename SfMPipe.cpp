//
// Created by nannan on 2019/10/28.
//
#include "SfMPipe.h"


using namespace std;
using namespace cv;

SfMPipe::SfMPipe() {

}

bool SfMPipe::mySfMSet() {

    myFileTool.makeMyDirs();
    myImgs = myFileTool.setImagesDirectory(imageSourcePath);
    if (myImgs.size() <= 0) {
        cerr << "No images to work on." << endl;
        return ErrorCode::ERROR;
    } else {
        myFeatureVects = myFeatureTool.extractMultiImgKeys(myImgs);
    }
    //initialize intrinsics
    mIntrinsics.K = (Mat_<float>(3, 3) << 2500, 0, myImgs[0].cols / 2,
            0, 2500, myImgs[0].rows / 2,
            0, 0, 1);
    mIntrinsics.Kinv = mIntrinsics.K.inv();
    mIntrinsics.distortion = Mat_<float>::zeros(1, 4);

    mCameraPoses.resize(myImgs.size());

    //Create a matching matrix between all images' features
    myMatchMatrix = myMatchTool.createFeatureMatchMatrix(myImgs.size(), myFeatureVects, myImgs);

    cout << "The pipleline has been set successfully, the features have been extracted and match matrix is created."
         << endl;
    return true;
}

//ErrorCode SfMPipe::runSfM() {
//    //Find the best two views for an initial triangulation on the 3D map
//    findBaselineTriangulation();
//    //Lastly - add more camera views to the map
//    addMoreViewsToReconstruction();
//    cout << "----------------------- Done -----------------------" << endl;
//    return OKAY;
//}


void SfMPipe::findBaselineTriangulation() {

    cout<<"Start to find the initial triangulation."<<endl;
    //maps are sorted, so the best pair is at the beginnning
    map<float, ImagePair> pairsHomographyInliers = sortViewsForBaseline();

    Matx34f Pleft = Matx34f::eye();
    Matx34f Pright = Matx34f::eye();
    PointCloud pointCloud;

    //try to find the best pair, stating at the beginning
    for (auto &imagePair : pairsHomographyInliers) {

        size_t i = imagePair.second.left;
        size_t j = imagePair.second.right;

        Matching prunedMatching;
        //recover camera matrices (poses) from the point matching
        bool success = StereoUtils::findCameraMatricesFromMatch(
                mIntrinsics,
                myMatchMatrix[i][j],
                myFeatureVects[i],
                myFeatureVects[j],
                prunedMatching,
                Pleft, Pright
        );
//        if (not success) {
//                cerr << "stereo view could not be obtained " << imagePair.second.str() << ", go to next pair" << endl << flush;
//            continue;
//        }

        if (not success) {
            cerr << "stereo view could not be obtained , go to next pair" << endl << flush;
            continue;
        }

        float poseInliersRatio = (float) prunedMatching.size() / (float) myMatchMatrix[i][j].size();

//        if (mConsoleDebugLevel <= LOG_TRACE) {
//            cout << "pose inliers ratio " << poseInliersRatio << endl;
//        }

        if (poseInliersRatio < POSE_INLIERS_MINIMAL_RATIO) {

            cout << "insufficient pose inliers. skip." << endl;

            continue;
        }


        Mat outImage;
        drawMatches(myImgs[i], myFeatureVects[i].keyPoints,
                    myImgs[j], myFeatureVects[j].keyPoints,
                    prunedMatching,
                    outImage);
        resize(outImage, outImage, cv::Size(), 0.5, 0.5);
        imshow("outimage", outImage);
        waitKey(0);


        myMatchMatrix[i][j] = prunedMatching;

//        if (mConsoleDebugLevel <= LOG_DEBUG) {
//            cout << "---- Triangulate from stereo views: " << imagePair.second << endl;
//        }
        success = StereoUtils::triangulateViews(
                mIntrinsics,
                imagePair.second,
                myMatchMatrix[i][j],
                myFeatureVects[i], myFeatureVects[j],
                Pleft, Pright,
                pointCloud
        );

        if (not success) {

            cerr << "could not triangulate: " << endl << flush;

            continue;
        }

        mReconstructionCloud = pointCloud;
        mCameraPoses[i] = Pleft;
        mCameraPoses[j] = Pright;
        mDoneViews.insert(i);
        mDoneViews.insert(j);
        mGoodViews.insert(i);
        mGoodViews.insert(j);

        adjustCurrentBundle();

        break;
    }
}


void SfMPipe::adjustCurrentBundle() {
    BAUtils::adjustBundle(
            mReconstructionCloud,
            mCameraPoses,
            mIntrinsics,
            myFeatureVects);
}


map<float, ImagePair> SfMPipe::sortViewsForBaseline() {
//    if (mConsoleDebugLevel <= LOG_INFO) {
//        cout << "---------- Find Views Homography Inliers -----------" << endl;
//    }
    //sort pairwise matches to find the lowest Homography inliers [Snavely07 4.2]
    cout<<"Start sortViewsForBaseline"<<endl;
    map<float, ImagePair> matchesSizes;
    const size_t numImages = myImgs.size();
    for (size_t i = 0; i < numImages - 1; i++) {
        for (size_t j = i + 1; j < numImages; j++) {
            if (myMatchMatrix[i][j].size() < MIN_POINT_COUNT_FOR_HOMOGRAPHY) {
                //Not enough points in matching
                matchesSizes[1.0] = {i, j};
                continue;
            }
            //Find number of homography inliers
            const int numInliers = StereoUtils::findHomographyInliers(
                    myFeatureVects[i],
                    myFeatureVects[j],
                    myMatchMatrix[i][j]);
            const float inliersRatio = (float) numInliers / (float) (myMatchMatrix[i][j].size());
            matchesSizes[inliersRatio] = {i, j};

//            if (mConsoleDebugLevel <= LOG_DEBUG) {
//                cout << "Homography inliers ratio: " << i << ", " << j << " " << inliersRatio << endl;
//            }
        }
    }
    cout<<"The matchesSizes is extracted succesfully with size:"<<to_string(matchesSizes.size())<<endl;
    return matchesSizes;
}

void SfMPipe::addMoreViewsToReconstruction() {
//    if (mConsoleDebugLevel <= LOG_INFO) {
//        cout << "------------------ Add More Views ------------------" << endl;
//    }

    while (mDoneViews.size() != myImgs.size()) {
        //Find the best view to add, according to the largest number of 2D-3D corresponding points
        Images2D3DMatches matches2D3D = find2D3DMatches();

        size_t bestView;
        size_t bestNumMatches = 0;
        for (const auto &match2D3D : matches2D3D) {
            const size_t numMatches = match2D3D.second.points2D.size();
            if (numMatches > bestNumMatches) {
                bestView = match2D3D.first;
                bestNumMatches = numMatches;
            }
        }
//        if (mConsoleDebugLevel <= LOG_DEBUG) {
//            cout << "Best view " << bestView << " has " << bestNumMatches << " matches" << endl;
//            cout << "Adding " << bestView << " to existing " << Mat(vector<int>(mGoodViews.begin(), mGoodViews.end())).t() << endl;
//        }

        mDoneViews.insert(bestView);

        //recover the new view camera pose
        Matx34f newCameraPose;
        bool success = StereoUtils::findCameraPoseFrom2D3DMatch(
                mIntrinsics,
                matches2D3D[bestView],
                newCameraPose);

        if (not success) {
//            if (mConsoleDebugLevel <= LOG_WARN) {
            cerr << "Cannot recover camera pose for view " << bestView << endl;
//            }
            continue;
        }

        mCameraPoses[bestView] = newCameraPose;

//        if (mConsoleDebugLevel <= LOG_DEBUG) {
//            cout << "New view " << bestView << " pose " << endl << newCameraPose << endl;
//        }

        //triangulate more points from new view to all existing good views
        bool anyViewSuccess = false;
        for (const int goodView : mGoodViews) {
            //since match matrix is upper-triangular (non symmetric) - use lower index as left
            size_t leftViewIdx = (goodView < bestView) ? goodView : bestView;
            size_t rightViewIdx = (goodView < bestView) ? bestView : goodView;

            Matching prunedMatching;
            Matx34f Pleft = Matx34f::eye();
            Matx34f Pright = Matx34f::eye();

            //use the essential matrix recovery to prune the matches
            bool success = StereoUtils::findCameraMatricesFromMatch(
                    mIntrinsics,
                    myMatchMatrix[leftViewIdx][rightViewIdx],
                    myFeatureVects[leftViewIdx],
                    myFeatureVects[rightViewIdx],
                    prunedMatching,
                    Pleft, Pright
            );
            myMatchMatrix[leftViewIdx][rightViewIdx] = prunedMatching;

            //triangulate the matching points
            PointCloud pointCloud;
            success = StereoUtils::triangulateViews(
                    mIntrinsics,
                    {leftViewIdx, rightViewIdx},
                    myMatchMatrix[leftViewIdx][rightViewIdx],
                    myFeatureVects[leftViewIdx],
                    myFeatureVects[rightViewIdx],
                    mCameraPoses[leftViewIdx],
                    mCameraPoses[rightViewIdx],
                    pointCloud
            );

            if (success) {
//                if (mConsoleDebugLevel <= LOG_DEBUG) {
//                    cout << "Merge triangulation between " << leftViewIdx << " and " << rightViewIdx <<
//                         " (# matching pts = " << (myMatchMatrix[leftViewIdx][rightViewIdx].size()) << ") ";
//                }

                //add new points to the reconstruction
                mergeNewPointCloud(pointCloud);

                anyViewSuccess = true;
            } else {
//                if (mConsoleDebugLevel <= LOG_WARN) {
                cerr << "Failed to triangulate " << leftViewIdx << " and " << rightViewIdx << endl;
//                }
            }
        }

        //Adjust bundle if any additional view was added
        if (anyViewSuccess) {
            adjustCurrentBundle();
        }
        mGoodViews.insert(bestView);
    }
}

SfMPipe::Images2D3DMatches SfMPipe::find2D3DMatches() {
    Images2D3DMatches matches;

    //scan all not-done views
    for (size_t viewIdx = 0; viewIdx < myImgs.size(); viewIdx++) {
        if (mDoneViews.find(viewIdx) != mDoneViews.end()) {
            continue; //skip done views
        }

        Image2D3DMatch match2D3D;

        //scan all cloud 3D points
        for (const Point3DInMap &cloudPoint : mReconstructionCloud) {
            bool found2DPoint = false;

            //scan all originating views for that 3D point
            for (const auto &origViewAndPoint : cloudPoint.originatingViews) {
                //check for 2D-2D matching via the match matrix
                const int originatingViewIndex = origViewAndPoint.first;
                const int originatingViewFeatureIndex = origViewAndPoint.second;

                //match matrix is upper-triangular (not symmetric) so the left index must be the smaller one
                const int leftViewIdx = (originatingViewIndex < viewIdx) ? originatingViewIndex : viewIdx;
                const int rightViewIdx = (originatingViewIndex < viewIdx) ? viewIdx : originatingViewIndex;

                //scan all 2D-2D matches between originating view and new view
                for (const DMatch &m : myMatchMatrix[leftViewIdx][rightViewIdx]) {
                    int matched2DPointInNewView = -1;
                    if (originatingViewIndex < viewIdx) { //originating view is 'left'
                        if (m.queryIdx == originatingViewFeatureIndex) {
                            matched2DPointInNewView = m.trainIdx;
                        }
                    } else {                              //originating view is 'right'
                        if (m.trainIdx == originatingViewFeatureIndex) {
                            matched2DPointInNewView = m.queryIdx;
                        }
                    }
                    if (matched2DPointInNewView >= 0) {
                        //This point is matched in the new view
                        const Features &newViewFeatures = myFeatureVects[viewIdx];
                        match2D3D.points2D.push_back(newViewFeatures.points[matched2DPointInNewView]);
                        match2D3D.points3D.push_back(cloudPoint.p);
                        found2DPoint = true;
                        break;
                    }
                }

                if (found2DPoint) {
                    break;
                }
            }
        }

        matches[viewIdx] = match2D3D;
    }

    return matches;
}

void SfMPipe::mergeNewPointCloud(const PointCloud &cloud) {
    const size_t numImages = myImgs.size();
    MatchMatrix mergeMatchMatrix;
    mergeMatchMatrix.resize(numImages, vector<Matching>(numImages));

    size_t newPoints = 0;
    size_t mergedPoints = 0;

    for (const Point3DInMap &p : cloud) {
        const Point3f newPoint = p.p; //new 3D point

        bool foundAnyMatchingExistingViews = false;
        bool foundMatching3DPoint = false;
        for (Point3DInMap &existingPoint : mReconstructionCloud) {
            if (norm(existingPoint.p - newPoint) < MERGE_CLOUD_POINT_MIN_MATCH_DISTANCE) {
                //This point is very close to an existing 3D cloud point
                foundMatching3DPoint = true;

                //Look for common 2D features to confirm match
                for (const auto &newKv : p.originatingViews) {
                    //kv.first = new point's originating view
                    //kv.second = new point's view 2D feature index

                    for (const auto &existingKv : existingPoint.originatingViews) {
                        //existingKv.first = existing point's originating view
                        //existingKv.second = existing point's view 2D feature index

                        bool foundMatchingFeature = false;

                        const bool newIsLeft = newKv.first < existingKv.first;
                        const int leftViewIdx = (newIsLeft) ? newKv.first : existingKv.first;
                        const int leftViewFeatureIdx = (newIsLeft) ? newKv.second : existingKv.second;
                        const int rightViewIdx = (newIsLeft) ? existingKv.first : newKv.first;
                        const int rightViewFeatureIdx = (newIsLeft) ? existingKv.second : newKv.second;

                        const Matching &matching = myMatchMatrix[leftViewIdx][rightViewIdx];
                        for (const DMatch &match : matching) {
                            if (match.queryIdx == leftViewFeatureIdx
                                and match.trainIdx == rightViewFeatureIdx
                                and match.distance < MERGE_CLOUD_FEATURE_MIN_MATCH_DISTANCE) {

                                mergeMatchMatrix[leftViewIdx][rightViewIdx].push_back(match);

                                //Found a 2D feature match for the two 3D points - merge
                                foundMatchingFeature = true;
                                break;
                            }
                        }

                        if (foundMatchingFeature) {
                            //Add the new originating view, and feature index
                            existingPoint.originatingViews[newKv.first] = newKv.second;

                            foundAnyMatchingExistingViews = true;

                        }
                    }
                }
            }
            if (foundAnyMatchingExistingViews) {
                mergedPoints++;
                break; //Stop looking for more matching cloud points
            }
        }

        if (not foundAnyMatchingExistingViews and not foundMatching3DPoint) {
            //This point did not match any existing cloud points - add it as new.
            mReconstructionCloud.push_back(p);
            newPoints++;
        }
    }

//    if (mVisualDebugLevel <= LOG_DEBUG) {
    //debug: show new matching points in the cloud
    for (size_t i = 0; i < numImages - 1; i++) {
        for (size_t j = i; j < numImages; j++) {
            const Matching &matching = mergeMatchMatrix[i][j];
            if (matching.empty()) {
                continue;
            }

            Mat outImage;
            drawMatches(myImgs[i], myFeatureVects[i].keyPoints,
                        myImgs[j], myFeatureVects[j].keyPoints,
                        matching, outImage);
            //write the images index...
            putText(outImage, "Image " + to_string(i), cv::Point(10, 50), CV_FONT_NORMAL, 3.0, cv::Scalar(0, 255, 255),
                    3);
            putText(outImage, "Image " + to_string(j), cv::Point(10 + outImage.cols / 2, 50), CV_FONT_NORMAL, 3.0,
                    cv::Scalar(0, 255, 255), 3);
            resize(outImage, outImage, cv::Size(), 0.25, 0.25);
            imshow("Merge Match", outImage);
            waitKey(0);
        }
    }
    destroyWindow("Merge Match");
//    }

//    if (mConsoleDebugLevel <= LOG_DEBUG) {
//        cout << " adding: " << cloud.size() << " (new: " << newPoints << ", merged: " << mergedPoints << ")" << endl;
//    }
}

void SfMPipe::saveCloudAndCamerasToPLY(const std::string &prefix) {
//    if (mConsoleDebugLevel <= LOG_INFO) {
//        cout << "Saving result reconstruction with prefix " << prefix << endl;
//    }

    ofstream ofs(prefix + "_points.ply");

    //write PLY header
    ofs << "ply                 " << endl <<
        "format ascii 1.0    " << endl <<
        "element vertex " << mReconstructionCloud.size() << endl <<
        "property float x    " << endl <<
        "property float y    " << endl <<
        "property float z    " << endl <<
        "property uchar red  " << endl <<
        "property uchar green" << endl <<
        "property uchar blue " << endl <<
        "end_header          " << endl;

    for (const Point3DInMap &p : mReconstructionCloud) {
        //get color from first originating view
        auto originatingView = p.originatingViews.begin();
        const int viewIdx = originatingView->first;
        Point2f p2d = myFeatureVects[viewIdx].points[originatingView->second];
        Vec3b pointColor = myImgs[viewIdx].at<Vec3b>(p2d);

        //write vertex
        ofs << p.p.x << " " <<
            p.p.y << " " <<
            p.p.z << " " <<
            (int) pointColor(2) << " " <<
            (int) pointColor(1) << " " <<
            (int) pointColor(0) << " " << endl;
    }

    ofs.close();

    ofstream ofsc(prefix + "_cameras.ply");

    //write PLY header
    ofsc << "ply                 " << endl <<
         "format ascii 1.0    " << endl <<
         "element vertex " << (mCameraPoses.size() * 4) << endl <<
         "property float x    " << endl <<
         "property float y    " << endl <<
         "property float z    " << endl <<
         "element edge " << (mCameraPoses.size() * 3) << endl <<
         "property int vertex1" << endl <<
         "property int vertex2" << endl <<
         "property uchar red  " << endl <<
         "property uchar green" << endl <<
         "property uchar blue " << endl <<
         "end_header          " << endl;

    //save cameras polygons..
    for (const auto &pose : mCameraPoses) {
        Point3d c(pose(0, 3), pose(1, 3), pose(2, 3));
        Point3d cx = c + Point3d(pose(0, 0), pose(1, 0), pose(2, 0)) * 0.2;
        Point3d cy = c + Point3d(pose(0, 1), pose(1, 1), pose(2, 1)) * 0.2;
        Point3d cz = c + Point3d(pose(0, 2), pose(1, 2), pose(2, 2)) * 0.2;

        ofsc << c.x << " " << c.y << " " << c.z << endl;
        ofsc << cx.x << " " << cx.y << " " << cx.z << endl;
        ofsc << cy.x << " " << cy.y << " " << cy.z << endl;
        ofsc << cz.x << " " << cz.y << " " << cz.z << endl;
    }

    const int camVertexStartIndex = mReconstructionCloud.size();

    for (size_t i = 0; i < mCameraPoses.size(); i++) {
        ofsc << (i * 4 + 0) << " " <<
             (i * 4 + 1) << " " <<
             "255 0 0" << endl;
        ofsc << (i * 4 + 0) << " " <<
             (i * 4 + 2) << " " <<
             "0 255 0" << endl;
        ofsc << (i * 4 + 0) << " " <<
             (i * 4 + 3) << " " <<
             "0 0 255" << endl;
    }
}