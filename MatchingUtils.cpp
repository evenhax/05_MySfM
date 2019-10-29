//
// Created by nannan on 2019/10/21.
//

#include <thread>
#include "MatchingUtils.h"

using namespace std;

Matching MatchingUtils::matchFeatures(
        const Features &featuresLeft,
        const Features &featuresRight) {
    //initial matching between features
    std::vector<Matching> initialMatching;

    auto matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
    matcher->knnMatch(featuresLeft.descriptors, featuresRight.descriptors, initialMatching, 2);

    //prune the matching using the ratio test
    Matching prunedMatching;
    for (unsigned i = 0; i < initialMatching.size(); i++) {
        if (initialMatching[i][0].distance < NN_MATCH_RATIO * initialMatching[i][1].distance) {
            prunedMatching.push_back(initialMatching[i][0]);
        }
    }

    return prunedMatching;
}

void MatchingUtils::GetAlignedPointsFromMatch(const Features &leftFeatures,
                                              const Features &rightFeatures,
                                              const Matching &matches,
                                              Features &alignedLeft,
                                              Features &alignedRight) {
    std::vector<int> leftBackReference, rightBackReference;
    GetAlignedPointsFromMatch(
            leftFeatures,
            rightFeatures,
            matches,
            alignedLeft,
            alignedRight,
            leftBackReference,
            rightBackReference
    );

}

void MatchingUtils::GetAlignedPointsFromMatch(const Features &leftFeatures,
                                              const Features &rightFeatures,
                                              const Matching &matches,
                                              Features &alignedLeft,
                                              Features &alignedRight,
                                              std::vector<int> &leftBackReference,
                                              std::vector<int> &rightBackReference) {
    alignedLeft.keyPoints.clear();
    alignedRight.keyPoints.clear();
    alignedLeft.descriptors = cv::Mat();
    alignedRight.descriptors = cv::Mat();

    for (unsigned int i = 0; i < matches.size(); i++) {
        alignedLeft.keyPoints.push_back(leftFeatures.keyPoints[matches[i].queryIdx]);
        alignedLeft.descriptors.push_back(leftFeatures.descriptors.row(matches[i].queryIdx));
        alignedRight.keyPoints.push_back(rightFeatures.keyPoints[matches[i].trainIdx]);
        alignedRight.descriptors.push_back(rightFeatures.descriptors.row(matches[i].trainIdx));
        leftBackReference.push_back(matches[i].queryIdx);
        rightBackReference.push_back(matches[i].trainIdx);
    }

    KeyPointsToPoints(alignedLeft.keyPoints, alignedLeft.points);
    KeyPointsToPoints(alignedRight.keyPoints, alignedRight.points);
}

Matching MatchingUtils::GetAlignedMatching(size_t size) {
    Matching match;
    for (size_t i = 0; i < size; i++) {
        match.push_back(cv::DMatch(i, i, 0));
    }
    return match;
}

MatchMatrix MatchingUtils::createFeatureMatchMatrix(int imgsCount, std::vector<Features> mImgFeatureVect) {

    MatchMatrix mFeatureMatchMatrix;
    mFeatureMatchMatrix.resize(imgsCount, std::vector<Matching>(imgsCount));
    std::vector<ImagePair> pairs;
    for (size_t i = 0; i < imgsCount; i++) {
        for (size_t j = i + 1; j < imgsCount; j++) {
            pairs.push_back({i, j});
        }
    }

    vector<thread> threads;

    //find out how many threads are supported, and how many pairs each thread will work on
    const int numThreads = std::thread::hardware_concurrency() - 1;
    const int numPairsForThread = (numThreads > pairs.size()) ? 1 : (int) ceilf((float) (pairs.size()) / numThreads);

    mutex writeMutex;

    //invoke each thread with its pairs to process (if less pairs than threads, invoke only #pairs threads with 1 pair each)
    for (size_t threadId = 0; threadId < MIN(numThreads, pairs.size()); threadId++) {
        threads.push_back(thread([&, threadId] {
            const int startingPair = numPairsForThread * threadId;

            for (int j = 0; j < numPairsForThread; j++) {
                const int pairId = startingPair + j;
                if (pairId >= pairs.size()) { //make sure threads don't overflow the pairs
                    break;
                }
                const ImagePair &pair = pairs[pairId];
                mFeatureMatchMatrix[pair.left][pair.right] = matchFeatures(mImgFeatureVect[pair.left],
                                                                           mImgFeatureVect[pair.right]);

//                if (mConsoleDebugLevel <= LOG_DEBUG) {
//                    writeMutex.lock();
//                    cout << "Thread " << threadId << ": Match (pair " << pairId << ") " << pair.left << ", " << pair.right << ": " << mFeatureMatchMatrix[pair.left][pair.right].size() << " matched features" << endl;
//                    writeMutex.unlock();
//                }
            }
        }));
    }

    //wait for threads to complete
    for (auto &t : threads) {
        t.join();
    }
return mFeatureMatchMatrix;

}