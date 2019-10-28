//
// Created by nannan on 2019/10/20.
//


#include "SfMCommon.h"


void KeyPointsToPoints(const Keypoints &kps, Points2f &ps) {
    ps.clear();
    for (const auto &kp : kps) {
        ps.push_back(kp.pt);
    }
}

Keypoints PointsToKeyPoints(const Points2f &ps) {
    Keypoints kps;
    PointsToKeyPoints(ps, kps);
    return kps;
}

void PointsToKeyPoints(const Points2f &ps, Keypoints &kps) {
    kps.clear();
    for (const auto &p : ps) {
        kps.push_back(cv::KeyPoint(p, 1.0f));
    }
}

