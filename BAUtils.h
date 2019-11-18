//
// Created by nannan on 2019/10/21.
//

#ifndef ZARANTSFM_BAUTILS_H
#define ZARANTSFM_BAUTILS_H

#include "SfMCommon.h"
#include <ceres/ceres.h>
#include<ceres/rotation.h>

class BAUtils {

public:
    /**
     *
     * @param pointCloud
     * @param cameraPoses
     * @param intrinsics
     * @param image2dFeatures
     */
    static bool adjustBundle(
            PointCloud &pointCloud,
            std::vector<cv::Matx34f> &cameraPoses,
            Intrinsics &intrinsics,
            const std::vector<Features> &image2dFeatures
    );

};


#endif //ZARANTSFM_BAUTILS_H
