//
// Created by nannan on 2019/10/20.
//
#include <sys/stat.h>
#include "MyFileUtils.h"
#include "SfMCommon.h"


using namespace boost::filesystem;
using namespace std;

std::vector<cv::Mat> MyFileUtils::setImagesDirectory(const std::string &directoryPath) {

    ImgNamesVect aImgNamesVect;
    ImgsVect aImgsVect;

    //define the path and ensure its existence
    path dirPath(directoryPath);
    if (not exists(dirPath) or not is_directory(dirPath)) {
        cerr << "Cannot open directory: " << directoryPath << endl;

    }

    //fetch all imgs with excepted extension and store their path in mImageFilenames
    for (directory_entry &x : directory_iterator(dirPath)) {
        std::string extension = x.path().extension().string();
        boost::algorithm::to_lower(extension);
        if (extension == ".jpg" or extension == ".png") {
            aImgNamesVect.push_back(x.path().string());
            cout<<x.path().string()<<" has been written to the aImgNamesVect."<<endl;
        }
    }

    if (aImgNamesVect.size() <= 0) {
        cerr << "Unable to find valid files in images directory (\"" << directoryPath << "\")." << endl;

    }

//    if (mConsoleDebugLevel <= LOG_DEBUG) {
//        cout << "Found " << mImageFilenames.size() << " image files in directory." << endl;
//    }

//read all images (default downscale factor=1,i.e.,keep the original size) into mImages
    for (auto &imageFilename : aImgNamesVect) {
        aImgsVect.push_back(cv::imread(imageFilename));

        if (mDownscaleFactor != 1.0) {
            cv::resize(aImgsVect.back(), aImgsVect.back(), cv::Size(), mDownscaleFactor, mDownscaleFactor);
        }

        if (aImgsVect.back().empty()) {
            cerr << "Unable to read image from file: " << imageFilename << endl;

        }
    }
    int aImgVectSize=aImgsVect.size();
    cout<<"The size of aImgsVect is :"<<to_string(aImgVectSize)<<endl;

    return aImgsVect;
}

void MyFileUtils::makeMyDirs() {
    if(0!=access(featureOutputPath.c_str(),0)){
        mkdir(featureOutputPath.c_str(),0777);
        cout<<"The path:"<<featureOutputPath.c_str()<<" is created successfully"<<endl;
    }
    if(0!=access(matchingOutputPath.c_str(),0)){
        mkdir(matchingOutputPath.c_str(),0777);
        cout<<"The path:"<<matchingOutputPath.c_str()<<" is created successfully"<<endl;
    }

}