

//int main() {
//    MyFileUtils myFileTool;
//    FeatureUtils myFeatureTool;
//    MatchingUtils myMatchTool;
//    if(myFileTool.setImagesDirectory(imageSourcePath)){
//        myFeatureTool.extractMultiImgKeys(myFileTool.returnImgs());
//    }
//    myMatchTool.createFeatureMatchMatrix(myFileTool.returnNumImgs(),myFeatureTool.returnMImageFeatures());
//
//}


#include "SfMPipe.h"

using namespace std;


int main(){
    SfMPipe myPipe;
    myPipe.mySfMSet();
    myPipe.findBaselineTriangulation();
    myPipe.adjustCurrentBundle();
    myPipe.saveCloudAndCamerasToPLY(totalPath);
}