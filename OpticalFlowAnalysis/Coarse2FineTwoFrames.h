#include "project.h"
#include "Image.h"
#include "OpticalFlow.h"
#include <iostream>
#include "cv.h"
#include "highgui.h"
#include "ImageIO.h"
#include <math.h>
using namespace OpticalFlowAnalysis;
bool GetOpticalFlow(const cv::Mat& Intensity_Ref,const cv::Mat& Intensity_Cur,DImage& vx,DImage& vy,DImage& warpI2);
void GetWccWithOF(DImage vx, DImage vy, int t_Norm, int t_Dir, cv::Mat& Wcc,int type);

