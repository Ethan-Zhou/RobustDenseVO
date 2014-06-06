#include <iostream>
#include <dense_tracking.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <datatypes.h>
using namespace cv;
using namespace std;
using namespace dvo;
using namespace Eigen;
//dvo::core::RgbdImagePyramid GetImagePyramid(string rgb_filename,string depth_filename);
void ShowImage(Mat image,int type,string WindowName);
