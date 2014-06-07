#include "TestFuncs.h"
/*void GetImagePyramid(string rgb_filename,string depth_filename,dvo::core::RgbdImagePyramid& RIP)
{
    cv::Mat rgb = cv::imread(rgb_filename,CV_LOAD_IMAGE_COLOR);
    cv::Mat intensity = cv::imread(rgb_filename,CV_LOAD_IMAGE_GRAYSCALE);
    cv::Mat depth = cv::imread(depth_filename,CV_LOAD_IMAGE_GRAYSCALE);

    intensity.convertTo(intensity,CV_32FC1);
    depth.convertTo(depth,CV_32FC1);

    dvo::core::RgbdImagePyramid temp(intensity,depth);
    RIP = &temp;
}*/

void ShowImage(Mat image,int type,string WindowName)
{
    namedWindow( WindowName, WINDOW_AUTOSIZE );// Create a window for display.
    image.convertTo(image,type);
    imshow( WindowName, image);
}

void Test()
{

}
