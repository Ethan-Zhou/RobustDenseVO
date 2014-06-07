#include <iostream>
#include <fstream>
#include <dense_tracking.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <datatypes.h>
#include <Eigen/Core>
#include <dvo/util/revertable.h>
#include "TestFuncs.h"
#include "Image.h"
#include "Coarse2FineTwoFrames.h"

#include "project.h"
#include "OpticalFlow.h"
#include "ImageIO.h"
#include <math.h>

using namespace cv;
using namespace std;
using namespace dvo::core;
using namespace dvo;
using namespace Eigen;
using namespace dvo::util;
using namespace OpticalFlowAnalysis;

int main()
{
    // Core Object definition
    IntrinsicMatrix IM = IntrinsicMatrix();
    //IM = IM.create(517.3f,516.5f,318.6f,255.3f);//fr1
    IM = IM.create(520.9f,521.0f,325.1f,249.7f);//fr2
    DenseTracker DTObj = DenseTracker(IM);
    DTObj.configure();
    Eigen::Affine3d Tran;
    Tran.setIdentity();
    //DTObj.itctx_.Level = 1;
    //cout << DTObj.itctx_.Level << endl;
    //dvo::core::NormalEquationsLeastSquares Nels;

    // Main Iteration

    // TODO: Read successive frames (rgb,intensity,depth)
    //const int FrameNum = 792;//fr1
    const int FrameNum = 3983;//fr2
    char rgb_filename[73];//[73][60];
    char depth_filename[75];//[75][62];
    char buf[23];
    char rgb_dir[] = "dataset/rgbd_dataset_freiburg2_desk_with_person/rgb/";//"dataset/rgbd_dataset_freiburg1_xyz/rgb/"
    char depth_dir[] = "dataset/rgbd_dataset_freiburg2_desk_with_person/depth/";//"dataset/rgbd_dataset_freiburg1_xyz/depth/";
    string aaa;
    string bbb;
    ifstream rgb_txt;
    ifstream depth_txt;
    rgb_txt.open("dataset/rgbd_dataset_freiburg2_desk_with_person/rgb_ass.txt");//("dataset/rgbd_dataset_freiburg1_xyz/rgb_ass.txt");//
    depth_txt.open("dataset/rgbd_dataset_freiburg2_desk_with_person/depth_ass.txt");//("dataset/rgbd_dataset_freiburg1_xyz/depth_ass.txt");//

    const char OutputFilename[] = "MotionMatrix_rgbd_dataset_freiburg2_desk_with_person.txt";//"MotionMatrix_rgbd_dataset_freiburg1_xyz.txt";
    ofstream o_file;

    cv::Mat rgb_ref;
    cv::Mat intensity_ref;
    cv::Mat depth_ref;
    cv::Mat rgb_cur;
    cv::Mat intensity_cur;
    cv::Mat depth_cur;

    cv::Mat ImgTemp;
    o_file.open(OutputFilename);

    for(int i = 1;i <= 1232;i++)
    {
        rgb_txt.getline(buf,23);
        depth_txt.getline(buf,23);
    }

    if(rgb_txt.is_open() && depth_txt.is_open() && o_file.is_open())
    {
        // Read Reference frame
        rgb_txt.getline(buf,23);
        strcpy(rgb_filename,rgb_dir);
        strcat(rgb_filename,buf);
        aaa = rgb_filename;
        aaa.erase(aaa.size()-1);
        //cout << aaa << endl;
        rgb_ref = imread(aaa,CV_LOAD_IMAGE_UNCHANGED);
        intensity_ref = imread(aaa,CV_LOAD_IMAGE_GRAYSCALE);

        depth_txt.getline(buf,23);
        strcpy(depth_filename,depth_dir);
        strcat(depth_filename,buf);
        bbb = depth_filename;
        //cout << bbb << endl;
        bbb.erase(bbb.size()-1);
        depth_ref = imread(bbb,CV_LOAD_IMAGE_UNCHANGED);

        //while(!rgb_txt.eof() || !depth_txt.eof())
        for(int i = 2;i <= 2 ;i++)
        {
            // Read Current frame
            rgb_txt.getline(buf,23);
            strcpy(rgb_filename,rgb_dir);
            strcat(rgb_filename,buf);
            aaa = rgb_filename;
            aaa.erase(aaa.size()-1);
            //cout << aaa << endl;
            rgb_cur = imread(aaa,CV_LOAD_IMAGE_UNCHANGED);
            intensity_cur = imread(aaa,CV_LOAD_IMAGE_GRAYSCALE);

            depth_txt.getline(buf,23);
            strcpy(depth_filename,depth_dir);
            strcat(depth_filename,buf);
            bbb = depth_filename;
            bbb.erase(bbb.size()-1);
            //cout << bbb <<endl;
            depth_cur = imread(bbb,CV_LOAD_IMAGE_UNCHANGED);

            //namedWindow( "Display window1", WINDOW_AUTOSIZE );// Create a window for display.
            //imshow( "Display window1", depth_cur);

            // Preprocessing (channel)
            intensity_ref.convertTo(intensity_ref,CV_32FC1);
            intensity_cur.convertTo(intensity_cur,CV_32FC1);
            depth_ref.convertTo(depth_ref,CV_32FC1);
            depth_cur.convertTo(depth_cur,CV_32FC1);
            depth_ref /= 5000.0f;
            depth_cur /= 5000.0f;
            //cout << intensity_ref.ptr[0] << endl;
            //cout << depth_ref.at<float>(200,200) << endl;
            //cout << depth_ref.data[200*depth_ref.size().width] << endl;

            //  Camera Motion Estimation
            RgbdImagePyramid RefPyd(intensity_ref,depth_ref);
            RgbdImagePyramid CurPyd(intensity_cur,depth_cur);

            Tran.setIdentity();
            /*********************************************************/
            /*Optical Flow Test*/
            /*********************************************************/
            //cout << intensity_ref.size().width << endl;
            DImage vx(intensity_ref.size().width,intensity_ref.size().height,1);
            DImage vy(intensity_ref.size().width,intensity_ref.size().height,1);
            DImage warp2(intensity_ref.size().width,intensity_ref.size().height,1);
            cv::Mat Wcc;
            //ShowImage(intensity_ref,intensity_ref.type(),"intensity_ref");

            intensity_ref.convertTo(intensity_ref,CV_8UC1);
            intensity_cur.convertTo(intensity_cur,CV_8UC1);
            //cout << (int)intensity_ref.at<unsigned char>(100,100) << endl;

            GetOpticalFlow(intensity_ref,intensity_cur,vx,vy,warp2);
            /*ofstream of1,of2;
            of1.open("vx_data.txt");
            of2.open("vy_data.txt");
            for(int i = 0;i<vx.width()*vx.height();i++)
            {
                of1 << vx.pData[i];
                of2 << vy.pData[i];
                if((i+1)%vx.width()==0)
                {
                    of1 << "\n";
                    of2 << "\n";
                }
                else
                {
                    of1 << " ";
                    of2 << " ";
                }
            }
            of1.close();
            of2.close();*/
            // Look at the result of the vx vy
            //cv::Mat Vx;
            //cv::Mat Vy;
            //ImageIO::Image2cvMat(vx.pData,Vx,vx.width(),vx.height(),1);
            //ImageIO::Image2cvMat(vy.pData,Vy,vy.width(),vy.height(),1);
            //ShowImage(Vx,0,"vx");
            //ShowImage(Vy,0,"vy");
            //cout << (int)(Vx.data[2000]) << endl;
            cout << vx.pData[0] << " "<< vx.pData[99] <<" "<< vy.pData[0] <<" "<< vy.pData[99] << endl;
            GetWccWithOF(vx,vy,2.5,1.5,Wcc,CV_8UC1);
            ShowImage(Wcc,0,"Wcc_OF");

            //Ref is 1st frame, Cur is 2nd frame;
            //we want: 1->2
            /**********************************/
            //DTObj.match(RefPyd,CurPyd,Tran);
            cout << i - 1 << endl;
            cout << Tran.matrix() << endl;
            //cout << Tran.translation() << endl;

            // Copy Cur to Ref
            rgb_ref = imread(aaa,CV_LOAD_IMAGE_UNCHANGED);
            intensity_ref = imread(aaa,CV_LOAD_IMAGE_GRAYSCALE);
            depth_ref = imread(bbb,CV_LOAD_IMAGE_UNCHANGED);

            /**************************************************************************/
            /**Used for test the correctness of the motion estimatio*/
            /**********************************************************/
            /*
            RgbdImage Ref = RefPyd.level(0);
            RgbdImage Cur = CurPyd.level(0);

            RgbdImage result;
            Mat Residual1;
            Mat Residual2;

            AffineTransform transformation = Tran.cast<NumType>();
            //cout << *(transformation.data() + 3) << endl;
            //cout << (transformation.rotation() * transformation.rotation().transpose()).matrix() << endl;
            //cout << transformation.translation() << endl;
            //cout << transformation.matrix().col(3) <<endl;
            //Revertable<AffineTransform> estimate(AffineTransform::Identity());
            //estimate.update() = Tran.matrix().cast<NumType>() * estimate().matrix();

            Cur.buildPointCloud(IM);
            Ref.buildPointCloud(IM);

            Ref.warpIntensitySse(transformation,Cur.pointcloud,IM,result);//,Cur.pointcloud);

            Residual1 = Ref.intensity - Cur.intensity;
            Residual2 = result.intensity - Cur.intensity;

            //ShowImage(Residual1,0,"I1-I2");
            //ShowImage(Residual2,0,"I11-I2");
            */

            /****************************************************************************/
            // Write the motion matrix to output file
            /******************************************************************************/
            //o_file << Tran.matrix() << endl;
        }
        o_file.close();
        rgb_txt.close();
        depth_txt.close();
    }

    /*
    cv::Mat rgb_ref = cv::imread("dataset/rgb/1.png",CV_LOAD_IMAGE_COLOR);
    cv::Mat rgb_cur = cv::imread("dataset/rgb/2.png",CV_LOAD_IMAGE_COLOR);
    cv::Mat intensity_ref = cv::imread("dataset/rgb/1.png",CV_LOAD_IMAGE_GRAYSCALE);
    cv::Mat intensity_cur = cv::imread("dataset/rgb/2.png",CV_LOAD_IMAGE_GRAYSCALE);
    cv::Mat depth_ref = cv::imread("dataset/depth/1.png",CV_LOAD_IMAGE_GRAYSCALE);
    cv::Mat depth_cur = cv::imread("dataset/depth/2.png",CV_LOAD_IMAGE_GRAYSCALE);

    // TODO: PreProcessing (channel)
    intensity_ref.convertTo(intensity_ref,CV_32FC1);
    intensity_cur.convertTo(intensity_cur,CV_32FC1);
    depth_ref.convertTo(depth_ref,CV_32FC1);
    depth_cur.convertTo(depth_cur,CV_32FC1);


    */
    waitKey(0);
    return 0;
}



