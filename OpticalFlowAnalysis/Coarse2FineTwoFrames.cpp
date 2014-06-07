//#include "mex.h"
#include "Coarse2FineTwoFrames.h"
using namespace std;
namespace OpticalFlowAnalysis
{
// void LoadImage(DImage& image,const mxArray* matrix)
// {
// 	if(mxIsClass(matrix,"uint8"))
// 	{
// 		image.LoadMatlabImage<unsigned char>(matrix);
// 		return;
// 	}
// 	if(mxIsClass(matrix,"int8"))
// 	{
// 		image.LoadMatlabImage<char>(matrix);
// 		return;
// 	}
// 	if(mxIsClass(matrix,"int32"))
// 	{
// 		image.LoadMatlabImage<int>(matrix);
// 		return;
// 	}
// 	if(mxIsClass(matrix,"uint32"))
// 	{
// 		image.LoadMatlabImage<unsigned int>(matrix);
// 		return;
// 	}
// 	if(mxIsClass(matrix,"int16"))
// 	{
// 		image.LoadMatlabImage<short int>(matrix);
// 		return;
// 	}
// 	if(mxIsClass(matrix,"uint16"))
// 	{
// 		image.LoadMatlabImage<unsigned short int>(matrix);
// 		return;
// 	}
// 	if(mxIsClass(matrix,"double"))
// 	{
// 		image.LoadMatlabImage<double>(matrix);
// 		return;
// 	}
// 	mexErrMsgTxt("Unknown type of the image!");
// }

/*
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
	// check for proper number of input and output arguments
	if(nrhs<2 || nrhs>3)
		mexErrMsgTxt("Only two or three input arguments are allowed!");
	if(nlhs<2 || nlhs>3)
		mexErrMsgTxt("Only two or three output arguments are allowed!");
	DImage Im1,Im2;
    Im1.LoadMatlabImage(prhs[0]);
    Im2.LoadMatlabImage(prhs[1]);
	//LoadImage(Im1,prhs[0]);
	//LoadImage(Im2,prhs[1]);
	//mexPrintf("width %d   height %d   nchannels %d\n",Im1.width(),Im1.height(),Im1.nchannels());
	//mexPrintf("width %d   height %d   nchannels %d\n",Im2.width(),Im2.height(),Im2.nchannels());
	if(Im1.matchDimension(Im2)==false)
		mexErrMsgTxt("The two images don't match!");

	// get the parameters
	double alpha= 1;
	double ratio=0.5;
	int minWidth= 40;
	int nOuterFPIterations = 3;
	int nInnerFPIterations = 1;
	int nSORIterations= 20;
	if(nrhs>2)
	{
		int nDims=mxGetNumberOfDimensions(prhs[2]);
		const int *dims=mxGetDimensions(prhs[2]);
		double* para=(double *)mxGetData(prhs[2]);
		int npara=dims[0]*dims[1];
		if(npara>0)
			alpha=para[0];
		if(npara>1)
			ratio=para[1];
		if(npara>2)
			minWidth=para[2];
		if(npara>3)
			nOuterFPIterations=para[3];
		if(npara>4)
			nInnerFPIterations=para[4];
		if(npara>5)
			nSORIterations = para[5];
	}
	//mexPrintf("alpha: %f   ratio: %f   minWidth: %d  nOuterFPIterations: %d  nInnerFPIterations: %d   nCGIterations: %d\n",alpha,ratio,minWidth,nOuterFPIterations,nInnerFPIterations,nCGIterations);

	DImage vx,vy,warpI2;
	OpticalFlow::Coarse2FineFlow(vx,vy,warpI2,Im1,Im2,alpha,ratio,minWidth,nOuterFPIterations,nInnerFPIterations,nSORIterations);

	// output the parameters
	vx.OutputToMatlab(plhs[0]);
	vy.OutputToMatlab(plhs[1]);
	if(nlhs>2)
		warpI2.OutputToMatlab(plhs[2]);
}
*/

bool GetOpticalFlow(const cv::Mat& Intensity_Ref,const cv::Mat& Intensity_Cur,DImage& vx,DImage& vy,DImage& warpI2)
{
	// Remember to convert the mat img into the corresping type!!!
	assert(Intensity_Ref.type() == CV_8UC1 && Intensity_Cur.type() == CV_8UC1); // we only support three types of image information for now
	assert(Intensity_Ref.size().width == vx.width() && Intensity_Ref.size().height == vx.height());
	assert(Intensity_Cur.size().width == vy.width() && Intensity_Cur.size().height == vy.height());
	DImage Im1(Intensity_Ref.size().width,Intensity_Ref.size().height,1);
	DImage Im2(Intensity_Cur.size().width,Intensity_Cur.size().height,1);
	ImageIO::loadcvMat(Intensity_Ref,Im1.pData);
	ImageIO::loadcvMat(Intensity_Cur,Im2.pData);
	//cout << Im1.pData[0] << endl;
	//cout << Im1.pData[640*480-1] << endl;
	/*
	ofstream o_file1,o_file2;
	o_file1.open("Im1_data.txt");
	o_file2.open("Im2_data.txt");
	int length = Im1.width()*Im1.height();
	for(int i = 0;i<length;i++)
	{
	    o_file1 << Im1.pData[i];
	    o_file2 << Im2.pData[i];
	    if((i+1)%Im1.width()==0)
        {
            o_file1 << "\n";
            o_file2 << "\n";
        }
        else
        {
            o_file1 << " ";
            o_file2 << " ";
        }
	}
	o_file1.close();
	o_file2.close();
	*/
    assert(Im1.matchDimension(Im2));
	// get the parameters
	double alpha= 0.012;
	double ratio=0.75;
	int minWidth= 20;
	int nOuterFPIterations = 7;
	int nInnerFPIterations = 1;
	int nSORIterations= 30;

	OpticalFlow::Coarse2FineFlow(vx,vy,warpI2,Im1,Im2,alpha,ratio,minWidth,nOuterFPIterations,nInnerFPIterations,nSORIterations);
}

void GetWccWithOF(DImage vx, DImage vy, int t_Norm, int t_Dir, cv::Mat& Wcc,int type)
{
    int Width = vx.width();
    int Height = vx.height();
    int length = Width*Height;
    Wcc.create(Height,Width,type);
    float Norm[length];
    float Dir[length];

    double Avg_Norm = 0;
    double Avg_Dir = 0;
    for(int i = 0;i < length;i++)
    {
        Norm[i] = sqrt(vx.pData[i]*vx.pData[i] + vy.pData[i]*vy.pData[i]);
        Dir[i] = atan2(vy.pData[i],vx.pData[i]);
        Avg_Norm += Norm[i];
        Avg_Dir += Dir[i];
    }
    Avg_Norm /= length;
    Avg_Dir /= length;


    for(int i = 0;i < length;i++)
    {
        //float a = fabs(Dir.pData[i] - Avg_Dir);
        if (Norm[i] > t_Norm * Avg_Norm && fabs(Dir[i] - Avg_Dir) > t_Dir)
            Wcc.data[i] = 0;
        else
            Wcc.data[i] = 255;
    }
}

void GetWccWithOF(cv::Mat vx, cv::Mat vy, int t_Norm, int t_Dir, cv::Mat& Wcc, int type)
{
    int Width = vx.size().width;
    int Height = vx.size().height;
    int length = Width * Height;
    Wcc.create(Height,Width,type);
    float Norm[length];
    float Dir[length];

    float Avg_Norm = 0;
    float Avg_Dir = 0;

    for(int i = 0;i < length;i++)
    {
        Norm[i] = sqrt(vx.data[i]*vx.data[i] + vy.data[i]*vy.data[i]);
        Dir[i] = atan2(vy.data[i],vx.data[i]);
        Avg_Norm += Norm[i];
        Avg_Dir += Dir[i];
    }
    Avg_Norm /= length;
    Avg_Dir /= length;

    for(int i = 0;i < length;i++)
    {
        //float a = fabs(Dir.pData[i] - Avg_Dir);
        if (Norm[i] < t_Norm * Avg_Norm && fabs(Dir[i] - Avg_Dir) > t_Dir)
            Wcc.data[i] = 0;
        else
            Wcc.data[i] = 255;
    }
}

}

