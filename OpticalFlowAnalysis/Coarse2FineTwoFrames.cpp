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
	if(Intensity_Ref.type()!= CV_8UC1 || Intensity_Cur.type()!= CV_8UC1) // we only support three types of image information for now
		return false;
	DImage Im1,Im2;
	ImageIO::loadcvMat(Intensity_Ref,Im1.pData);
	ImageIO::loadcvMat(Intensity_Cur,Im2.pData);

    assert(Im1.matchDimension(Im2));
	// get the parameters
	double alpha= 1;
	double ratio=0.5;
	int minWidth= 40;
	int nOuterFPIterations = 3;
	int nInnerFPIterations = 1;
	int nSORIterations= 20;

	OpticalFlow::Coarse2FineFlow(vx,vy,warpI2,Im1,Im2,alpha,ratio,minWidth,nOuterFPIterations,nInnerFPIterations,nSORIterations);
}

void GetWccWithOF(DImage vx, DImage vy, int t_Norm, int t_Dir, cv::Mat& Wcc,int type)
{
    int Width = vx.width();
    int Height = vx.height();
    Wcc.create(Height,Width,type);
    DImage Norm(Width,Height);
    DImage Dir(Width,Height);

    double Avg_Norm = 0;
    double Avg_Dir = 0;
    int length = vx.width() * vx.height();
    for(int i = 0;i < length;i++)
    {
        *(Norm.pData + i) = sqrt((*(vx.pData + i))*(*(vx.pData + i)) + (*(vy.pData + i))*(*(vy.pData + i)));
        *(Dir.pData + i) = atan2(*(vy.pData + i),*(vx.pData + i));
        Avg_Norm += *(Norm.pData + i);
        Avg_Dir += *(Dir.pData + i);
    }
    Avg_Norm /= length;
    Avg_Dir /= length;

    int x,y;
    for(int i = 0;i < length;i++)
    {
        y = i/Width;
        x = i%Width;
        if (*(Norm.pData + i) > t_Norm * Avg_Norm && abs(*(Dir.pData + i) - Avg_Dir) > t_Dir)
            *(Wcc.ptr(y,x) + i) = 0;
        else
            *(Wcc.ptr(y,x) + i) = 1;
    }
}
}

