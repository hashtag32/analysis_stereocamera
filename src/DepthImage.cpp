#include "DepthImage.h"

#define WIDTH	320
#define HEIGHT	240
#define FPS		30

bool opening_duo()
{
	// Open DUO camera and start capturing
	if (!OpenDUOCamera(WIDTH, HEIGHT, FPS))
	{
		printf("Could not open DUO camera\n");
		return 0;
	}
	printf("\nHit <ESC> to exit.\n");
	return 1;
}

bool sgbm_settings(StereoSGBM &sgbm)
{
	sgbm.SADWindowSize = 5;
	sgbm.numberOfDisparities = 192;
	sgbm.preFilterCap = 4;
	sgbm.minDisparity = -64;
	sgbm.uniquenessRatio = 1;
	sgbm.speckleWindowSize = 150;
	sgbm.speckleRange = 2;
	sgbm.disp12MaxDiff = 10;
	sgbm.fullDP = true;
	sgbm.P1 = 1500;
	sgbm.P2 = 2400;
	return 1;
}

bool get_frames(IplImage *input_left, IplImage *input_right, Mat &output_left, Mat &output_right)
{
	// Capture DUO frame
	PDUOFrame pFrameData = GetDUOFrame();
	if (pFrameData == NULL)
	{
		printf("No image. Is the camera connected?");
		return 0;
	}
		
	// Set the image data
	input_left->imageData = (char*)pFrameData->leftData;
	input_right->imageData = (char*)pFrameData->rightData;

	output_left = Mat(input_left, 0);
	output_right = Mat(input_right, 0);
	return 1;
}

int main(int argc, char* argv[])
{
	opening_duo();

	// Create image headers for left & right frames
	IplImage *left = cvCreateImageHeader(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 1);
	IplImage *right = cvCreateImageHeader(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 1);

	// Set exposure and LED brightness
	SetGain(100);
	SetExposure(100);

	// Run capture loop until <Esc> key is pressed
	while ((cvWaitKey(1) & 0xff) != 27)
	{
		Mat imgLeft, imgRight;
		get_frames(left,right,imgLeft,imgRight);
		
		Mat disp_n, disp;
		StereoSGBM sgbm;
		//readin the settings from sgbm_settings
		sgbm_settings(sgbm);
		
		//Calculate the disparity and normalize it (due to CV_16)
		sgbm(imgLeft, imgRight, disp);
		normalize(disp, disp_n, 0, 255, CV_MINMAX, CV_8U);

		//Display result
		imshow("left", imgLeft);
		imshow("right", imgRight);
		imshow("disp", disp_n);
	}

	cvReleaseImageHeader(&left);
	cvReleaseImageHeader(&right);
	CloseDUOCamera();
	return 0;
}
