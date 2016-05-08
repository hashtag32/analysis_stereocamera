/*
 *  DUOInput.cpp
 *  Description: Reads the input from the DUO3D stereo camera
 *  Created on: Apr 24, 2016
 *  Author: Alexander Treib
 */

#include "DUOInput.h"

#define WIDTH	320
#define HEIGHT	240
#define FPS		30

DUOInput::DUOInput()
{

}

DUOInput::~DUOInput()
{

}

bool DUOInput::opening_duo()
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

bool DUOInput::get_frames(IplImage *input_left, IplImage *input_right, Mat &output_left, Mat &output_right)
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

bool DUOInput::create_all_trackbars(std::string windowname)
{
	createTrackbar("m_SADWindowSize", windowname, &m_SADWindowSize, m_trackbar_samples);
	createTrackbar("m_numberOfDisparities", windowname, &m_numberOfDisparities, m_trackbar_samples);
	createTrackbar("m_preFilterCap", windowname, &m_preFilterCap, m_trackbar_samples);
	createTrackbar("m_minDisparity", windowname, &m_minDisparity, m_trackbar_samples);
	createTrackbar("m_uniquenessRatio", windowname, &m_uniquenessRatio, m_trackbar_samples);
	createTrackbar("m_speckleWindowSize", windowname, &m_speckleWindowSize, m_trackbar_samples);
	createTrackbar("m_speckleRange", windowname, &m_speckleRange, m_trackbar_samples);
	createTrackbar("m_disp12MaxDiff", windowname, &m_disp12MaxDiff, m_trackbar_samples);
	createTrackbar("m_P1", windowname, &m_P1, m_trackbar_samples);
	createTrackbar("m_P2", windowname, &m_P2, m_trackbar_samples);
	return 1;
}

StereoSGBM DUOInput::sgbm_settings()
{
	StereoSGBM sgbm;

	sgbm.SADWindowSize = m_SADWindowSize;  //range 0 to 100
	sgbm.numberOfDisparities = m_numberOfDisparities;// 20 * 16 * m_numberOfDisparities / m_trackbar_samples; //range 10 to 330; divisible by 16
	sgbm.preFilterCap = m_preFilterCap;
	sgbm.minDisparity = m_minDisparity; //200 * m_minDisparity / m_trackbar_samples - 200;
	sgbm.uniquenessRatio = m_uniquenessRatio;
	sgbm.speckleWindowSize = m_speckleWindowSize;
	sgbm.speckleRange = m_speckleRange;
	sgbm.disp12MaxDiff = m_disp12MaxDiff;
	sgbm.fullDP = m_fullDP;
	sgbm.P1 = m_P1;
	sgbm.P2 = m_P2;

	return sgbm;
}


int main(int argc, char* argv[])
{
	DUOInput obj;
	obj.opening_duo();

	// Create image headers for left & right frames
	IplImage *left = cvCreateImageHeader(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 1);
	IplImage *right = cvCreateImageHeader(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 1);

	// Set exposure and LED brightness
	SetGain(100);
	SetExposure(100);

	//create all trackbars for sgbm
	std::string windowname = "Trackbars";
	namedWindow(windowname, 0);
	//obj.create_all_trackbars(windowname);

	// Run capture loop until <Esc> key is pressed
	while ((cvWaitKey(1) & 0xff) != 27)
	{
		Mat imgLeft, imgRight, disp, disp_n;

		obj.get_frames(left, right, imgLeft, imgRight);

		//readin the settings from sgbm_settings
		StereoSGBM sgbm = obj.sgbm_settings();

		//Calculate the disparity and normalize it (due to CV_16)
		sgbm(imgLeft, imgRight, disp);
		normalize(disp, disp_n, 0, 255, CV_MINMAX, CV_8U);

		//Create Window
		cout << obj.m_SADWindowSize << endl;
		//imshow("left", imgLeft);
		//imshow("right", imgRight);
		imshow("DUOInput", disp_n);
	}

	cvReleaseImageHeader(&left);
	cvReleaseImageHeader(&right);
	CloseDUOCamera();
	return 0;
}
