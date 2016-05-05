/*
 *  DepthImage.cpp
 *
 *  Created on: Apr 24, 2016
 *  Author: Alexander Treib
 */

#include "DepthImage.h"

#define WIDTH	320
#define HEIGHT	240
#define FPS		30

DepthImage::DepthImage()
{

}

DepthImage::~DepthImage()
{

}

bool DepthImage::opening_duo()
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

bool DepthImage::get_frames(IplImage *input_left, IplImage *input_right, Mat &output_left, Mat &output_right)
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

bool DepthImage::create_all_trackbars(std::string windowname)
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

bool DepthImage::sgbm_update(StereoSGBM &sgbm)
{
	//Einflusnehmende: SADWindowSize,numberOfDisparities,minDisparity,P1,P2


	//Formula to calculate ranges (due to trackbar): var=(max-min)*(trackvar/trackbar_samples)+min
	sgbm.SADWindowSize = 100 * m_SADWindowSize / m_trackbar_samples;  //range 0 to 100
	
	sgbm.numberOfDisparities = 16*6;// 20 * 16 * m_numberOfDisparities / m_trackbar_samples; //range 10 to 330; divisible by 16
	sgbm.preFilterCap = 5 * m_preFilterCap / m_trackbar_samples;
	sgbm.minDisparity = m_minDisparity; //200 * m_minDisparity / m_trackbar_samples - 200;
	sgbm.uniquenessRatio = 20 * m_uniquenessRatio / m_trackbar_samples + 1;
	sgbm.speckleWindowSize = 150 * m_speckleWindowSize / m_trackbar_samples + 50;
	sgbm.speckleRange = 1;
	sgbm.disp12MaxDiff = 40 * m_disp12MaxDiff / m_trackbar_samples - 10;
	sgbm.fullDP = m_fullDP;
	sgbm.P1 = 1000 * m_P1 / m_trackbar_samples + 1000;
	sgbm.P2 = 1000 * m_P2 / m_trackbar_samples + 2001;
	return 1;
}



int main(int argc, char* argv[])
{
	DepthImage obj;
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
	obj.create_all_trackbars(windowname);

	// Run capture loop until <Esc> key is pressed
	while ((cvWaitKey(1) & 0xff) != 27)
	{
		Mat imgLeft, imgRight, disp, disp_n;

		StereoSGBM sgbm;

		obj.get_frames(left, right, imgLeft, imgRight);

		//readin the settings from sgbm_settings
		obj.sgbm_update(sgbm);

		//Calculate the disparity and normalize it (due to CV_16)
		sgbm(imgLeft, imgRight, disp);
		normalize(disp, disp_n, 0, 255, CV_MINMAX, CV_8U);

		//Create Window
		cout << obj.m_SADWindowSize << endl;
		//imshow("left", imgLeft);
		//imshow("right", imgRight);
		imshow("DepthImage", disp_n);
	}

	cvReleaseImageHeader(&left);
	cvReleaseImageHeader(&right);
	CloseDUOCamera();
	return 0;
}
