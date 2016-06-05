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

bool DUOInput::go(Mat &imgLeft, Mat &imgRight, bool lastrequest)
{
	//Open the camera
	this->opening_duo();

	// Create image headers for left & right frames
	IplImage *left = cvCreateImageHeader(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 1);
	IplImage *right = cvCreateImageHeader(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 1);

	// Set exposure and LED brightness
	SetGain(20);
	SetExposure(70);

	// Get the actual frames
	this->get_frames(left, right, imgLeft, imgRight);

	if (lastrequest)
	{
		cvReleaseImageHeader(&left);
		cvReleaseImageHeader(&right);
		CloseDUOCamera();
	}
	return 1;
}

