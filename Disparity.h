/*
*  Disparity.h
*  Description: Computes a disparity image based on the calibrated, rectified images of the stereo camera
*  Created on: Mai 8, 2016
*  Author: Alexander Treib
*/

#include "externalheader.h"
#pragma once

class Disparity
{
public:
	Disparity();
	~Disparity();

	//functions
	bool create_all_trackbars(std::string windowname);
	StereoSGBM sgbm_settings();
	Mat go(Mat &imgLeft, Mat &imgRight, Mat &Q);

	//members
	int m_trackbar_samples = 100;
	//SGBM default values
	int m_SADWindowSize = 5;
	int m_numberOfDisparities = 192;
	int m_preFilterCap = 4;
	int m_minDisparity = -64;
	int m_uniquenessRatio = 1;
	int m_speckleWindowSize = 150;
	int m_speckleRange = 2;
	int m_disp12MaxDiff = 10;
	int m_fullDP = true;
	int m_P1 = 1500;
	int m_P2 = 2400;
};

