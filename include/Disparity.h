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
	bool go(Mat &imgLeft, Mat &imgRight, Mat &disp8);

	//members
	int m_trackbar_samples = 100;
	//SGBM default values
	//parameters to variate
	int m_SADWindowSize = 6;	//smoothness range: 5-15
	int m_numberOfDisparities = 32;	//distance sensitivity 16-32-64 normally not more

	int m_preFilterCap = 41;
	int m_minDisparity = -64;
	int m_uniquenessRatio = 1;
	int m_speckleWindowSize = 0;
	int m_speckleRange = 0;
	int m_disp12MaxDiff = 10;
	int m_fullDP = true;
	int m_P1 = 1500;
	int m_P2 = 2400;
};

