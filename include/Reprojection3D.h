/*
*  Reprojection3D.h
*  Description: Reproject a disparity image to 3D space (input: dispimage, Q-Matrix)
*  Created on: Mai 8, 2016
*  Author: Alexander Treib
*/

#include"externalheader.h"
#include "Disparity.h"

#pragma once
class Reprojection3D
{
public:
	Reprojection3D();
	~Reprojection3D();

	//functions
	bool readin();
	void save_pointcloud(const Mat& mat);
	bool go(Mat &img1, Mat &img2, Mat &disp8);

	//members
	Disparity disp_obj;
	const char* m_calibration_filename;
	const char* m_rectification_filename;
	const char* m_disparity_filename;
	const char* m_point_cloud_filename;
	int m_SADWindowSize;
	int m_numberOfDisparities;
	int m_alg;
	Rect m_roi1, m_roi2;
	Mat m_Q, m_M1, m_D1, m_M2, m_D2, m_R, m_T, m_R1, m_P1, m_R2, m_P2;

	enum { STEREO_BM = 0, STEREO_SGBM = 1, STEREO_HH = 2, STEREO_VAR = 3 };
};

