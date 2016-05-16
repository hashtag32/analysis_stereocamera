/*
*  Stereocalibration.h
*  Description: Calibrates and rectifies the images of the stereocamera
*  Created on: Mai 8, 2016
*  Author: Alexander Treib
*/
#include"externalheader.h"

#pragma once
class Stereocalibration
{
public:
	Stereocalibration();
	~Stereocalibration();
	//functions
	void computecoefficients(const vector<string>& imagelist, Size boardSize);
	void save_coefficients();
	void checkquality(bool useCalibrated = true, bool showRectified = true);

	bool go(Mat &imgLet, Mat &imgRight);

	//members
	vector<vector<Point2f> > m_imagePoints[2];
	vector<vector<Point3f> > m_objectPoints;
	Mat m_cameraMatrix[2], m_distCoeffs[2];
	vector<string> m_goodImageList;
	Size m_imageSize;
	int m_nimages;
	int m_nimages_size;
	Mat m_R, m_T, m_E, m_F, m_P1, m_P2;
	Mat m_R1, m_R2,  m_Q;
	Rect m_validRoi[2];
	int test;
	std::string m_intrinsic_filename;
	std::string m_extrinsic_filename;
};

