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
	bool recordCalibImages(Mat &imgLeft, Mat &imgRight);
	void computecoefficients();
	void rectification();
	void checkquality(bool useCalibrated = true, bool showRectified = true);
	bool write_calibrationFiles(bool calibration, bool rectification);
	bool loadImageList(vector<string>& imagelist);

	bool readin(bool calibration, bool rectification);
	bool undistort_and_rectify(Mat &img1, Mat &img2);

	bool go(Mat &imgLet, Mat &imgRight);
	

	//members
	Size m_boardSize;
	vector<vector<Point2f> > m_imagePoints[2];
	vector<vector<Point3f> > m_objectPoints;
	Mat m_cameraMatrix[2], m_distCoeffs[2];
	vector<string> m_goodImageList;
	Size m_imageSize;
	int m_nimages;
	int m_nimages_size;
	Mat m_Q, m_M1, m_D1, m_M2, m_D2, m_R, m_R1, m_P1, m_R2, m_P2, m_T, m_E, m_F;
	Rect m_validRoi[2];
	int test;
	std::string m_calibration_filename;
	std::string m_rectification_filename;
};

