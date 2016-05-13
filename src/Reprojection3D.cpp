/*
*  Reprojection3D.cpp
*  Description: Reproject a disparity image to 3D space (input: dispimage, Q-Matrix)
*  Created on: Mai 8, 2016
*  Author: Alexander Treib
*/
//http://www.pcl-users.org/reprojectImageTo3D-for-Point-Cloud-from-stereo-Images-td4037071.html
#include "Reprojection3D.h"


//support function
void saveXYZ(const char* filename, const Mat& mat)
{
	const double max_z = 1.0e4;
	FILE* fp = fopen(filename, "wt");
	for (int y = 0; y < mat.rows; y++)
	{
		for (int x = 0; x < mat.cols; x++)
		{
			Vec3f point = mat.at<Vec3f>(y, x);
			if (fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z) continue;
			fprintf(fp, "%f %f %f\n", point[0], point[1], point[2]);
		}
	}
	fclose(fp);
}

Reprojection3D::Reprojection3D()
{
	m_intrinsic_filename = "calibration/intrinsics.yml";
	m_extrinsic_filename = "calibration/extrinsics.yml";
	m_disparity_filename = "results/disparityoutput.jpg";
	m_point_cloud_filename = "results/pointcloud.xyz";
	m_SADWindowSize = 15;
	m_numberOfDisparities = 96;
	m_alg = STEREO_SGBM;
}


Reprojection3D::~Reprojection3D()
{
}

bool Reprojection3D::readin(float scale)
{
	//reading intrinsic parameters
	FileStorage fs(m_intrinsic_filename, CV_STORAGE_READ);
	if (!fs.isOpened())
	{
		printf("Failed to open file %s\n", m_intrinsic_filename);
		return -1;
	}

	fs["M1"] >> m_M1;
	fs["D1"] >> m_D1;
	fs["M2"] >> m_M2;
	fs["D2"] >> m_D2;

	m_M1 *= scale;
	m_M2 *= scale;

	fs.open(m_extrinsic_filename, CV_STORAGE_READ);
	if (!fs.isOpened())
	{
		printf("Failed to open file %s\n", m_extrinsic_filename);
		return -1;
	}

	fs["R"] >> m_R;
	fs["T"] >> m_T;
	return true;
}

bool Reprojection3D::undistort_and_rectify(Mat &img1, Mat &img2, Size img_size)
{
	stereoRectify(m_M1, m_D1, m_M2, m_D2, img_size, m_R, m_T, m_R1, m_R2, m_P1, m_P2, m_Q, CALIB_ZERO_DISPARITY, -1, img_size, &m_roi1, &m_roi2);

	Mat map11, map12, map21, map22;
	initUndistortRectifyMap(m_M1, m_D1, m_R1, m_P1, img_size, CV_16SC2, map11, map12);
	initUndistortRectifyMap(m_M2, m_D2, m_R2, m_P2, img_size, CV_16SC2, map21, map22);

	Mat img1r, img2r;
	remap(img1, img1r, map11, map12, INTER_LINEAR);
	remap(img2, img2r, map21, map22, INTER_LINEAR);

	img1 = img1r;
	img2 = img2r;
	return 0;
}

bool Reprojection3D::Depthimage(Mat &img1, Mat &img2, Mat &disp8)
{
	StereoBM bm;
	StereoSGBM sgbm;
	StereoVar var;

	int numberOfDisparities = m_numberOfDisparities > 0 ? m_numberOfDisparities : ((img1.size().width / 8) + 15) & -16;

	//Block-Matching
	bm.state->roi1 = m_roi1;
	bm.state->roi2 = m_roi2;
	bm.state->preFilterCap = 31;
	bm.state->SADWindowSize = m_SADWindowSize > 0 ? m_SADWindowSize : 9;
	bm.state->minDisparity = 0;
	bm.state->numberOfDisparities = numberOfDisparities;
	bm.state->textureThreshold = 10;
	bm.state->uniquenessRatio = 15;
	bm.state->speckleWindowSize = 100;
	bm.state->speckleRange = 32;
	bm.state->disp12MaxDiff = 1;

	//Semi global matching
	sgbm.preFilterCap = 100;
	sgbm.SADWindowSize = m_SADWindowSize > 0 ? m_SADWindowSize : 3;

	int cn = img1.channels();

	sgbm.P1 = 8 * cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
	sgbm.P2 = 32 * cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
	sgbm.minDisparity = 0;
	sgbm.numberOfDisparities = numberOfDisparities;
	sgbm.uniquenessRatio = 10;
	sgbm.speckleWindowSize = bm.state->speckleWindowSize;
	sgbm.speckleRange = bm.state->speckleRange;
	sgbm.disp12MaxDiff = 1;
	sgbm.fullDP = m_alg == STEREO_HH;

	//StereoVar
	var.levels = 3;                                 // ignored with USE_AUTO_PARAMS
	var.pyrScale = 0.5;                             // ignored with USE_AUTO_PARAMS
	var.nIt = 25;
	var.minDisp = -numberOfDisparities;
	var.maxDisp = 0;
	var.poly_n = 3;
	var.poly_sigma = 0.0;
	var.fi = 15.0f;
	var.lambda = 0.03f;
	var.penalization = var.PENALIZATION_TICHONOV;   // ignored with USE_AUTO_PARAMS
	var.cycle = var.CYCLE_V;                        // ignored with USE_AUTO_PARAMS
	var.flags = var.USE_SMART_ID | var.USE_AUTO_PARAMS | var.USE_INITIAL_DISPARITY | var.USE_MEDIAN_FILTERING;

	Mat disp;

	int64 t = getTickCount();
	if (m_alg == STEREO_BM)
		bm(img1, img2, disp);
	else if (m_alg == STEREO_VAR) {
		var(img1, img2, disp);
	}
	else if (m_alg == STEREO_SGBM || m_alg == STEREO_HH)
		sgbm(img1, img2, disp);
	t = getTickCount() - t;
	printf("Time elapsed: %fms\n", t * 1000 / getTickFrequency());

	//disp = dispp.colRange(numberOfDisparities, img1p.cols);
	if (m_alg != STEREO_VAR)
		disp.convertTo(disp8, CV_8U, 255 / (numberOfDisparities*16.));
	else
		disp.convertTo(disp8, CV_8U);
	return true;
}

bool Reprojection3D::go(Mat &img1, Mat &img2, Mat &disp8)
{
	bool no_display = true;
	float scale = 1.f;

	if (scale != 1.f)
	{
		Mat temp1, temp2;
		int method = scale < 1 ? INTER_AREA : INTER_CUBIC;
		resize(img1, temp1, Size(), scale, scale, method);
		img1 = temp1;
		resize(img2, temp2, Size(), scale, scale, method);
		img2 = temp2;
	}

	this->readin(scale);
	this->undistort_and_rectify(img1, img2, img1.size());
	this->Depthimage(img1, img2, disp8);

	if (!no_display)
	{
		namedWindow("left", 1);
		imshow("left", img1);
		namedWindow("right", 1);
		imshow("right", img2);
		namedWindow("disparity", 0);
		imshow("disparity", disp8);
	}

	if (m_disparity_filename)
		imwrite(m_disparity_filename, disp8);

	if (m_point_cloud_filename)
	{
		Mat xyz;
		//use disp for generating the map!
		reprojectImageTo3D(disp8, xyz, m_Q, true);
		saveXYZ(m_point_cloud_filename, xyz);
	}
	return 1;
}

