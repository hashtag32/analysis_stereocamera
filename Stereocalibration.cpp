/*
*  Stereocalibration.cpp
*  Description: Determines the Camera matrix and distortion coefficients on the basis of the /calibimg 
*  Not used in realtime
*  Created on: Mai 8, 2016
*  Author: Alexander Treib
*/
//https://github.com/daviddoria/Examples/blob/master/c%2B%2B/OpenCV/StereoCalibration/orig.cxx
#include "Stereocalibration.h"


Stereocalibration::Stereocalibration()
{

	m_cameraMatrix[0] = Mat::eye(3, 3, CV_64F);
	m_cameraMatrix[1] = Mat::eye(3, 3, CV_64F);
	m_intrinsic_filename = "calibration/intrinsics.yml";
	m_extrinsic_filename = "calibration/extrinsics.yml";
}


Stereocalibration::~Stereocalibration()
{
}

void Stereocalibration::computecoefficients(const vector<string>& imagelist, Size boardSize)
{
	if (imagelist.size() % 2 != 0)
	{
		cout << "Error: the image list contains odd (non-even) number of elements\n";
		return;
	}

	bool displayCorners = false;//true;
	const int maxScale = 2;
	const float squareSize = 1.f;  // Set this to your actual square size

	// ARRAY AND VECTOR STORAGE:
	int i, j, k, m_nimages = (int)imagelist.size() / 2;

	m_imagePoints[0].resize(m_nimages);
	m_imagePoints[1].resize(m_nimages);
	

	for (i = j = 0; i < m_nimages; i++)
	{
		for (k = 0; k < 2; k++)
		{
			const string& filename = imagelist[i * 2 + k];
			cout << filename << endl;
			Mat img = imread(filename, 0);
			if (img.empty())
				break;
			if (m_imageSize == Size())
				m_imageSize = img.size();
			else if (img.size() != m_imageSize)
			{
				cout << "The image " << filename << " has the size different from the first image size. Skipping the pair\n";
				break;
			}
			bool found = false;
			vector<Point2f>& corners = m_imagePoints[k][j];
			for (int scale = 1; scale <= maxScale; scale++)
			{
				Mat timg;
				if (scale == 1)
					timg = img;
				else
					resize(img, timg, Size(), scale, scale);
				found = findChessboardCorners(timg, boardSize, corners,
					CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
				if (found)
				{
					if (scale > 1)
					{
						Mat cornersMat(corners);
						cornersMat *= 1. / scale;
					}
					break;
				}
			}
			if (displayCorners)
			{
				cout << filename << endl;
				Mat cimg, cimg1;
				cvtColor(img, cimg, COLOR_GRAY2BGR);
				drawChessboardCorners(cimg, boardSize, corners, found);
				double sf = 640. / MAX(img.rows, img.cols);
				resize(cimg, cimg1, Size(), sf, sf);
				imshow("corners", cimg1);
				char c = (char)waitKey(500);
				if (c == 27 || c == 'q' || c == 'Q') //Allow ESC to quit
					exit(-1);
			}
			else
				putchar('.');
			if (!found)
				break;
			cornerSubPix(img, corners, Size(11, 11), Size(-1, -1),
				TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS,
				30, 0.01));
		}
		if (k == 2)
		{
			m_goodImageList.push_back(imagelist[i * 2]);
			m_goodImageList.push_back(imagelist[i * 2 + 1]);
			j++;
		}
	}
	cout << j << " pairs have been successfully detected.\n";
	m_nimages = j;
	if (m_nimages < 2)
	{
		cout << "Error: too little pairs to run the calibration\n";
		return;
	}

	m_imagePoints[0].resize(m_nimages);
	m_imagePoints[1].resize(m_nimages);
	m_objectPoints.resize(m_nimages);

	for (i = 0; i < m_nimages; i++)
	{
		for (j = 0; j < boardSize.height; j++)
			for (k = 0; k < boardSize.width; k++)
				m_objectPoints[i].push_back(Point3f(k*squareSize, j*squareSize, 0));
	}

	cout << "Running stereo calibration ...\n";


	double rms = stereoCalibrate(m_objectPoints, m_imagePoints[0], m_imagePoints[1],
		m_cameraMatrix[0], m_distCoeffs[0],
		m_cameraMatrix[1], m_distCoeffs[1],
		m_imageSize, m_R, m_T, m_E, m_F,
		TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 100, 1e-5));
	/*CV_CALIB_RATIONAL_MODEL +
	CV_CALIB_FIX_K3 + CV_CALIB_FIX_K4 + CV_CALIB_FIX_K5);*/
	cout << "done with RMS error=" << rms << endl;
}

void Stereocalibration::save_coefficients()
{
	// save intrinsic parameters
	FileStorage fs(m_intrinsic_filename, CV_STORAGE_WRITE);
	if (fs.isOpened())
	{
		fs << "M1" << m_cameraMatrix[0] << "D1" << m_distCoeffs[0] <<
			"M2" << m_cameraMatrix[1] << "D2" << m_distCoeffs[1];
		fs.release();
	}
	else
		cout << "Error: can not save the intrinsic parameters\n";

	stereoRectify(m_cameraMatrix[0], m_distCoeffs[0],
		m_cameraMatrix[1], m_distCoeffs[1],
		m_imageSize, m_R, m_T, m_R1, m_R2, m_P1, m_P2, m_Q,
		CALIB_ZERO_DISPARITY, 1, m_imageSize, &m_validRoi[0], &m_validRoi[1]);

	fs.open(m_extrinsic_filename, CV_STORAGE_WRITE);
	if (fs.isOpened())
	{
		fs << "R" << m_R << "T" << m_T << "R1" << m_R1 << "R2" << m_R2 << "m_P1" << m_P1 << "P2" << m_P2 << "Q" << m_Q;
		fs.release();
	}
	else
		cout << "Error: can not save the extrinsic parameters\n";
}

void Stereocalibration::checkquality(bool useCalibrated , bool showRectified )
{
	// CALIBRATION QUALITY CHECK
	// because the output fundamental matrix implicitly
	// includes all the output information,
	// we can check the quality of calibration using the
	// epipolar geometry constraint: m2^t*F*m1=0
	int i, j, k;
	double err = 0;
	int npoints = 0;
	vector<Vec3f> lines[2];
	for (i = 0; i < m_nimages; i++)
	{
		int npt = (int)m_imagePoints[0][i].size();
		Mat imgpt[2];
		for (k = 0; k < 2; k++)
		{
			imgpt[k] = Mat(m_imagePoints[k][i]);
			undistortPoints(imgpt[k], imgpt[k], m_cameraMatrix[k], m_distCoeffs[k], Mat(), m_cameraMatrix[k]);
			computeCorrespondEpilines(imgpt[k], k + 1, m_F, lines[k]);
		}
		for (j = 0; j < npt; j++)
		{
			double errij = fabs(m_imagePoints[0][i][j].x*lines[1][j][0] +
				m_imagePoints[0][i][j].y*lines[1][j][1] + lines[1][j][2]) +
				fabs(m_imagePoints[1][i][j].x*lines[0][j][0] +
				m_imagePoints[1][i][j].y*lines[0][j][1] + lines[0][j][2]);
			err += errij;
		}
		npoints += npt;
	}
	cout << "average reprojection err = " << err / npoints << endl;



	// OpenCV can handle left-right
	// or up-down camera arrangements
	bool isVerticalStereo = fabs(m_P2.at<double>(1, 3)) > fabs(m_P2.at<double>(0, 3));

	// COMPUTE AND DISPLAY RECTIFICATION
	if (!showRectified)
		return;

	Mat rmap[2][2];
	// IF BY CALIBRATED (BOUGUET'S METHOD)
	if (useCalibrated)
	{
		// we already computed everything
	}
	else//  HARTLEY'S METHOD
		// use intrinsic parameters of each camera, but
		// compute the rectification transformation directly
		// from the fundamental matrix
	{
		vector<Point2f> allimgpt[2];
		for (k = 0; k < 2; k++)
		{
			for (i = 0; i < m_nimages; i++)
				std::copy(m_imagePoints[k][i].begin(), m_imagePoints[k][i].end(), back_inserter(allimgpt[k]));
		}
		m_F = findFundamentalMat(Mat(allimgpt[0]), Mat(allimgpt[1]), FM_8POINT, 0, 0);
		Mat H1, H2;
		stereoRectifyUncalibrated(Mat(allimgpt[0]), Mat(allimgpt[1]), m_F, m_imageSize, H1, H2, 3);

		m_R1 = m_cameraMatrix[0].inv()*H1*m_cameraMatrix[0];
		m_R2 = m_cameraMatrix[1].inv()*H2*m_cameraMatrix[1];
		m_P1 = m_cameraMatrix[0];
		m_P2 = m_cameraMatrix[1];
	}

	//Precompute maps for cv::remap()
	initUndistortRectifyMap(m_cameraMatrix[0], m_distCoeffs[0], m_R1, m_P1, m_imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
	initUndistortRectifyMap(m_cameraMatrix[1], m_distCoeffs[1], m_R2, m_P2, m_imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);

	Mat canvas;
	double sf;
	int w, h;
	if (!isVerticalStereo)
	{
		sf = 600. / MAX(m_imageSize.width, m_imageSize.height);
		w = cvRound(m_imageSize.width*sf);
		h = cvRound(m_imageSize.height*sf);
		canvas.create(h, w * 2, CV_8UC3);
	}
	else
	{
		sf = 300. / MAX(m_imageSize.width, m_imageSize.height);
		w = cvRound(m_imageSize.width*sf);
		h = cvRound(m_imageSize.height*sf);
		canvas.create(h * 2, w, CV_8UC3);
	}

	for (i = 0; i < m_nimages; i++)
	{
		for (k = 0; k < 2; k++)
		{
			Mat img = imread(m_goodImageList[i * 2 + k], 0), rimg, cimg;
			remap(img, rimg, rmap[k][0], rmap[k][1], CV_INTER_LINEAR);
			cvtColor(rimg, cimg, COLOR_GRAY2BGR);
			Mat canvasPart = !isVerticalStereo ? canvas(Rect(w*k, 0, w, h)) : canvas(Rect(0, h*k, w, h));
			resize(cimg, canvasPart, canvasPart.size(), 0, 0, CV_INTER_AREA);
			if (useCalibrated)
			{
				Rect vroi(cvRound(m_validRoi[k].x*sf), cvRound(m_validRoi[k].y*sf),
					cvRound(m_validRoi[k].width*sf), cvRound(m_validRoi[k].height*sf));
				rectangle(canvasPart, vroi, Scalar(0, 0, 255), 3, 8);
			}
		}

		if (!isVerticalStereo)
			for (j = 0; j < canvas.rows; j += 16)
				line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1, 8);
		else
			for (j = 0; j < canvas.cols; j += 16)
				line(canvas, Point(j, 0), Point(j, canvas.rows), Scalar(0, 255, 0), 1, 8);
		imshow("rectified", canvas);
		char c = (char)waitKey();
		if (c == 27 || c == 'q' || c == 'Q')
			break;
	}
}



//support function
bool writeImageList(const string& filename, vector<string>& imagelist)
{
	imagelist.resize(0);
	for (int i = 0; i < 20; i++)
	{
		char buffer[50];

		sprintf(buffer, "calibimg/imgLeft%d.jpg", i);
		imagelist.push_back(buffer);

		sprintf(buffer, "calibimg/imgRight%d.jpg", i);
		imagelist.push_back(buffer);
	}
	
	return true;
}

bool Stereocalibration::go(Mat &imgLeft,Mat &imgRight)
{
	Size boardSize;
	string imagelistfn;
	bool showRectified = true;

	boardSize = Size(9, 6);
	imagelistfn = "stereo_calib.xml";

	vector<string> imagelist;
	bool ok = writeImageList(imagelistfn, imagelist);
	if (!ok || imagelist.empty())
	{
		cout << "can not open " << imagelistfn << " or the string list is empty" << endl;
		return 0;
	}

	this->computecoefficients(imagelist, boardSize);
	this->save_coefficients();
	this->checkquality(true, showRectified);

	return 1;
}

































//inserted existing code

//bool Stereocalibration::go()
//{
//	return 1;
//}
