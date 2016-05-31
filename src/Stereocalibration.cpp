/*
*  Stereocalibration.cpp
*  Description: Determines the Camera matrix and distortion coefficients on the basis of the /calibimg
*  Not used in realtime
*  Created on: Mai 8, 2016
*  Author: Alexander Treib
*/

#include "Stereocalibration.h"

Stereocalibration::Stereocalibration()
{
	m_cameraMatrix[0] = Mat::eye(3, 3, CV_64F);
	m_cameraMatrix[1] = Mat::eye(3, 3, CV_64F);
	m_calibration_filename = "calibration/calibration.yml";
	m_rectification_filename = "calibration/rectification.yml";
	m_boardSize = Size(9, 6);
}

Stereocalibration::~Stereocalibration()
{
}

//calibration path
bool Stereocalibration::recordCalibImages(Mat &imgLeft, Mat &imgRight)
{
	//requires opening the camera (In ImgProc call DUOInput.go())

	for (int j = 0; j < 100; j++)
	{
		char buffer[50];
		sprintf(buffer, "calibimg/imgLeft%d.jpg", j);
		cout << buffer << endl;
		imwrite(buffer, imgLeft);
		sprintf(buffer, "calibimg/imgRight%d.jpg", j);
		imwrite(buffer, imgRight);

		std::cout << "countdown:\n";
		for (int i = 1; i > 0; --i) {
			std::cout << i << std::endl;
			std::this_thread::sleep_for(std::chrono::seconds(1));
		}
	}
	return 1;
}

void Stereocalibration::computecoefficients()
{
	vector<string> imagelist;
	bool ok = this->loadImageList(imagelist);

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
				found = findChessboardCorners(timg, m_boardSize, corners,
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
				drawChessboardCorners(cimg, m_boardSize, corners, found);
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
	m_nimages_size = m_nimages;
	for (i = 0; i < m_nimages; i++)
	{
		for (j = 0; j < m_boardSize.height; j++)
			for (k = 0; k < m_boardSize.width; k++)
				m_objectPoints[i].push_back(Point3f(k*squareSize, j*squareSize, 0));
	}

	cout << "Running stereo calibration ...\n";


	double rms = stereoCalibrate(m_objectPoints, m_imagePoints[0], m_imagePoints[1],
		m_cameraMatrix[0], m_distCoeffs[0],
		m_cameraMatrix[1], m_distCoeffs[1],
		m_imageSize, m_R, m_T, m_E, m_F,
		TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 100, 1e-5),
		CV_CALIB_ZERO_TANGENT_DIST + CV_CALIB_SAME_FOCAL_LENGTH + CV_CALIB_FIX_K3 +
		CV_CALIB_FIX_K4);

	cout << "done with RMS error=" << rms << endl;
	int test_var = 3;
	test = test_var;
	cout << test << endl;

	//save intrinsic parameters
	this->write_calibrationFiles(1, 0);
}

void Stereocalibration::rectification()
{
	//readin the calibration parameters to calibration.yml
	this->readin(true, false);
	//Rectification
	stereoRectify(m_cameraMatrix[0], m_distCoeffs[0],
		m_cameraMatrix[1], m_distCoeffs[1],
		m_imageSize, m_R, m_T, m_R1, m_R2, m_P1, m_P2, m_Q,
		CALIB_ZERO_DISPARITY, 1, m_imageSize, &m_validRoi[0], &m_validRoi[1]);
	//write the rectification parameters to rectification.yml
	this->write_calibrationFiles(false, true);
}

void Stereocalibration::checkquality(bool useCalibrated, bool showRectified)
{
	//reading intrinsic parameters
	this->readin(true, true);
	// CALIBRATION QUALITY CHECK
	// because the output fundamental matrix implicitly
	// includes all the output information,
	// we can check the quality of calibration using the
	// epipolar geometry constraint: m2^t*F*m1=0
	int i, j, k;
	double err = 0;
	int npoints = 0;
	vector<Vec3f> lines[2];
	cout << m_nimages_size << endl;
	for (i = 0; i < m_nimages_size; i++)
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
	cout << err << "points" << npoints << endl;
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
			for (i = 0; i < m_nimages_size; i++)
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

	for (i = 0; i < m_nimages_size; i++)
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

bool Stereocalibration::write_calibrationFiles(bool calibration, bool rectification)
{
	// save calibration parameters
	if (calibration)
	{
		FileStorage fs(m_calibration_filename, CV_STORAGE_WRITE);
		if (fs.isOpened())
		{
			fs << "M1" << m_cameraMatrix[0] << "D1" << m_distCoeffs[0] <<
				"M2" << m_cameraMatrix[1] << "D2" << m_distCoeffs[1] << "R" << m_R << "T" << m_T << "E" << m_E << "F" << m_F;
			fs.release();
		}
		else{
			cout << "Error: can not save the intrinsic parameters\n";
			return 0;
		}
	}

	//writing rectification parametrers
	if (rectification)
	{
		FileStorage fs(m_rectification_filename, CV_STORAGE_WRITE);
		if (fs.isOpened())
		{
			fs << "R" << m_R << "T" << m_T << "R1" << m_R1 << "R2" << m_R2 << "P1" << m_P1 << "P2" << m_P2 << "Q" << m_Q;
			fs.release();
		}
		else{
			cout << "Error: can not save the extrinsic parameters\n";
			return 0;
		}
	}
	return 1;
}


//undistortion and rectification path (gothrough path)
bool Stereocalibration::readin(bool calibration, bool rectification)
{
	//reading intrinsic parameters
	if (calibration)
	{
		cout << "calibration" << endl;
		FileStorage fs(m_calibration_filename, CV_STORAGE_READ);
		cout << "calibraiton success" << endl;
		if (!fs.isOpened())
		{
			printf("Failed to open file %s\n", m_calibration_filename);
			return 0;
		}

		fs["M1"] >> m_M1;
		fs["D1"] >> m_D1;
		fs["M2"] >> m_M2;
		fs["D2"] >> m_D2;
		fs["R"] >> m_R;
		fs["T"] >> m_T;
		fs["E"] >> m_E;
		fs["F"] >> m_F;
		m_M1 *= 1.5;
		m_M2 *= 1.5;
		fs.release();
	}


	//reading extrinsic parameters
	if (rectification)
	{
		FileStorage fs(m_rectification_filename, CV_STORAGE_READ);
		if (!fs.isOpened())
		{
			printf("Failed to open file %s\n", m_rectification_filename);
			return 0;
		}
		fs["R1"] >> m_R1;
		fs["R2"] >> m_R2;
		fs["P1"] >> m_P1;
		fs["P2"] >> m_P2;
		fs["T"] >> m_T;
		fs["Q"] >> m_Q;
		fs.release();
	}
	return true;
}

bool Stereocalibration::undistort_and_rectify(Mat &img1, Mat &img2)
{
	this->readin(true, true);
	Size img_size = img1.size();
	//stereoRectify(m_M1, m_D1, m_M2, m_D2, img_size, m_R, m_T, m_R1, m_R2, m_P1, m_P2, m_Q, CALIB_ZERO_DISPARITY, -1, img_size, &m_roi1, &m_roi2);

	Mat map11, map12, map21, map22;
	Mat rmap[2][2];
	//initUndistortRectifyMap(m_cameraMatrix[0], m_distCoeffs[0], m_R1, m_P1, m_imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
	initUndistortRectifyMap(m_M1, m_D1, m_R, m_P1, img_size, CV_16SC2, rmap[0][0], rmap[0][1]);
	initUndistortRectifyMap(m_M2, m_D2, m_R, m_P2, img_size, CV_16SC2, rmap[1][0], rmap[1][1]);

	Mat img1r, img2r;
	//emap(img, rimg, rmap[k][0], rmap[k][1], CV_INTER_LINEAR);
	remap(img1, img1r, rmap[0][0], rmap[0][1], CV_INTER_LINEAR);
	remap(img2, img2r, rmap[1][0], rmap[1][1], CV_INTER_LINEAR);

	img1 = img1r;
	img2 = img2r;
	return true;
}


//support function
bool Stereocalibration::loadImageList(vector<string>& imagelist)
{
	imagelist.resize(0);
	for (int i = 0; i < 10; i++)
	{
		char buffer[50];

		sprintf(buffer, "calibimg/imgLeft%d.jpg", i);
		imagelist.push_back(buffer);

		sprintf(buffer, "calibimg/imgRight%d.jpg", i);
		imagelist.push_back(buffer);
	}
	return true;
}

bool Stereocalibration::go(Mat &imgLeft, Mat &imgRight)
{
	//settings for calibration -> default: all false
	bool newimages_calibration = false;		//will overwrite the old images
	bool calibration = true;				//will compute the distortion coefficients -> will be saved in calibration.yml
	bool showRectified_calibration = true;	//show result after calibration
	bool rectification = true;				//will copute the rectification coefficients -> will be saved in rectification.yml
	bool qualitycheck = true;

	if (newimages_calibration == true)
		//writing new images 
		this->recordCalibImages(imgLeft, imgRight);
	if (calibration == true)
		this->computecoefficients();
	if (rectification == true)
		this->rectification();
	if (qualitycheck == true)
		this->checkquality(true, showRectified_calibration);

	//gothrough path
	if ((newimages_calibration == false) && (calibration == false) && (rectification == false) && (qualitycheck == false)){
		//undistort and rectify the images
		this->undistort_and_rectify(imgLeft, imgRight);
	}
	return 1;
}