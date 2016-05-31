/*
*  ImgProc.cpp
*  Description: Overall function the contains members of all other classes
*  Created on: Mai 8, 2016
*  Author: Alexander Treib
*/

#include "ImgProc.h"


ImgProc::ImgProc()
{
}

ImgProc::~ImgProc()
{
}


void main()
{
	ImgProc imgproc_obj;
	Mat inputLeft, inputRight, imgLeft, imgRight, disp;
	//opening the camera (like VideoCapture)
	imgproc_obj.duoinput_obj.go(inputLeft, inputRight, 0);
	int i;
	
	imgproc_obj.stereocalibration_obj.go(inputLeft, inputRight);

	while ((cvWaitKey(100) & 0xff) != 27)
	{
	inputLeft.copyTo(imgLeft);
	inputRight.copyTo(imgRight);
	//imgproc_obj.stereocalibration_obj.go(imgLeft, imgRight);
	imgproc_obj.disparity_obj.go(imgLeft, imgRight,disp);
	//result is saved in pointcloud.ply
	//imgproc_obj.reprojection3D_obj.go(imgLeft, imgRight, disp);
	namedWindow("imgLeft", 0);
	imshow("imgLeft", imgLeft);
	namedWindow("imgRight", 0);
	imshow("imgRight", imgRight);
	namedWindow("disparity", 0);
	imshow("disparity", disp);
	}
	
	//imgproc_obj.disparity_obj.go(imgLeft,imgRight, disp);
	
	imgproc_obj.disparity_obj.go(imgLeft, imgRight, disp);
	//result is saved in pointcloud.ply
	imgproc_obj.reprojection3D_obj.go(imgLeft, imgRight, disp);

	//imgproc_obj.objectdetect_obj_mono.go();
}