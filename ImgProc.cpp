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


int main()
{
	ImgProc imgproc_obj;
	Mat imgLeft, imgRight;
	imgproc_obj.duoinput_obj.go(imgLeft,imgRight,0);
	
	
	while ((cvWaitKey(1) & 0xff) != 27)
	{
		imshow("imgLeft", imgLeft);
	}

	imgproc_obj.stereocalibration_obj.go();
	/*imgproc_obj.disparity_obj.go();
	imgproc_obj.reporjection3D_obj.go();
	imgproc_obj.objectdetect_obj.go();*/
}