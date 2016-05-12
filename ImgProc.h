/*
*  ImgProc.h
*  Description: Overall function the contains members of all other classes
*  Created on: Mai 8, 2016
*  Author: Alexander Treib
*/

#include "externalheader.h"

#include "DUOInput.h"
#include "Stereocalibration.h"
#include "Disparity.h"
#include "Reprojection3D.h"
#include "ObjectDetect.h"

#pragma once
class ImgProc
{
public:
	ImgProc();
	~ImgProc();
public:
	DUOInput duoinput_obj;
	Stereocalibration stereocalibration_obj;
	Disparity disparity_obj;
	Reprojection3D reprojection3D_obj;
	ObjectDetect objectdetect_obj;
};

