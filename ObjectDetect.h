/*
*  ObjectDetect.hpp
*  Description: Objectdetection based on the depth image obtained by the Reprojection3D
*  Created on: Mai 8, 2016
*  Author: Alexander Treib
*/

#include"externalheader.h"

#pragma once
class ObjectDetect
{
public:
	virtual bool function()=0;
	virtual ~ObjectDetect();

	//Implementation in cpp
	//bool function()
	//{
	//	std::cout << "function" << endl;
	//}


};

