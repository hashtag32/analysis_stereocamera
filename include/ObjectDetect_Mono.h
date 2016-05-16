/*
*  ObjectDetect_Mono.h
*  Description: ObjectDetection based on the webcam image
*  interface is ObjectDetect
*  Created on: Mai 13, 2016
*  Author: Alexander Treib
*/

#include"externalheader.h"
#include"ObjectDetect.h"

#pragma once
class ObjectDetect_Mono:public ObjectDetect
{
public:
	ObjectDetect_Mono();
	~ObjectDetect_Mono();
	bool go();
};

