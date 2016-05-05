/*
 *  DepthImage.h
 *
 *  Created on: Apr 24, 2016
 *  Author: Alexander Treib
 */

#ifndef DEPTHIMAGE_H
#define DEPTHIMAGE_H


// Include some generic header files
#if defined(WIN32)
	#include <SDKDDKVer.h>
	#include <windows.h>
	#include <stdio.h>
	#include <conio.h>
#elif defined(__linux__) || defined(__APPLE__)
	#include <stdio.h>
	#include <termios.h>
	#include <unistd.h>
	#include <fcntl.h>
	static struct termios _old, _new;
	/* Initialize new terminal i/o settings */
	void initTermios(int echo) 
	{
	  tcgetattr(0, &_old); /* grab old terminal i/o settings */
	  _new = _old; /* make new settings same as old settings */
	  _new.c_lflag &= ~ICANON; /* disable buffered i/o */
	  _new.c_lflag &= echo ? ECHO : ~ECHO; /* set echo mode */
	  tcsetattr(0, TCSANOW, &_new); /* use these new terminal i/o settings now */
	}
	/* Restore old terminal i/o settings */
	void resetTermios(void) 
	{
	  tcsetattr(0, TCSANOW, &_old);
	}
	/* Read 1 character - echo defines echo mode */
	char getch_(int echo) 
	{
	  char ch;
	  initTermios(echo);
	  ch = getchar();
	  resetTermios();
	  return ch;
	}
	/* Read 1 character without echo */
	char _getch(void) 
	{
	  return getch_(0);
	}
	int _kbhit(void)
	{
	  struct termios oldt, newt;
	  int ch;
	  int oldf;
	 
	  tcgetattr(STDIN_FILENO, &oldt);
	  newt = oldt;
	  newt.c_lflag &= ~(ICANON | ECHO);
	  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
	  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
	 
	  ch = getchar();
	 
	  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	  fcntl(STDIN_FILENO, F_SETFL, oldf);
	 
	  if(ch != EOF)
	  {
		ungetc(ch, stdin);
		return 1;
	  }
	  return 0;
	}
#endif


#include <math.h>
#include <vector>
#include <stdio.h>
#include <iostream>

// Include DUO and OpenCV header files
#include <DUOLib.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/contrib/contrib.hpp"

using namespace cv;
using namespace std;

// Some global variables
static DUOInstance _duo = NULL;
static PDUOFrame _pFrameData = NULL;

#if defined(WIN32)
	static HANDLE _evFrame = CreateEvent(NULL, FALSE, FALSE, NULL);
#elif defined(__linux__) || defined(__APPLE__)
	#include <pthread.h>
	#include <stdlib.h>
	#define WAIT_OBJECT_0	0
	struct event_flag
	{
		pthread_mutex_t mutex;
		pthread_cond_t  condition;
		unsigned int flag;
	};

	event_flag *CreateEvent(void *lpEventAttributes, bool bManualReset, bool bInitialState, char *name)
	{
		struct event_flag* ev = (struct event_flag*) malloc(sizeof(struct event_flag));
		pthread_mutex_init(&ev->mutex, NULL);
		pthread_cond_init(&ev->condition, NULL);
		ev->flag = 0;
		return ev;
	}

	void SetEvent(struct event_flag* ev)
	{
		pthread_mutex_lock(&ev->mutex);
		ev->flag = 1;
		pthread_cond_signal(&ev->condition);
		pthread_mutex_unlock(&ev->mutex);
	}

	int WaitForSingleObject(struct event_flag* ev, int timeout)
	{
		pthread_mutex_lock(&ev->mutex);
		while (!ev->flag)
			pthread_cond_wait(&ev->condition, &ev->mutex);
		ev->flag = 0;
		pthread_mutex_unlock(&ev->mutex);
		return WAIT_OBJECT_0;
	}

	static event_flag *_evFrame = CreateEvent(NULL, 0, 0, NULL);
#endif

//DepthImage class
class DepthImage
{
public:
	//functions
	DepthImage();
	~DepthImage();
	bool opening_duo();
	bool get_frames(IplImage *input_left, IplImage *input_right, Mat &output_left, Mat &output_right);
	bool create_all_trackbars(std::string windowname);
	bool sgbm_update(StereoSGBM &sgbm);

	//members
	int m_trackbar_samples = 100;
	//SGBM default values
	int m_SADWindowSize = 5 ;
	int m_numberOfDisparities = 192;
	int m_preFilterCap = 4 ;
	int m_minDisparity = -64 ;
	int m_uniquenessRatio = 1 ;
	int m_speckleWindowSize = 150 ;
	int m_speckleRange = 2 ;
	int m_disp12MaxDiff = 10 ;
	int m_fullDP = true;
	int m_P1 = 1500 ;
	int m_P2 = 2400 ;

};





//DUO related stuff

// One and only duo callback function
// It sets the current frame data and signals that the new frame data is ready
static void CALLBACK DUOCallback(const PDUOFrame pFrameData, void *pUserData)
{
	_pFrameData = pFrameData;
	SetEvent(_evFrame);
}

// Opens, sets current image format, fps and start capturing
static bool OpenDUOCamera(int width, int height, float fps)
{
	if(_duo != NULL)
	{
		// Stop capture
		StopDUO(_duo);
		// Close DUO
		CloseDUO(_duo);
		_duo = NULL;
	}

	// Find optimal binning parameters for given (width, height)
	// This maximizes sensor imaging area for given resolution
	int binning = DUO_BIN_NONE;
	if(width <= 752/2) 
		binning += DUO_BIN_HORIZONTAL2;
	else if(width <= 752/4) 
		binning += DUO_BIN_HORIZONTAL4;
	if(height <= 480/4) 
		binning += DUO_BIN_VERTICAL4;
	else if(height <= 480/2) 
		binning += DUO_BIN_VERTICAL2;

	// Check if we support given resolution (width, height, binning, fps)
	DUOResolutionInfo ri;
	if(!EnumerateResolutions(&ri, 1, width, height, binning, fps))
		return false;

	if(!OpenDUO(&_duo))
		return false;

	char tmp[260];
	// Get and print some DUO parameter values
	GetDUODeviceName(_duo,tmp);
	printf("DUO Device Name:      '%s'\n", tmp);
	GetDUOSerialNumber(_duo, tmp);
	printf("DUO Serial Number:    %s\n", tmp);
	GetDUOFirmwareVersion(_duo, tmp);
	printf("DUO Firmware Version: v%s\n", tmp);
	GetDUOFirmwareBuild(_duo, tmp);
	printf("DUO Firmware Build:   %s\n", tmp);

	// Set selected resolution
	SetDUOResolutionInfo(_duo, ri);

	// Start capture
	if(!StartDUO(_duo, DUOCallback, NULL))
		return false;

	return true;
}

// Waits until the new DUO frame is ready and returns it
static PDUOFrame GetDUOFrame()
{
	if(_duo == NULL) 
		return NULL;

	if(WaitForSingleObject(_evFrame, 1000) == WAIT_OBJECT_0)
		return _pFrameData;
	else
		return NULL;
}

// Stops capture and closes the camera
static void CloseDUOCamera()
{
	if(_duo == NULL)
		return;

	// Stop capture
	StopDUO(_duo);
	// Close DUO
	CloseDUO(_duo);
	_duo = NULL;
}

static void SetExposure(float value)
{
	if(_duo == NULL)
		return;
	SetDUOExposure(_duo, value);
}

static void SetGain(float value)
{
	if(_duo == NULL)
		return;
	SetDUOGain(_duo, value);
}

static void SetLed(float value)
{
	if(_duo == NULL)
		return;
	SetDUOLedPWM(_duo, value);
}

#endif
