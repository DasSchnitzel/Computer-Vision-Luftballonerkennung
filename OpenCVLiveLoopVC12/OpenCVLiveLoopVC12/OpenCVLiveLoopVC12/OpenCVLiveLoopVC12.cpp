// OpenCVLiveLoopVC12.cpp : Definiert den Einstiegspunkt für die Konsolenanwendung.
//

#include "stdafx.h"
#include "MonoLoop.h"
#include "StereoLoop.h"
#include <iostream>
using namespace std;

const int MY_IMAGE_WIDTH  = 640;
const int MY_IMAGE_HEIGHT = 480;
const int MY_WAIT_IN_MS   = 20;


int MonoLoopOldStyle()
{
  IplImage* grabImage = 0;
  IplImage* resultImage = 0;
  int key;

  // create window for live video
  cvNamedWindow("Live", CV_WINDOW_AUTOSIZE);
  // create connection to camera
  CvCapture* capture = cvCaptureFromCAM(0);
  // init camera
  cvSetCaptureProperty( capture, CV_CAP_PROP_FRAME_WIDTH, MY_IMAGE_WIDTH ); 
  cvSetCaptureProperty( capture, CV_CAP_PROP_FRAME_HEIGHT, MY_IMAGE_HEIGHT );
  
  // check connection to camera by grabbing a frame
  if(!cvGrabFrame(capture))
  {      
    cvReleaseCapture(&capture);
    cvDestroyWindow("Live");
    printf("Could not grab a frame\n\7");
    return -1;
  }
  
  // retrieve the captured frame
  grabImage=cvRetrieveFrame(capture);           
  // init result image, e.g. with size and depth of grabImage
  resultImage = cvCreateImage(cvGetSize(grabImage), grabImage->depth, grabImage->nChannels);
 
  bool continueGrabbing = true;
  while (continueGrabbing)
  {
    if(!cvGrabFrame(capture))
    {              
      cvReleaseCapture(&capture);
      cvDestroyWindow("Live");
      cvReleaseImage(&grabImage);
      printf("Could not grab a frame\n\7");
      return -1;
    }
    else
    {
      grabImage = cvRetrieveFrame(capture);          
    
      /*******************************todo*****************************/
        cvCopy(grabImage, resultImage, NULL );
      /***************************end todo*****************************/
  
      cvShowImage("Live", resultImage);

      key = cvWaitKey(MY_WAIT_IN_MS);

      if (key == 27)
        continueGrabbing = false;       
    }
  }
 
  // release all
  cvReleaseCapture(&capture);
  cvDestroyWindow("Live");
  cvReleaseImage(&resultImage);

  return 0;
}

int MonoLoop()
{
  cv::VideoCapture cap(0);

  if(!cap.isOpened())
  {
    cout << "Cannot open the video cam" << endl;
    return -1;
  }
  
  // Set cameras to 15fps (if wanted!!!)
  cap.set(CV_CAP_PROP_FPS, 15);

  double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH);
  double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT);

  // Set image size
  cap.set(CV_CAP_PROP_FRAME_WIDTH, MY_IMAGE_WIDTH);
  cap.set(CV_CAP_PROP_FRAME_HEIGHT, MY_IMAGE_HEIGHT);

  // display the frame size that OpenCV has picked in order to check 
  cout << "cam Frame size: " << dWidth << " x " << dHeight << endl;
  cv::namedWindow("cam",CV_WINDOW_AUTOSIZE);

  cv::Mat inputFrame;
  cv::Mat outputFrame;

  while(1)
  {
   
    bool bSuccess = cap.read(inputFrame);

    if (!bSuccess)
    {
      cout << "Cannot read a frame from video stream" << endl;
      break;
    }

    /*******************************todo*****************************/
    outputFrame = inputFrame;
    /***************************end todo*****************************/
    
    imshow("cam", outputFrame);

    if(cv::waitKey(MY_WAIT_IN_MS) == 27)
    {
      cout << "ESC key is pressed by user" << endl;
      break;
    }
  }
  return 0;
}

int StereoLoop()
{
  cv::VideoCapture cap1(0);
  cv::VideoCapture cap2(1);

  if(!cap1.isOpened())
  {
    cout << "Cannot open the video cam [0]" << endl;
    return -1;
  }

  if(!cap2.isOpened())
  {
    cout << "Cannot open the video cam [1]" << endl;
    return -1;
  }
  
  // Set cameras to 15fps (if wanted!!!)
  cap1.set(CV_CAP_PROP_FPS, 15);
  cap2.set(CV_CAP_PROP_FPS, 15);

  double dWidth1 = cap1.get(CV_CAP_PROP_FRAME_WIDTH);
  double dHeight1 = cap1.get(CV_CAP_PROP_FRAME_HEIGHT);
  double dWidth2 = cap2.get(CV_CAP_PROP_FRAME_WIDTH);
  double dHeight2 = cap2.get(CV_CAP_PROP_FRAME_HEIGHT);

  // Set image size
  cap1.set(CV_CAP_PROP_FRAME_WIDTH, MY_IMAGE_WIDTH);
  cap1.set(CV_CAP_PROP_FRAME_HEIGHT, MY_IMAGE_HEIGHT);
  cap2.set(CV_CAP_PROP_FRAME_WIDTH, MY_IMAGE_WIDTH);
  cap2.set(CV_CAP_PROP_FRAME_HEIGHT, MY_IMAGE_HEIGHT);

  // display the frame size that OpenCV has picked in order to check 
  cout << "cam[0] Frame size: " << dWidth1 << " x " << dHeight1 << endl;
  cout << "cam[1] Frame size: " << dWidth2 << " x " << dHeight2 << endl;
  cv::namedWindow("cam[0]",CV_WINDOW_AUTOSIZE);
  cv::namedWindow("cam[1]",CV_WINDOW_AUTOSIZE);

  cv::Mat inputFrame1, inputFrame2;
  cv::Mat outputFrame1, outputFrame2;

  while(1)
  {
   
    bool bSuccess1 = cap1.read(inputFrame1);
    bool bSuccess2 = cap2.read(inputFrame2);

    if (!bSuccess1)
    {
      cout << "Cannot read a frame from video stream [0]" << endl;
      break;
    }

    if (!bSuccess2)
    {
      cout << "Cannot read a frame from video stream [1]" << endl;
      break;
    }
    

    /*******************************todo*****************************/
    outputFrame1 = inputFrame1;
    outputFrame2 = inputFrame2;
    /***************************end todo*****************************/


    imshow("cam[0]", outputFrame1);
    imshow("cam[1]", outputFrame2);

    if(cv::waitKey(MY_WAIT_IN_MS) == 27)
    {
      cout << "ESC key is pressed by user" << endl;
      break;
    }
  }
  return 0;
}



int _tmain(int argc, _TCHAR* argv[])
{
  CMonoLoop myLoop;
  //CStereoLoop myLoop;
  //myLoop.Run();

  return MonoLoop();
}

