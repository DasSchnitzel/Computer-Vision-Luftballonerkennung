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

unsigned char findColor()
{
	cv::VideoCapture cap(0);

	if (!cap.isOpened())
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
	cv::namedWindow("cam", CV_WINDOW_AUTOSIZE);
	int counter = 0;
	long hueSum = 0;
	cv::Mat inputFrame;

	for (int i = 0; i < 50; i++)
	{
		int histo[256];
		long hueSumFrame = 0;
		int counterFrame = 0;
		bool bSuccess = cap.read(inputFrame);

		if (!bSuccess)
		{
			cout << "Cannot read a frame from video stream" << endl;
			break;
		}

		for (int i = 0; i < inputFrame.rows; i++) {
			for (int j = 0; j < inputFrame.cols; j++) {
				histo[inputFrame.at<cv::Vec3b>(i, j)[0]]++;
			}
		}
		for (int i = 50; i < 130; i++) {
			hueSumFrame += histo[i] * i;
			counterFrame += histo[i];
		}
		if (counterFrame != 0) {
			cout << "Average frame hue: " << hueSumFrame / counterFrame << endl;
			hueSum += hueSumFrame / counterFrame;
			counter++;
		}
		else {
			cout << "No blue//green pixel in Frame" << endl;
		}
		
		if (cv::waitKey(MY_WAIT_IN_MS) == 27)
		{
			cout << "ESC key is pressed by user" << endl;
			break;
		}
	}
	if (counter != 0) {
		cout << "Average Hue: " << hueSum / counter << endl;
		return (unsigned char) (hueSum / counter);
	}
	else return 0;
}

int MonoLoop(unsigned char color)
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
	cv::Mat src_gray, src;
	cv::Mat dst, cdst;

	src = inputFrame;
	cvtColor(inputFrame, src_gray, CV_BGR2GRAY);

	//lines1
	Canny(src, dst, 50, 200, 3);
	cvtColor(dst, cdst, CV_GRAY2BGR);
	vector<cv::Vec2f> lines;
	HoughLines(dst, lines, 1, CV_PI / 180, 100, 0, 0);
	for (size_t i = 0; i < lines.size(); i++)
	{
		float rho = lines[i][0], theta = lines[i][1];
		cv::Point pt1, pt2;
		double a = cos(theta), b = sin(theta);
		double x0 = a * rho, y0 = b * rho;
		pt1.x = cvRound(x0 + 1000 * (-b));
		pt1.y = cvRound(y0 + 1000 * (a));
		pt2.x = cvRound(x0 - 1000 * (-b));
		pt2.y = cvRound(y0 - 1000 * (a));
		line(src, pt1, pt2, cv::Scalar(0, 0, 255), 3, CV_AA);
	}

	/// Reduce the noise so we avoid false circle detection
	GaussianBlur(src_gray, src_gray, cv::Size(9, 9), 2, 2);

	vector<cv::Vec3f> circles;

	/// Apply the Hough Transform to find the circles
	HoughCircles(src_gray, circles, CV_HOUGH_GRADIENT, 1, 100, 10, 90, 0, 0);

	/// Draw the circles detected
	for (size_t i = 0; i < circles.size(); i++)
	{
		cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
		int radius = cvRound(circles[i][2]);
		// circle center
		circle(src, center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
		// circle outline
		circle(src, center, radius, cv::Scalar(0, 0, 255), 3, 8, 0);
	}

    outputFrame = src;
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
  unsigned char color = findColor();
  return MonoLoop(color);
}


