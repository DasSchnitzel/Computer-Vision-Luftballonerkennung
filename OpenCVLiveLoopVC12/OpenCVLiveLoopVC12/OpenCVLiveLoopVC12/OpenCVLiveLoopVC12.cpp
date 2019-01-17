// OpenCVLiveLoopVC12.cpp : Definiert den Einstiegspunkt für die Konsolenanwendung.
//

#include "stdafx.h"
#include "MonoLoop.h"
#include "StereoLoop.h"
#include <math.h>
#include <iostream>
using namespace std;

const int MY_IMAGE_WIDTH  = 640;
const int MY_IMAGE_HEIGHT = 480;
const int MY_WAIT_IN_MS   = 20;
const double PI = 3.14159265359;


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

cv::Point intersectNoseMouth(cv::Vec2f mouth, cv::Vec2f nose)
{
	int x, y;
	x = ((mouth[0] / sin(mouth[1])) - (nose[0] / sin(nose[1]))) / ((-cos(nose[1]) / sin(nose[1])) + (cos(mouth[1]) / sin(mouth[1])));
	y = (-cos(nose[1]) / sin(nose[1])) * x + (nose[0] / sin(nose[1]));
	return cv::Point(x, y);
}

cv::Point intersectNoseEyes(cv::Vec3f eye1, cv::Vec3f eye2, cv::Vec2f nose)
{
	int x, y;
	x = (((nose[0] / sin(nose[1])) - eye1[1]) / (((eye2[1] - eye1[1]) / (eye2[0] - eye1[0])) + (cos(nose[1]) / sin(nose[1])))) + eye1[0];
	y = (((eye2[1] - eye1[1]) / (eye2[0] - eye1[0])) * (x - eye1[0])) + eye1[1];
	return cv::Point(x, y);
}

int distance(cv::Point a, cv::Point b)
{
	int d = sqrt(pow((a.x - b.x), 2) + pow((a.y - b.y), 2));
	return abs(d);
}

int findFace(cv::Mat circleMask, cv::Point center, int radius)
{
	cv::Mat gaussFrame;
	cv::Mat cannyFrame;
	vector<cv::Vec2f> lines;
	vector<cv::Vec3f> circles;
	int angle, eye1Index, eye2Index, eyeMouthDist, eyeNoseDist, noseMouthDist, lineIndex, circleDist, minCircleDist;
	cv::Point eyeNose, eyeMouth, noseMouth, circle1, circle2;

	GaussianBlur(circleMask, gaussFrame, cv::Size(9, 9), 2, 2); 
	HoughCircles(gaussFrame, circles, CV_HOUGH_GRADIENT, 1, 100, 10, 20, 0, radius/5);


	Canny(circleMask, cannyFrame, 50, 200, 3);
	HoughLines(cannyFrame, lines, 1, CV_PI / 180, 55, 0, 0);

	if ((lines.size() > 2) && (circles.size() > 2))
	{
		for (size_t i = 1; i < lines.size(); i++)
		{
			if (abs(lines[0][1] - lines[i][1]) > ((70.0 / 180.0) * PI) && abs(lines[0][1] - lines[i][1]) < ((110.0 / 180.0) * PI))
			{
				lineIndex = i;
				break;
			}
		}
		circle1 = cv::Point(circles[0][0], circles[0][1]);
		circle2 = cv::Point(circles[1][0], circles[1][1]);
		minCircleDist = distance(circle1, circle2);
		eye1Index = 0;
		eye2Index = 1;
		for (size_t i = 0; i < circles.size(); i++)
		{
			for (size_t j = 0; j < circles.size(); j++)
			{
				if (i != j)
				{
					circle1 = cv::Point(circles[i][0], circles[i][1]);
					circle2 = cv::Point(circles[j][0], circles[j][1]);
					minCircleDist = distance(circle1, circle2);
					circleDist = distance(circle1, circle2);
					if (circleDist < minCircleDist)
					{
						minCircleDist = circleDist;
						eye1Index = i;
						eye2Index = j;
					}
				}
			}
		}
		noseMouth = intersectNoseMouth(lines[0], lines[lineIndex]);
		eyeMouth = intersectNoseEyes(circles[eye1Index], circles[eye2Index], lines[0]);
		eyeNose = intersectNoseEyes(circles[eye1Index], circles[eye2Index], lines[lineIndex]);
		noseMouthDist = distance(noseMouth, center);
		eyeMouthDist = distance(eyeMouth, center);
		eyeNoseDist = distance(eyeNose, center);
		if (noseMouthDist < radius)
		{
			if (eyeMouthDist < radius && eyeNoseDist > radius)
			{
				return (int)(((eyeMouth.y < noseMouth.y ? (lines[0][1] + PI) : lines[0][1]) / PI) * 180.0);
			}
			else if (eyeMouthDist > radius && eyeNoseDist < radius)
			{
				return (int)(((eyeNose.y < noseMouth.y ? (lines[lineIndex][1] + PI) : lines[lineIndex][1]) / PI) * 180.0);
			}
		}
	}
	return -1;/*
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
		line(circleMask, pt1, pt2, 255, 3, CV_AA);
	}
	for (size_t i = 0; i < circles.size(); i++)
	{
		cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
		int radius = cvRound(circles[i][2]);
		// black circle 
		circle(circleMask, center, radius, 255, 5, 8, 0);
	}
	return circleMask;*/
}

cv::Mat drawCage(cv::Point center, int angle, int radius, cv::Mat inputCage, cv::Mat inputFrame)
{
	cv::Mat outputCage;
	cv::Vec3b green, cageColor;
	green[0] = 0;
	green[1] = 255;
	green[2] = 0;
	cv::Mat matrix = cv::getRotationMatrix2D(cv::Point(inputCage.cols / 2, inputCage.rows / 2), angle, 1);
	cv::warpAffine(inputCage, outputCage, matrix, cv::Size(inputCage.cols, inputCage.rows), cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 255, 0));
	cv::resize(outputCage, outputCage, cv::Size(radius * 2, radius * 2));
	for (int x = -radius; x < radius; x++)
	{
		for (int y = -radius; y < radius; y++)
		{
			cageColor = outputCage.at<cv::Vec3b>(cv::Point(x + radius, y + radius));
			if (cageColor != green)
			{
				inputFrame.at<cv::Vec3b>(center + cv::Point(x, y)) = cageColor;
			}
		}
	}
	return inputFrame;
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
  cv::Mat blackCircle;
  cv::Mat greyFrame;
  cv::Mat gaussFrame;
  cv::Mat circleMask;

  cv::Point center = (0,0);
  vector<cv::Vec3f> circles;
  //inputCage 
  cv::Mat inputCage;

  int faceAngle = -1;
  int angle = -1;
  int cageCounter = 0;
  int radius = 0;
  int entprellen = 0;

  inputCage = cv::imread("C:\\Users\\Philipp\\Desktop\\Coputervision\\Computer-Vision-Luftballonerkennung\\cage.bmp", CV_LOAD_IMAGE_COLOR);
  //endinputCage 

  while(1)
  {
   
    bool bSuccess = cap.read(inputFrame);

    if (!bSuccess)
    {
      cout << "Cannot read a frame from video stream" << endl;
      break;
    }

    /*******************************todo*****************************/

	if (!inputCage.data)
	{
		break;
	}

	//cv::Mat dst, cdst;


	/*/lines1
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
	}*/

	if (entprellen < 15 && circles.size() != 0)
	{
		entprellen++;
	}
	else
	{
		cvtColor(inputFrame, greyFrame, CV_BGR2GRAY);
		cvtColor(inputFrame, blackCircle, CV_BGR2GRAY);
		cvtColor(inputFrame, circleMask, CV_BGR2GRAY);

		/// Reduce the noise so we avoid false circle detection 
		GaussianBlur(greyFrame, gaussFrame, cv::Size(9, 9), 2, 2);

		/// Apply the Hough Transform to find the circles 
		HoughCircles(gaussFrame, circles, CV_HOUGH_GRADIENT, 1, 100, 10, 90, 0, 0);
		entprellen = 0;
	}
	/// Draw the circles detected 
	for (size_t i = 0; i < circles.size(); i++)
	{
		center = cv::Point(cvRound(circles[i][0]), cvRound(circles[i][1]));
		cvtColor(inputFrame, blackCircle, CV_BGR2GRAY); 
		cvtColor(inputFrame, circleMask, CV_BGR2GRAY); 
		radius = cvRound(circles[i][2]);
		// black circle 
		circle(blackCircle, center, radius, 0, -1, 8, 0); 
		//circle Mask
		for (int y = 0; y < blackCircle.rows; y++)
		{
			for (int x = 0; x < blackCircle.cols; x++)
			{
				if (blackCircle.at<uchar>(cv::Point(x, y)) != 0)
				{
					circleMask.at<uchar>(cv::Point(x, y)) = 0;
				}
			}
		}
		angle = findFace(circleMask, center, radius);
		if (angle == -1)
		{
			cageCounter++;
		}
		else
		{
			faceAngle = angle;
		}
		if (cageCounter == 15)
		{
			faceAngle = angle;
			cageCounter = 0;
		}
		if (faceAngle != -1)
		{
			inputFrame = drawCage(center, -faceAngle, radius, inputCage, inputFrame);
		}
	}
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
 // unsigned char color = findColor();
  return MonoLoop(130);
}


