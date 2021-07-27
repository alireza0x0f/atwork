#include<opencv2/core.hpp>
#include<opencv2/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include "opencv2/core.hpp"
#include <opencv2/core/utility.hpp>
#include "opencv2/core/opengl.hpp"
#include "opencv2/core/cuda.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fstream>
#include <string>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <algorithm>

using namespace std;
using namespace cv;
using namespace cv::cuda;
using namespace cuda;
//using namespace cv::Gpu;
using namespace std;

int henri23;
cv::Scalar b_l = Scalar(0, 170, 0);
cv::Scalar b_h = Scalar(50, 255, 50);
vector<Point>  shap1, shap2, shap3, shap4, shap5, shap6;
double intshap1, intshap2, intshap3, intshap4, intshap5, intshap6;
//DWORD dwRes;
//HANDLE hComm;
//DWORD dwRead;
//BOOL fWaitingOnRead = FALSE;
//OVERLAPPED osReader = { 0 };
char * lpBuf[10];
//BOOL henrihenri = TRUE;
int count = 0;
Point2f pt = Point2f(0, 0);/*
BOOL WriteABuffer(char * lpBuf, DWORD dwToWrite)   // A good working function
{
	OVERLAPPED osWrite = { 0 };
	DWORD dwWritten;
	DWORD dwRes;
	BOOL fRes;

	// Create this write operation's OVERLAPPED structure's hEvent.
	osWrite.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
	if (osWrite.hEvent == NULL)
		// error creating overlapped event handle
		return FALSE;

	// Issue write.
	if (!WriteFile(hComm, lpBuf, dwToWrite, &dwWritten, &osWrite)) {
		if (GetLastError() != ERROR_IO_PENDING) {
			// WriteFile failed, but isn't delayed. Report error and abort.
			fRes = FALSE;
		}
		else
			// Write is pending.
			dwRes = WaitForSingleObject(osWrite.hEvent, INFINITE);
		switch (dwRes)
		{
			// OVERLAPPED structure's event has been signaled. 
		case WAIT_OBJECT_0:
			if (!GetOverlappedResult(hComm, &osWrite, &dwWritten, FALSE))
				fRes = FALSE;
			else
				// Write operation completed successfully.
				fRes = TRUE;
			break;

		default:
			// An error has occurred in WaitForSingleObject.
			// This usually indicates a problem with the
			// OVERLAPPED structure's event handle.
			fRes = FALSE;
			break;
		}
	}

	else
		// WriteFile completed immediately.
		fRes = TRUE;

	//	CloseHandle(osWrite.hEvent);   // close the port 
	return fRes;
}


void Comport4()
{
	hComm = CreateFile("\\\\.\\COM4",   // com4 is ready to communicate ==> open the port
		GENERIC_READ | GENERIC_WRITE,
		0,
		0,
		OPEN_EXISTING,
		FILE_FLAG_OVERLAPPED,
		0);
	if (hComm == INVALID_HANDLE_VALUE)
	{
		//std::cout << "ER!" << endl;// error opening port; abort
		//std::cout << GetLastError();
		if (GetLastError() == 2)
		{
			std::cout << "Port 4 is not available!" << endl;

		}
		if (GetLastError() == 5)
		{
			std::cout << "Port 4 is already taken!" << endl;

		}

	}
	else
	{

		std::cout << "COM4 is available." << endl;  // And reday to use (TX/RX)

	}

}

void WriteComport4()
{
	if (_kbhit())   // otherwise it waits till the user prompts a charcter! You need this
	{          // to proceed it to the next step and read the received character in a real time basis!
		std::string henri7;
		henri7 = _getch();   // get the charcter from the user without hitting the Enter 
		std::cout << henri7;
		const char * c = henri7.c_str(); // string 2 char *
		WriteABuffer((char*)c, 1);

	}
	else
	{

	}

}


void ReadComport4()
{

	osReader.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
	// Create the overlapped event. Must be closed before exiting
	// to avoid a handle leak.

	//lpBuf[0] += 10;
	//dwRead == 1;
	ReadFile(hComm, lpBuf, 1, &dwRead, &osReader); // Read the Keyboard!
	if (dwRead == 1)         // In order to have an on the fly reading process
	{
		std::cout << (char*)lpBuf;
	}

}*/
//void onMouse(int event, int x, int y, int, void*);
#include<iostream>
cv::Mat imgOriginal;
// may have to modify this line if not using Windows
vector<vector<Point> > findColor(const cv::Mat & inputBGRimage, int rng, cv::Scalar b_l, cv::Scalar b_h)
{
	// Make sure that your input image uses the channel order B, G, R (check not implemented).
	cv::Mat input = inputBGRimage.clone();
	cv::Mat imageHSV;//(input.rows, input.cols, CV_8UC3);
	cv::Mat imgThreshold, imgThreshold0, imgThreshold1;//(input.rows, input.cols, CV_8UC1);
	Mat xyz;

	// convert input-image to HSV-image
	try {
		cv::cvtColor(input, imageHSV, cv::COLOR_BGR2HSV);
	}
	catch (Exception e) {}
	// In the HSV-color space the color 'red' is located around the H-value 0 and also around the
	// H-value 180. That is why you need to threshold your image twice and the combine the results.
	cv::inRange(imageHSV, b_l, b_h, imgThreshold0);

	if (rng > 0)
	{
		cv::inRange(imageHSV, b_l, b_h, imgThreshold1);
		cv::bitwise_or(imgThreshold0, imgThreshold1, imgThreshold);
	}
	else
	{
		imgThreshold = imgThreshold0;
	}
	Mat hierarchy;
	vector<vector<Point> > contours0;
	vector<vector<Point> > contours;
	GaussianBlur(imgThreshold1, imgThreshold1, Size(31, 31), 2.2, 2.2);

	findContours(imgThreshold1, contours0, hierarchy, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

	contours.resize(contours0.size());

	for (size_t k = 0; k < contours0.size(); k++) {


		approxPolyDP(Mat(contours0[k]), contours[k], 3, true);


	}



	return contours;
}


// input image
cv::Mat imgGrayscale;       // grayscale of input image
cv::Mat imgBlurred;         // intermediate blured image
cv::Mat imgCanny;
Mat mask;
cv::cuda::GpuMat lines;// Canny edge void condefects(vector<Vec4i> convexityDefectsSet, vector<Point> mycontour, Mat &original)
double condefects(vector<Vec4i> convexityDefectsSet, vector<Point> mycontour, Mat &original)
{
	vector<Point> allpoint;
	allpoint.resize(convexityDefectsSet.size());
	for (int cDefIt = 0; cDefIt < convexityDefectsSet.size(); cDefIt++) {
		int startIdx = convexityDefectsSet[cDefIt].val[0]; Point ptStart(mycontour[startIdx]);

		int endIdx = convexityDefectsSet[cDefIt].val[1]; Point ptEnd(mycontour[endIdx]);

		int farIdx = convexityDefectsSet[cDefIt].val[2]; Point ptFar(mycontour[farIdx]);

		double depth = static_cast<double>(convexityDefectsSet[cDefIt].val[3]) / 256;
		//cout << "depth" << depth << endl;
		//display start points
		circle(original, ptStart, 5, CV_RGB(255, 0, 0), 2, 8);
		//display all end points
		circle(original, ptEnd, 5, CV_RGB(255, 255, 0), 2, 8);
		//display all far points
		circle(original, ptFar, 5, CV_RGB(0, 0, 255), 2, 8);
		allpoint[cDefIt] = ptFar;

	}
	double res;
	double maxiu = 0;

	for (int i = 0; i < allpoint.size(); i++) {
		for (int j = 0; j < allpoint.size(); j++) {
			res = cv::norm(allpoint[i] - allpoint[j]);
			if (res > maxiu) {
				maxiu = res;
			}


		}

	}
	//	cout << std::to_string(maxiu)<<'\n';
	return maxiu;

}
//////////////////////////////////////////////////////////////////////////////////////////////
double erf(Mat b) {
	Mat hierarchy;
	double allpoint2 = 0;
	vector<vector<Point> > contours01;
	vector<vector<Point> > contours02;
	findContours(b, contours01, hierarchy, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
	contours02.resize(contours01.size());
	//	vector<Point> we;
	for (size_t m = 0; m < contours01.size(); m++) {
		vector<int>hull78;
		//	hull.resize(2);
		vector<Vec4i> defects50;

		approxPolyDP(Mat(contours01[m]), contours02[m], 5, true);
		convexHull(contours02[m], hull78, false);
		convexityDefects(Mat(contours01[m]), hull78, defects50);

		if (contourArea(contours01[m]) > 10 && contourArea(contours01[m]) < 500) {
			drawContours(b, contours01, m, CV_RGB(0, 255, 0), 2, 8, hierarchy);
			allpoint2 = condefects(defects50, contours01[m], b);
		}
	}
	return allpoint2;
}
//////////////////////////////////////////////////////////////////////////////////////////////
vector<Point> condefects1(vector<Vec4i> convexityDefectsSet, vector<Point> mycontour, Mat &original)
{
	vector<Point> allpoint;
	allpoint.resize(convexityDefectsSet.size());
	for (int cDefIt = 0; cDefIt < convexityDefectsSet.size(); cDefIt++) {

		int startIdx = convexityDefectsSet[cDefIt].val[0]; Point ptStart(mycontour[startIdx]);

		int endIdx = convexityDefectsSet[cDefIt].val[1]; Point ptEnd(mycontour[endIdx]);

		int farIdx = convexityDefectsSet[cDefIt].val[2]; Point ptFar(mycontour[farIdx]);
		//if (!shap1.empty()) {
		//		if (pointPolygonTest(shap1, ptFar, true) < -10){
		//	cout << "rt" << '\n';
		double depth = static_cast<double>(convexityDefectsSet[cDefIt].val[3]) / 256;
		//cout << "depth" << depth << endl;
		//display start points
		circle(original, ptStart, 5, CV_RGB(255, 0, 0), 2, 8);
		//display all end points
		circle(original, ptEnd, 5, CV_RGB(255, 255, 0), 2, 8);
		//display all far points
		circle(original, ptFar, 5, CV_RGB(0, 0, 255), 2, 8);
		allpoint[cDefIt] = ptFar;
		//}
		//	}

	}



	//	cout << std::to_string(maxiu)<<'\n';
	return allpoint;

}
//////////////////////////////////////////////////////////////////////////////////
int bb = 1;
///////////////////////////////////////////////////////////////////////////////////////////////////

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
	if (event == EVENT_LBUTTONDOWN)
	{
		Mat hsvv;
		cv::cvtColor(imgOriginal, hsvv, CV_BGR2HSV);
		Vec3b color = hsvv.at<Vec3b>(Point(x, y));
		b_l = Scalar(color[0] - 30, color[1] - 30, color[2] - 20);
		b_h = Scalar(color[0], color[1] + 30, color[2]);
		pt = Point(x, y);
		cout << ":" << x << "," << y << "," << std::to_string(color[0]) << "," << std::to_string(color[1]) << "," << std::to_string(color[2]);

	}
	else if (event == EVENT_RBUTTONDOWN)
	{
		cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
	}
	else if (event == EVENT_MBUTTONDOWN)
	{
		cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
	}
	else if (event == EVENT_MOUSEMOVE)
	{
		//	cout << "Mouse move over the window - position (" << x << ", " << y << ")" << endl;

	}
}
int main() {
	cv::VideoCapture cap(0);

	// declare a VideoCapture object and associate to webcam, 0 => use 1st webcam
	if (cap.isOpened() == false) {                                // check if VideoCapture object was associated to webcam successfully
		std::cout << "error: capWebcam not accessed successfully\n\n";      // if not, print error message to std out
		                                                       // may have to modify this line if not using Windows
		return(0);                                                          // and exit program
	}



	char charCheckForEscKey = 0;

	while (charCheckForEscKey != 27 && cap.isOpened()) {            // until the Esc key is pressed or webcam connection is lost
		bool blnFrameReadSuccessfully = cap.read(imgOriginal);            // get next frame

		if (!blnFrameReadSuccessfully || imgOriginal.empty()) {                 // if frame not read successfully
			std::cout << "error: frame not read from webcam\n";                 // print error message to std out
			break;                                                              // and jump out of while loop
		}


		// note: you can use CV_WINDOW_NORMAL which allows resizing the window
		cv::namedWindow("imgCanny", CV_WINDOW_NORMAL);

		// or CV_WINDOW_AUTOSIZE for a fixed size window matching the resolution of the image
		//	cv::namedWindow("imgHSV", CV_WINDOW_NORMAL);     														// CV_WINDOW_AUTOSIZE is the default


		//cv::Canny(imgOriginal, imgCanny, 100, 300);// show windows
		Mat gray, edge, draw, hierarchy;
		cv::cvtColor(imgOriginal, gray, COLOR_BGR2GRAY);
		int blockSize = 2;
		int apertureSize = 3;
		double k = 0.04;
		int thresh = 150;
		Mat dst = imgOriginal.clone();
		/*
		Canny(imgOriginal, dst, 100, 200, 3);
		vector<vector<Point> > contours0;
		vector<vector<Point> > contours;
		findContours(dst, contours0, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
		Mat drawing = Mat::zeros(imgOriginal.size(), CV_8UC3);
		contours.resize(contours0.size());
		for (size_t k = 0; k < contours0.size(); k++){
		approxPolyDP(Mat(contours0[k]), contours[k],5, true);


		}*/


		//cv::imshow("imgeage", drawing);
		cv::cvtColor(imgOriginal, gray, CV_BGR2GRAY);
		GaussianBlur(gray, gray, Size(1, 1), 1, 1);
		cv::Mat cnny;
		GaussianBlur(imgOriginal, draw, Size(1, 1), 2, 2);
		b_l = Scalar(0, 160, 0);
		b_h = Scalar(50, 255, 50);

		Mat brtt = imgOriginal.clone();
		imgOriginal.convertTo(brtt, -1, 0.9, 0);
		vector<vector<Point> > cont = findColor(brtt, 1, b_l, b_h);
		for (int i = 0; i< cont.size(); i++)
		{
			drawContours(imgOriginal, cont, i, Scalar(255, 255, 255), 1, 8, vector<Vec4i>(), 0, Point());
			if (contourArea(cont[i])>5000) {
				{
					if (pointPolygonTest(cont[i], pt, false)>0)
					{
						if (shap1.empty()) {
							vector<Vec4i> defects;
							vector<int>inthull;
							convexHull(cont[i], inthull, false);
							convexityDefects(Mat(cont[i]), inthull, defects);
							intshap1 = condefects(defects, cont[i], imgOriginal);
							cout << '\n' << std::to_string(intshap1);
							shap1 = cont[i];
						}
						else
						{
							if (shap2.empty()) {
								vector<Vec4i> defects;
								vector<int>inthull;
								convexHull(cont[i], inthull, false);
								convexityDefects(Mat(cont[i]), inthull, defects);
								intshap2 = condefects(defects, cont[i], imgOriginal);
								cout << '\n' << std::to_string(intshap2);
								shap2 = cont[i];
							}
							else
							{
								if (shap3.empty()) {
									vector<Vec4i> defects;
									vector<int>inthull;
									convexHull(cont[i], inthull, false);
									convexityDefects(Mat(cont[i]), inthull, defects);
									intshap3 = condefects(defects, cont[i], imgOriginal);
									shap3 = cont[i];
								}
								else
								{
									if (shap4.empty()) {
										vector<Vec4i> defects;
										vector<int>inthull;
										convexHull(cont[i], inthull, false);
										convexityDefects(Mat(cont[i]), inthull, defects);
										intshap4 = condefects(defects, cont[i], imgOriginal);
										shap4 = cont[i];
									}
									else
									{
										if (shap5.empty()) {
											vector<Vec4i> defects;
											vector<int>inthull;
											convexHull(cont[i], inthull, false);
											convexityDefects(Mat(cont[i]), inthull, defects);
											intshap5 = condefects(defects, cont[i], imgOriginal);
											shap5 = cont[i];
										}
										else
										{
											if (shap6.empty()) {
												vector<Vec4i> defects;
												vector<int>inthull;
												convexHull(cont[i], inthull, false);
												convexityDefects(Mat(cont[i]), inthull, defects);
												intshap6 = condefects(defects, cont[i], imgOriginal);
												shap6 = cont[i];
											}
											else
											{

											}
										}
									}
								}
							}

						}

						pt = Point(0, 0);
					}
					vector<Point> erq;
					//matchTemplate(Mat(cont[i]), shap1, erq, CV_TM_CCORR_NORMED);
					double er = matchShapes(cont[i], shap1, CV_CONTOURS_MATCH_I2, 0);
					if (er <0.2) {
						Moments mu;
						vector<int>inthull;
						vector<Point>hull;
						vector<Vec4i> defects;
						convexHull(cont[i], hull, false);
						convexHull(cont[i], inthull, false);
						convexityDefects(Mat(cont[i]), inthull, defects);
						drawContours(imgOriginal, cont, i, CV_RGB(0, 255, 0), 2, 8, hierarchy);
						double mm = condefects(defects, cont[i], imgOriginal);
						Point2f mc = Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);
						if (mm>intshap1 - 20 & mm<intshap1 + 20)
						{
							Moments mu;
							mu = moments(cont[i], false);
							Point2f mc = Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);
							putText(imgOriginal, "s40-40black", mc,
								FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200, 200, 250), 1, CV_AA);
						}

					}
					double er1 = matchShapes(cont[i], shap2, CV_CONTOURS_MATCH_I3, 0);
					if (er1 <0.2) {

						vector<int>inthull;
						vector<Point>hull;
						vector<Vec4i> defects;
						convexHull(cont[i], hull, false);
						convexHull(cont[i], inthull, false);
						convexityDefects(Mat(cont[i]), inthull, defects);
						drawContours(imgOriginal, cont, i, CV_RGB(0, 255, 0), 2, 8, hierarchy);
						//if want to show all contours use below one
						//drawContours(original,contours,-1, CV_RGB(0, 255, 0), 2, 8, hierarchy);

						//if want to show all hull, use below one
						//drawContours(original,hull,-1, CV_RGB(0, 255, 0), 2, 8, hierarchy);

						double mm = condefects(defects, cont[i], imgOriginal);
						if (mm>intshap2 - 5 & mm<intshap2 + 5)
						{
							Moments mu;
							mu = moments(cont[i], false);
							Point2f mc = Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);
							putText(imgOriginal, "s20-20black", mc,
								FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200, 200, 250), 1, CV_AA);
						}
					}
					double er2 = matchShapes(cont[i], shap3, CV_CONTOURS_MATCH_I3, 0);
					if (er2 <0.2) {
						vector<int>inthull;
						vector<Point>hull;
						vector<Vec4i> defects;
						convexHull(cont[i], hull, false);
						convexHull(cont[i], inthull, false);
						convexityDefects(Mat(cont[i]), inthull, defects);
						drawContours(imgOriginal, cont, i, CV_RGB(0, 255, 0), 2, 8, hierarchy);
						//if want to show all contours use below one
						//drawContours(original,contours,-1, CV_RGB(0, 255, 0), 2, 8, hierarchy);

						//if want to show all hull, use below one
						//drawContours(original,hull,-1, CV_RGB(0, 255, 0), 2, 8, hierarchy);
						double mm = condefects(defects, cont[i], imgOriginal);
						if (mm>intshap3 - 5 & mm<intshap3 + 5)
						{
							Moments mu;
							mu = moments(cont[i], false);
							Point2f mc = Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);
							putText(imgOriginal, "pich", mc,
								FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200, 200, 250), 1, CV_AA);
						}
					}
					double er3 = matchShapes(cont[i], shap4, CV_CONTOURS_MATCH_I3, 0);
					if (er3 <0.2) {

						vector<int>inthull;
						vector<Point>hull;
						vector<Vec4i> defects;
						convexHull(cont[i], hull, false);
						convexHull(cont[i], inthull, false);
						convexityDefects(Mat(cont[i]), inthull, defects);
						drawContours(imgOriginal, cont, i, CV_RGB(0, 255, 0), 2, 8, hierarchy);
						double mm = condefects(defects, cont[i], imgOriginal);
						if (mm>intshap4 - 10 & mm<intshap4 + 10)
						{
							Moments mu;
							mu = moments(cont[i], false);
							Point2f mc = Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);
							putText(imgOriginal, "mohre", mc,
								FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200, 200, 250), 1, CV_AA);
						}
					}
					double er4 = matchShapes(cont[i], shap5, CV_CONTOURS_MATCH_I3, 0);
					if (er4 <0.2) {

						vector<int>inthull;
						vector<Point>hull;
						vector<Vec4i> defects;
						convexHull(cont[i], hull, false);
						convexHull(cont[i], inthull, false);
						convexityDefects(Mat(cont[i]), inthull, defects);
						drawContours(imgOriginal, cont, i, CV_RGB(0, 255, 0), 2, 8, hierarchy);
						double mm = condefects(defects, cont[i], imgOriginal);
						if (mm>intshap5 - 5 & mm<intshap5 + 5)
						{
							Moments mu;
							mu = moments(cont[i], false);
							Point2f mc = Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);
							putText(imgOriginal, "motor", mc,
								FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200, 200, 250), 1, CV_AA);
						}
					}
					double er5 = matchShapes(cont[i], shap6, CV_CONTOURS_MATCH_I3, 0);
					if (er5 <0.2) {
						vector<int>inthull;
						vector<Point>hull;
						vector<Vec4i> defects;
						convexHull(cont[i], hull, false);
						convexHull(cont[i], inthull, false);
						convexityDefects(Mat(cont[i]), inthull, defects);
						drawContours(imgOriginal, cont, i, CV_RGB(0, 255, 0), 2, 8, hierarchy);
						double mm = condefects(defects, cont[i], imgOriginal);
						if (mm>intshap6 - 5 & mm<intshap6 + 5)
						{
							Moments mu;
							mu = moments(cont[i], false);
							Point2f mc = Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);
							putText(imgOriginal, "R20", mc,
								FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200, 200, 250), 1, CV_AA);
						}
					}



					Mat testMat = Mat(cont[0].size(), 2, CV_32FC1);
					memcpy(testMat.data, cont[0].data(), cont[0].size()*CV_32FC1);
					cnny = Mat(cont[i]);
					cv::imshow("imgCanny", testMat);
					Scalar color = Scalar(100, 20, 20);
					//drawContours(imgOriginal, cont, i, color, 2, 8);
				}
			}

		}
		/*
		Canny(draw, cnny, 100, 150);

		Mat hierarchy;
		vector<vector<Point> > contours;
		cv::findContours(cnny, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

		/// Draw contours
		Mat drawing = Mat::zeros(cnny.size(), CV_8UC3);
		for (int i = 0; i< contours.size(); i++)
		{
		Scalar color = Scalar(100, contourArea(contours[i]), 20);
		//	drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, Point());

		}
		*/
		//50,150,3
		vector<Vec3f> circles;
		/// Apply the Hough Transform to find the circles
		cv::HoughCircles(gray, circles, CV_HOUGH_GRADIENT, 1.5, gray.rows / 8, 180, 100, 0, 70);
		/// Draw the circles detected
		//cv::Mat mmb;
		/*
		vector<Vec4i> lines;
		Canny(imgOriginal, mmb, 50, 50, 3);
		//Canny(mmb, croo 100, 100);
		HoughLinesP(mmb, lines, 1, CV_PI / 180, 50, 50, 10);
		for (size_t i = 0; i < lines.size(); i++)
		{
		Vec4i l = lines[i];
		line(imgOriginal, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 3, CV_AA);
		}*/
		cv::Mat cropped;
		for (int i = 0; i < circles.size(); i++)
		{
			Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
			int radius = cvRound(circles[i][2]);
			cv::Rect(Point(center.x - radius, center.y - radius), Point(center.x + radius, center.y + radius));

			if (center.x - radius>5 && center.y - radius>5) {
				cv::Rect myROI(Point(center.x - radius, center.y - radius), Point(center.x + radius, center.y + radius));
				// Crop the full image to that image contained by the rectangle myROI
				// Note that this doesn't copy the 
				try {

					cv::Mat mmb;
					//	Canny(cropped, edge, 50, 200, 3
					//	vector<vector<Point> > cont = findColor(cropped, 1, b_l, b_h);
					//	drawContours(cropped, cont, -1, Scalar(0, 255, 0), 1, 8);
					cv::Mat crop;
					cv::Rect myROI(Point(center.x - 3 * radius / 2, center.y - 3 * radius / 2), Point(center.x + 3 * radius / 2, center.y + 3 * radius / 2));
					cv::Mat croppedRef(imgOriginal, myROI);
					croppedRef.copyTo(crop);
					vector<Vec4i> lines;
					Canny(crop, mmb, 50, 50, 3);

					HoughLinesP(mmb, lines, 1, CV_PI / 180, 50, 50, 10);
					if (lines.size() > 0) {
						if (lines.size() > 3) {
							//	putText(imgOriginal, "pich", center,
							//	FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200, 200, 250), 1, CV_AA);
						}
					}
					else {
						cv::Mat croppedRef(imgOriginal, myROI);
						croppedRef.copyTo(cropped);
						cv::Mat mmb;
						vector<Vec3f> circles01;
						/// Apply the Hough Transform to find the circles
						if (center.x < 450 & center.x>320 & center.y < 350 & center.y>210) {

							cropped.convertTo(cropped, -1, 0.2, 0);
						}
						cv::cvtColor(cropped, mmb, CV_BGR2GRAY);
						cv::HoughCircles(mmb, circles01, CV_HOUGH_GRADIENT, 15, mmb.rows / 8, 80, 160, 5, 70);
						for (int i = 0; i < circles01.size(); i++)
						{
							Point center1(cvRound(circles01[i][0]), cvRound(circles01[i][1]));
							int radius1 = cvRound(circles01[i][2]);
							circle(cropped, center1, radius, Scalar(255, 0, 255), 3, 8, 0);
							if (radius1 >(radius / 2) - 10 & radius1 < (radius / 2) + 10) {
								b_l = Scalar(0, 170, 0);
								b_h = Scalar(50, 255, 50);

								vector<vector<Point> > cont = findColor(cropped, 1, b_l, b_h);
								if (cont.size() >0)
								{
									putText(imgOriginal, "burbo", center,
										FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200, 200, 250), 1, CV_AA);

								}

							}
							else {
								b_l = Scalar(0, 170, 0);
								b_h = Scalar(60, 255, 50);
								vector<vector<Point> > cont = findColor(cropped, 1, b_l, b_h);
								if (cont.size() == 0)
									putText(imgOriginal, "ring", center,
										FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200, 200, 250), 1, CV_AA);


							}
						}

					}


				}
				catch (exception e) {}
				cout << std::to_string(radius) + '\n';
				// Copy the data into new matrix


			}


		}
		// circle center
		//circle(imgOriginal, center, 3, Scalar(0, 255, 0), -1, 8, 0);
		// circle outline


		//	circle(imgOriginal, Point(395, 298), 50, Scalar(0, 0, 255), 3, 8, 0);
		//	cv::namedWindow("imgOriginal", CV_WINDOW_AUTOSIZE);

		cv::imshow("imgOriginal", imgOriginal); setMouseCallback("imgOriginal", CallBackFunc, NULL);
		//GaussianBlur(imgOriginal, imgOriginal, Size(3, 3), 200, 200);
		dst.convertTo(dst, -1, 0.8, 0);
		Canny(dst, imgCanny, 150, 100);
		cv::Mat img, thresh1;
		//vector<vector<Point>> contours0;
		Vec4f lines;
		/*
		img = cv::imread("line.png");
		cv::cvtColor(dst, gray, cv::COLOR_BGR2GRAY);
		threshold(gray, thresh1, 127, 100, cv::THRESH_BINARY);
		findContours(thresh1, contours0, hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE);
		//cv::fitLine(Mat(contours[0]), lines, 2, 0, 0.01, 0.01);
		*/
		vector<vector<Point> > contours0;
		vector<vector<Point> > contours;
		findContours(imgCanny, contours0, hierarchy, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
		contours.resize(contours0.size());
		vector<Point> we;
		we.resize(contours0.size());
		for (size_t k = 0; k < contours0.size(); k++) {
			vector<int>hull;
			//	hull.resize(2);
			vector<Vec4i> defects;

			approxPolyDP(Mat(contours0[k]), contours[k], 5, true);
			convexHull(contours[k], hull, false);
			convexityDefects(Mat(contours0[k]), hull, defects);

			if (contourArea(contours0[k])>10 && contourArea(contours0[k])<500) {
				drawContours(dst, contours0, k, CV_RGB(0, 255, 0), 2, 8, hierarchy);
				vector <Point> allpoint = condefects1(defects, contours0[k], dst);

				vector <Point> tpoint;
				tpoint.resize(3);
				double res;
				double maxiu = 0;

				for (int i = 0; i < allpoint.size(); i++) {
					for (int j = 0; j < allpoint.size(); j++) {
						res = cv::norm(allpoint[i] - allpoint[j]);
						if (res > maxiu) {
							maxiu = res;
							tpoint[0] = allpoint[i];
							tpoint[1] = allpoint[j];
						}
					}

					if (maxiu>180 && maxiu<300) {
						bool check = true;
						/*
						for (int i = 0; i < allpoint.size(); i++) {
						double res = cv::norm(allpoint[i] - tpoint[0]);

						double res1 = cv::norm(-allpoint[i] + tpoint[1]);
						cout << std::to_string(res1-res) << '\n';
						if (res < 10 ) {
						Point we = Point((tpoint[1].x + tpoint[0].x) / 2, (tpoint[1].y + tpoint[0].y) / 2);
						putText(dst, "this is", we,
						FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200, 200, 250), 1, CV_AA);
						cout << "this is";
						//	check = false;

						}
						{

						}

						}

						if(check){

						we[k] = Point((tpoint[1].x + tpoint[0].x) / 2, (tpoint[1].y + tpoint[0].y) / 2);
						putText(dst, "ok",we[k] ,
						FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200, 200, 250), 1, CV_AA);
						cv::Rect myROI(Point(we[k].x-50, we[k].y - 50), Point(we[k].x + 50, we[k].y + 50));
						cv::Size s = dst.size();
						if (we[k].x - 50>0&& we[k].y - 50>0&& (we[k].x + 50)<s.width&& (we[k].y + 50)<s.height)
						{
						cv::Mat croppedRef(imgOriginal, myROI);
						Mat crop,can;
						croppedRef.copyTo(crop);


						Canny(crop, can, 100, 200, 3);
						vector<Vec2f> lines; // will hold the results of the detection
						HoughLines(can, lines, 2, CV_PI / 50, 80, 0, 0);
						for (size_t i = 0; i < lines.size(); i++)
						{
						float rho = lines[i][0], theta = lines[i][1];
						Point pt1, pt2;
						double a = cos(theta), b = sin(theta);
						double x0 = a*rho, y0 = b*rho;
						pt1.x = cvRound(x0 + 1000 * (-b));
						pt1.y = cvRound(y0 + 1000 * (a));
						pt2.x = cvRound(x0 - 1000 * (-b));
						pt2.y = cvRound(y0 - 1000 * (a));
						line(crop, pt1, pt2, Scalar(0, 0, 255), 3, CV_AA);
						}
						cv::imshow("imgOriginal02", crop);
						}
						}		*/

						we[k] = Point((tpoint[1].x + tpoint[0].x) / 2, (tpoint[1].y + tpoint[0].y) / 2);
						for (int bio = 0; bio < we.size(); bio++) {

							for (int bio1 = 0; bio1 < we.size(); bio1++) {
								if (bio1 != bio) {
									double aadr = cv::norm(we[bio] - we[bio1]);
									if (aadr > -10 & aadr < +10) {
										we[bio] = Point(0, 0);
									}
									else {

										cout << "";
									}
								}
							}

						}
						for (int k = 0; k < we.size(); k++) {
							if (we[k].x != 0 && we[k].y != 0)
							{
								//if (check) {

								//	we[k] = Point((tpoint[1].x + tpoint[0].x) / 2, (tpoint[1].y + tpoint[0].y) / 2);

								cv::Rect myROI(Point(we[k].x - 90, we[k].y - 90), Point(we[k].x + 90, we[k].y + 90));
								cv::Size s = dst.size();
								if (we[k].x - 90>0 && we[k].y - 90>0 && (we[k].x + 90)<s.width && (we[k].y + 90)<s.height)
								{
									cv::Mat croppedRef(imgOriginal, myROI);
									Mat crop, can;
									croppedRef.copyTo(crop);

									//	GaussianBlur(crop, crop, Size(5, 5), 2.2, 2.2);
									crop.convertTo(crop, -1, 0.5, 0);
									Canny(crop, can, 80, 120);

									vector<Vec2f> lines; // will hold the results of the detection


														 /////////////////////////////////////////////////////////////
									Mat hierarchy;
									double allpoint2 = 0;
									vector<vector<Point> > contours01;
									vector<vector<Point> > contours02;
									findContours(can, contours01, hierarchy, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
									contours02.resize(contours01.size());
									//	vector<Point> we;
									for (size_t m = 0; m < contours01.size(); m++) {
										vector<int>hull78;
										//	hull.resize(2);
										vector<Vec4i> defects50;

										approxPolyDP(Mat(contours01[m]), contours02[m], 7, true);
										convexHull(contours02[m], hull78, false);
										convexityDefects(Mat(contours01[m]), hull78, defects50);

										if (contourArea(contours01[m]) > 10 && contourArea(contours01[m]) < 500) {
											drawContours(crop, contours01, m, CV_RGB(0, 255, 0), 2, 8, hierarchy);
											allpoint2 = condefects(defects50, contours01[m], crop);
										}
									}

									////////////////////////////////////////////////////
									HoughLines(can, lines, 1.8, CV_PI / 50, 80, 0, 0);
									vector<Point> minpt;
									minpt.resize(lines.size());
									for (size_t i = 0; i < lines.size(); i++)
									{
										float rho = lines[i][0],
											theta = lines[i][1];
										//	float rho1 = lines[i+1][0];
										Point pt1, pt2;
										double a = cos(theta), b = sin(theta);
										double x0 = a*rho, y0 = b*rho;
										//cout << std::to_string(rho1-rho)<<'\n';
										pt1.x = cvRound(x0 + 100 * (-b));
										pt1.y = cvRound(y0 + 100 * (a));
										pt2.x = cvRound(x0 - 100 * (-b));
										pt2.y = cvRound(y0 - 100 * (a));

										minpt[i] = Point((pt1 + pt2) / 2);
										line(crop, pt1, pt2, Scalar(0, 0, 255), 3, CV_AA);
									}
									vector<double> yuo;
									yuo.resize(2 * lines.size());
									for (size_t i = 0; i < lines.size(); i++) {
										for (size_t j = 0; j < lines.size(); j++) {
											double aadr = cv::norm(minpt[i] - minpt[j]);
											yuo[i + j] = std::abs(aadr);
											//		cout << std::to_string(yuo[i + j]) << '\n';
										}
									}
									double maxyo = 0;
									for (size_t i = 0; i < 2 * lines.size(); i++) {
										if (yuo[i] > maxyo) {
											maxyo = yuo[i];

										}
									}

									if (maxyo > 30 && maxyo < 40 & allpoint2<2) {
										putText(dst, "qwe", we[k],
											FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200, 0, 250), 1, CV_AA);
									}
									if (maxyo > 65 && maxyo < 100 & lines.size()>3) {
										putText(dst, "pbd", we[k],
											FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200, 200, 0), 1, CV_AA);
									}
									cout << std::to_string(maxyo) << '\n';
									/*
									HoughLines(can, lines, 1, CV_PI / 50, 80, 0, 0);

									minpt.resize(lines.size());
									for (size_t i = 0; i < lines.size(); i++)
									{
									float rho = lines[i][0],
									theta = lines[i][1];
									//	float rho1 = lines[i+1][0];
									Point pt1, pt2;
									double a = cos(theta), b = sin(theta);
									double x0 = a*rho, y0 = b*rho;
									//cout << std::to_string(rho1-rho)<<'\n';
									pt1.x = cvRound(x0 + 100 * (-b));
									pt1.y = cvRound(y0 + 100 * (a));
									pt2.x = cvRound(x0 - 100 * (-b));
									pt2.y = cvRound(y0 - 100 * (a));

									minpt[i] = Point((pt1 + pt2) / 2);
									line(crop, pt1, pt2, Scalar(0, 0, 255), 3, CV_AA);
									}

									yuo.resize(2 * lines.size());
									for (size_t i = 0; i < lines.size(); i++) {
									for (size_t j = 0; j < lines.size(); j++) {
									double aadr = cv::norm(minpt[i] - minpt[j]);
									yuo[i + j] = std::abs(aadr);
									//		cout << std::to_string(yuo[i + j]) << '\n';
									}
									}

									for (size_t i = 0; i < 2 * lines.size(); i++) {
									if (yuo[i] > maxyo) {
									maxyo = yuo[i];

									}
									}
									if (maxyo > 5 && maxyo < 10 & allpoint2<2) {
									putText(dst, "miill", we[k],
									FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0, 0, 250), 1, CV_AA);
									}
									*/
									setMouseCallback("imgOriginal02", CallBackFunc, NULL);
									Mat fgi = crop.clone();
									//	cv::threshold(crop, fgi, 10, 100, cv::THRESH_MASK);
									cv::imshow("imgOriginal02", fgi);
								}
							}
						}

					}


				}

			}
		}


		//}


		cv::imshow("imgcanny", imgCanny);
		cv::imshow("imgOriginal01", dst);


		//	cv::imshow("imgHSV", HSV);
		//	setMouseCallback("imgOriginal", onMouse, &imgOriginal);
		charCheckForEscKey = cv::waitKey(2);        // delay (in ms) and get key press, if any
	}   // end while

	return(0);

}






