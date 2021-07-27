#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <sensor_msgs/LaserScan.h>
#include <math.h>
#include <cfloat>



#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv/cv.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
/////
#include <opencv2/core/utility.hpp>
#include "opencv2/core/opengl.hpp"
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
////

using namespace cv;
using namespace std;



/// anagle - distance
int obs_points [30][30][2] = {{{209,83},{207,81},{206,80},{204,79},{203,78},{201,78},{199,77},{197,76},{195,76},{193,75},{191,75},{189,74},{187,74},{185,73},{183,72},{180,72},{179,72},{177,73},{175,74},{172,74},{171,74},{168,75},{167,75},{165,76},{163,77},{161,78},{160,78},{158,80},{157,80},{155,81}},

{{210,80},{207,78},{206,77},{204,76},{203,75},{201,74},{199,74},{197,73},{195,72},{193,71},{191,71},{189,70},{187,69},{185,69},{183,69},{181,70},{178,70},{177,70},{175,70},{173,71},{171,71},{168,72},{166,72},{165,73},{163,74},{161,75},{159,75},{157,76},{156,77},{155,78}},

{{210,77},{208,76},{206,75},{205,74},{203,73},{201,72},{198,71},{197,70},{195,69},{193,69},{191,68},{189,68},{187,67},{185,67},{183,67},{180,67},{178,68},{177,68},{174,68},{173,68},{170,69},{169,70},{166,70},{164,71},{162,72},{161,72},{159,73},{156,74},{156,75},{154,76}},

{{210,75},{209,73},{207,73},{205,71},{204,70},{201,69},{199,68},{197,67},{195,67},{193,66},{191,66},{189,66},{187,65},{185,65},{183,64},{180,64},{178,64},{176,64},{174,65},{172,65},{170,66},{168,67},{166,67},{164,67},{162,68},{160,69},{158,70},{156,71},{155,72},{154,73}},

{{211,71},{209,70},{207,70},{205,69},{204,68},{202,67},{200,65},{198,65},{195,64},{193,64},{191,63},{189,63},{187,63},{185,62},{183,62},{180,62},{178,62},{176,62},{174,62},{172,63},{170,63},{168,64},{166,64},{163,65},{162,66},{160,66},{158,68},{156,68},{155,69},{153,69}},

{{212,70},{209,68},{208,67},{206,66},{204,65},{202,64},{200,64},{198,63},{195,62},{193,62},{191,62},{189,61},{187,60},{185,60},{183,60},{180,60},{178,60},{176,60},{174,60},{172,61},{170,62},{168,62},{166,62},{163,63},{162,64},{160,65},{157,65},{156,67},{154,67},{153,68}},

{{212,67},{210,66},{208,64},{206,64},{204,63},{202,62},{200,61},{198,60},{196,60},{193,59},{191,59},{189,59},{187,59},{185,58},{183,58},{180,58},{178,59},{176,59},{174,58},{172,59},{169,59},{168,60},{165,60},{163,60},{162,61},{159,62},{157,63},{156,64},{154,64},{153,65}},

{{212,64},{211,64},{208,62},{206,61},{204,60},{202,60},{200,59},{198,59},{196,58},{194,57},{191,57},{189,57},{187,57},{185,56},{182,56},{180,56},{178,56},{176,56},{174,57},{172,57},{169,57},{168,58},{164,58},{163,58},{161,59},{159,60},{157,60},{156,61},{154,62},{153,63}},

{{212,63},{211,61},{209,61},{206,59},{205,59},{203,58},{201,58},{199,57},{196,56},{194,56},{191,56},{189,55},{187,55},{185,55},{182,55},{180,54},{178,54},{176,54},{174,55},{172,55},{169,56},{167,56},{164,56},{163,57},{161,57},{159,58},{157,59},{155,60},{153,60},{152,61}},

{{213,61},{211,60},{209,59},{207,57},{205,57},{203,56},{201,56},{199,55},{197,55},{194,54},{192,54},{189,53},{187,53},{185,52},{182,53},{180,52},{178,52},{176,52},{173,53},{171,53},{169,54},{166,54},{164,54},{162,55},{160,56},{158,57},{156,58},{154,58},{153,59},{152,60}},

{{214,59},{212,57},{210,57},{207,56},{206,56},{204,54},{202,54},{199,53},{197,53},{194,53},{192,52},{189,51},{187,51},{185,51},{182,50},{180,50},{178,50},{176,50},{173,51},{171,51},{169,52},{166,52},{164,52},{162,53},{159,54},{158,55},{156,55},{154,56},{152,57},{151,58}},

{{214,57},{212,54},{211,55},{208,54},{206,54},{204,53},{202,53},{199,52},{197,51},{194,51},{192,50},{190,50},{187,49},{185,49},{182,49},{180,49},{178,49},{176,49},{173,49},{171,50},{168,50},{166,50},{164,51},{162,52},{159,52},{157,53},{156,54},{154,54},{152,55},{150,56}},

{{214,55},{213,54},{211,54},{209,53},{207,52},{204,51},{202,51},{199,50},{197,49},{194,48},{192,48},{190,48},{187,48},{185,47},{182,47},{180,47},{178,47},{176,48},{173,48},{171,48},{168,49},{166,49},{163,50},{161,50},{159,50},{157,51},{155,52},{153,53},{152,54},{149,55}},

{{215,56},{213,52},{212,53},{209,52},{207,50},{205,50},{203,49},{200,48},{197,47},{194,47},{192,46},{190,46},{187,46},{185,46},{182,46},{180,46},{178,46},{176,46},{173,46},{171,46},{167,47},{165,47},{163,48},{160,49},{159,49},{156,50},{154,50},{152,51},{151,53},{149,54}},

{{216,53},{213,52},{212,51},{210,50},{207,49},{205,48},{203,48},{200,47},{198,46},{195,45},{192,45},{190,45},{187,45},{185,45},{182,44},{180,44},{178,44},{176,45},{173,44},{171,45},{167,45},{165,46},{163,46},{161,47},{158,48},{156,48},{154,49},{152,50},{150,51},{149,52}},

{{217,52},{214,51},{212,50},{210,49},{208,47},{206,47},{204,46},{201,45},{198,44},{195,44},{193,43},{190,43},{187,43},{185,43},{182,43},{180,42},{178,42},{176,43},{173,43},{170,43},{167,44},{165,45},{162,45},{160,45},{157,46},{156,47},{153,48},{151,49},{149,50},{148,51}},

{{217,51},{215,49},{213,48},{211,48},{209,46},{206,45},{204,44},{201,44},{199,43},{196,43},{193,42},{190,41},{187,48},{185,48},{182,41},{180,41},{178,41},{176,41},{173,41},{170,42},{167,42},{164,43},{162,43},{159,44},{157,45},{155,45},{153,46},{151,47},{149,48},{148,49}},

{{218,48},{216,47},{214,46},{211,46},{209,44},{207,43},{204,43},{201,41},{198,41},{196,41},{193,40},{190,40},{187,39},{185,39},{182,39},{180,40},{178,40},{175,39},{173,39},{169,40},{166,40},{164,41},{162,42},{159,42},{157,43},{155,43},{152,44},{150,45},{148,47},{145,48}},

{{219,47},{217,46},{215,45},{212,44},{210,43},{207,41},{205,41},{202,40},{200,39},{196,39},{194,39},{191,39},{187,38},{185,38},{182,38},{180,38},{178,38},{175,38},{173,38},{169,39},{166,39},{164,39},{162,40},{159,40},{156,41},{153,42},{152,43},{149,44},{148,45},{145,46}},

{{220,45},{217,45},{216,43},{213,42},{210,41},{208,40},{205,40},{203,39},{200,38},{197,38},{194,37},{191,37},{187,37},{185,37},{182,37},{180,37},{178,37},{175,37},{174,37},{169,37},{166,38},{164,38},{162,39},{158,39},{156,40},{152,40},{151,41},{149,42},{147,44},{145,45}},

{{220,44},{218,43},{216,42},{214,41},{211,40},{210,39},{206,39},{203,37},{200,37},{198,37},{194,36},{191,35},{188,36},{185,36},{182,36},{180,36},{178,36},{175,36},{174,36},{169,36},{166,37},{163,38},{162,38},{158,38},{155,39},{152,39},{151,40},{148,41},{147,43},{144,44}},

{{221,43},{219,42},{217,41},{215,39},{212,39},{210,38},{207,37},{204,37},{201,36},{199,35},{194,35},{191,34},{188,34},{185,34},{182,34},{180,34},{178,34},{175,34},{174,34},{169,35},{166,35},{163,35},{159,36},{157,37},{154,38},{152,38},{151,39},{148,40},{146,41},{144,42}},

{{222,42},{220,41},{218,39},{216,39},{213,38},{211,36},{208,36},{205,35},{202,34},{199,34},{195,33},{191,32},{188,32},{185,32},{182,32},{180,32},{178,32},{175,32},{174,32},{169,32},{166,33},{162,34},{159,35},{157,36},{152,37},{151,37},{148,38},{147,39},{144,44},{143,41}},

{{222,39},{221,39},{219,39},{217,39},{214,36},{212,35},{209,35},{205,34},{203,33},{199,33},{195,32},{192,31},{188,31},{185,31},{182,31},{180,31},{178,31},{175,31},{174,31},{168,31},{166,32},{162,33},{159,34},{157,34},{152,36},{151,36},{148,37},{147,38},{144,39},{140,40}},

{{223,38},{222,38},{220,37},{218,36},{215,35},{213,34},{209,33},{206,33},{204,32},{200,31},{195,31},{192,30},{188,29},{185,29},{182,29},{180,29},{178,29},{175,29},{174,29},{167,30},{164,31},{162,31},{158,32},{155,33},{151,34},{150,35},{147,36},{146,37},{143,38},{140,38}},

{{224,37},{222,37},{220,36},{218,37},{216,34},{214,33},{210,32},{207,32},{204,31},{200,29},{196,28},{192,28},{188,28},{185,28},{182,28},{180,28},{178,28},{175,28},{174,28},{167,28},{164,29},{162,30},{157,31},{154,32},{151,32},{150,34},{147,35},{144,36},{143,37},{140,37}},

{{225,36},{223,36},{221,35},{219,34},{217,33},{214,32},{210,31},{208,30},{204,29},{200,28},{196,27},{192,27},{188,27},{185,27},{182,27},{180,27},{178,27},{175,27},{174,27},{167,27},{164,28},{161,29},{156,29},{153,31},{151,31},{150,32},{146,34},{144,35},{142,36},{140,36}},

{{225,34},{224,35},{222,34},{220,33},{218,32},{215,31},{211,30},{208,29},{204,28},{202,28},{196,26},{193,26},{188,26},{185,26},{182,26},{180,26},{178,26},{175,26},{174,26},{167,26},{164,27},{161,28},{155,28},{153,29},{150,30},{149,31},{146,32},{144,33},{142,34},{139,35}},

{{226,33},{225,33},{223,32},{221,32},{219,38},{217,29},{212,28},{209,28},{204,27},{203,27},{197,25},{191,25},{188,25},{185,25},{182,25},{180,25},{178,25},{175,25},{174,25},{167,26},{164,26},{161,27},{155,27},{150,28},{149,29},{48,29},{144,31},{143,32},{140,33},{138,34}},

{{227,32},{224,31},{223,30},{219,29},{217,28},{214,27},{210,27},{205,26},{204,26},{202,25},{190,24},{189,24},{185,24},{181,24},{180,23},{178,23},{175,24},{174,24},{169,25},{167,25},{164,25},{161,26},{155,26},{151,27},{149,28},{146,29},{144,30},{143,31},{142,32},{137,34}}

};
double ranges [360] ;
double intensities [360] ;
Mat W20x20;
int CIRCLE_MODE=0;
int BLACK_SHAPE_MODE=1;
int WHITE_SHAPE_MODE=2;
int GET_SHAPE_MODE=3;

int minH_y =15;
int maxH_y =45;
int minS_y =131;
int maxS_y =255;
int minV_y =102;
int maxV_y =255;

int minH_b =86;
int maxH_b =134;
int minS_b =175;
int maxS_b =255;
int minV_b =51;
int maxV_b =255;
int cx=0;
int cy=0;
bool finder=false;
///Scale = 0 -----> 1 ----->2
float Scale=1;
double search_o=0;

ros::NodeHandle *n_global;

vector<Point>  O40x40,O20x20,mohre_G,motor,mohre_S,pich,tube;

bool meterEnable = false;
vector<vector<Point> > color_dec(Mat frame,int Mode);
double Hull_dec(vector<Vec4i> convexityDefectsSet, vector<Point> mycontour);
VideoCapture cap;
vector<Point> search_x_y;
vector<Point> obj_selector;
vector<float> search_Angle;
ros::Publisher scan_pub;
Mat result;
//**************************************************************************************CHECK_POINT
bool find_if_close(vector<Point > cnt1, vector<Point>  cnt2) {
	for (int pixel_1 = 0; pixel_1 < cnt1.size(); pixel_1++) {
		for (int pixel_2 = 0; pixel_2 < cnt2.size(); pixel_2++) {
			int  dist = cv::norm(cnt1[pixel_1] - cnt2[pixel_2]);
			if (cv::abs(dist) < 20) {

				return true;

			}
			else
			{
				if (pixel_1 == cnt1.size() && pixel_2 == cnt2.size()) return false;
			}

		}

	}
}
///*************************************************************************************SHAFT_OBJ
Mat finder_white_shaft(Mat frame) {
	Mat gray;
	Mat draw;
        Mat dst;
	Mat hierarchy;
	vector<vector<Point> > cont;
        for(int ww=0;ww<19;ww++){
            frame.convertTo(dst,CV_8U,0.3+ww*0.1,2);
	cv::cvtColor(dst, gray, CV_BGR2GRAY);
	GaussianBlur(gray, gray, Size(3, 3), 3, 3);
        int qq=0;
        if(ww%2==0){
             qq=0;
            
        }
	cv::threshold(gray, draw, 160, 250, qq);
//for(size_t b=0;b<1000000;b++);
	//vector<vector<Point> > contours;
	findContours(draw, cont, hierarchy, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));
	vector<double> status(cont.size(), 0);

	for (int i = 0; i < cont.size(); i++) {

            RotatedRect r = minAreaRect( Mat(cont[i]));
             if(r.size.height>80&&r.size.height<250&&r.size.width>00&&r.size.width<8){
                    putText(frame, "mile", r.center,
                    FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200, 200, 250), 1, CV_AA);

                     if (r.size.width > r.size.height)
              {
                r.angle += 90;
              }


              r.angle = (((r.angle+90)*3.141592)/180) + 1.4229;
                     search_x_y.push_back(Point(r.center));
                  search_Angle.push_back(r.angle);
break;
            }
           /// &&frame.rows-r.center.y>50&&frame.cols-r.center.x>50
           ///////////   ROS_INFO("%f - %f - %f",r.size.height,r.size.width, r.angle);
              if(r.size.width>80&&r.size.width<250&&r.size.height>0&&r.size.height<8){
                    putText(frame, "mile", r.center,
                    FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200, 200, 250), 1, CV_AA);

                     if (r.size.width > r.size.height)
              {
                r.angle += 90;
              }


              r.angle = (((r.angle+90)*3.141592)/180) + 1.4229;
                     search_x_y.push_back(Point(r.center));
                  search_Angle.push_back(r.angle);
break;
            }
        }
        cv::imshow("imgcanny", draw);
        }
        return frame;
}



///====================================================================================================

//**************************************************************************************WHITE_SHAPE_MODE
Mat finder_white_show(Mat frame) {
	Mat gray;
	Mat draw;
        Mat dst;
	Mat hierarchy;
        vector<float> obj_selector_angle;
        vector<vector<Point> > result;
        int index=0;
         for(int qw=0;qw<50;qw++){
	vector<vector<Point> > cont;
        //vector<vector<Point> > city;
        frame.convertTo(dst,CV_8U,0.5+qw*0.02,2);
	cv::cvtColor(dst, gray, CV_BGR2GRAY);
	GaussianBlur(gray, gray, Size(3, 3), 3, 3);
	cv::threshold(gray, draw, 160, 255, 0);

	//vector<vector<Point> > contours;
	findContours(draw, cont, hierarchy, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));
	vector<double> status(cont.size(), 0);

	for (int i = 0; i < cont.size(); i++) {
		int x = i;
                //////******************************************************************************************LARGE____
                if(search_o==10){
                 RotatedRect r1 = minAreaRect( Mat(cont[i]));
                 if(r1.size.height>120&&r1.size.height<200&&r1.size.width>20&&r1.size.width<50){
                  //  putText(frame, "40x400", r1.center,
                   // FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200, 200, 250), 1, CV_AA);
                    finder=true;
                     if (r1.size.width > r1.size.height)
              {
                r1.angle += 90;
              }


              r1.angle = (((r1.angle+90)*3.141592)/180) + 1.4229;
                    obj_selector.push_back(r1.center);
                    obj_selector_angle.push_back(float(r1.angle));

            }
           /// &&frame.rows-r.center.y>50&&frame.cols-r.center.x>50
           ///   ROS_INFO("%f - %f - %f",r0.size.height,r0.size.width, r0.angle);
              if(r1.size.width>100&&r1.size.width<180&&r1.size.height>20&&r1.size.height<50){
                   // putText(frame, "40*40", r1.center,
                   // FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200, 200, 250), 1, CV_AA);
                     finder=true;
                      if (r1.size.width > r1.size.height)
              {
                r1.angle += 90;
              }


              r1.angle = (((r1.angle+90)*3.141592)/180) + 1.4229;
        obj_selector.push_back(r1.center);
        obj_selector_angle.push_back(float(r1.angle));

            }}
            //************************************************************************************************SMAILL
             if(search_o==9){
                RotatedRect r0 = minAreaRect( Mat(cont[i]));
                 if(r0.size.height>100&&r0.size.height<200&&r0.size.width>15&&r0.size.width<25){
                   /// putText(frame, "20x220", r0.center,
                   // FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200, 200, 250), 1, CV_AA);
                      if (r0.size.width > r0.size.height)
              {
                r0.angle += 90;
              }


              r0.angle = (((r0.angle+90)*3.141592)/180) + 1.4229;
                    obj_selector.push_back(r0.center);
                    obj_selector_angle.push_back(float(r0.angle));

            }
           /// &&frame.rows-r.center.y>50&&frame.cols-r.center.x>50
        //      ROS_INFO("%f - %f - %f",r0.size.height,r0.size.width, r0.angle);
              if(r0.size.width>100&&r0.size.width<180&&r0.size.height>15&&r0.size.height<25){
                   // putText(frame, "20*20", r0.center,
                   // FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200, 200, 250), 1, CV_AA);
                      if (r0.size.width > r0.size.height)
              {
                r0.angle += 90;
              }


              r0.angle = (((r0.angle+90)*3.141592)/180) + 1.4229;
                   obj_selector.push_back(r0.center);
                   obj_selector_angle.push_back(float(r0.angle));

            }}
            ///////////////////////*******************************************************************
            
            //******************************************************************************

        }

	cv::imshow("imgcanny", draw);

         }

         if(!obj_selector.empty())
         {

              vector <vector <Point> > costList;
              vector <vector <float> > costListAngle;
              for(int i =0 ; i<obj_selector.size(); i++)
              {



                  vector<Point> cost;
                  vector<float> costAngle;
                  for(int j =0 ; j<i ; j++)
                  {
                      int x2 = obj_selector[i].x - obj_selector[j].x;
                      int y2 = obj_selector[i].y - obj_selector[j].y;
                      x2 = x2*x2;
                      y2 = y2*y2;
                      int delta = sqrt(x2+y2);
                      if (delta < 30)
                      {
                          cost.push_back(obj_selector[j]);
                          costAngle.push_back(obj_selector_angle[j]);
                      }
                  }
                  costList.push_back(cost);
                  costListAngle.push_back(costAngle);

              }
              // get maximum index
              int maxIndex = 0;
              int maxSize = 0;
              for(int i =0 ; i<costList.size(); i++)
              {
                    if( maxSize < costList[i].size() )
                    {
                        maxSize = costList[i].size();
                        maxIndex = i;
                    }
              }
              int totalX=0;
              int totalY=0;
              float totalAngle = 0;
              for(int i =0 ; i<costList[maxIndex].size(); i++)
              {
                  totalX += costList[maxIndex][i].x;
                  totalY += costList[maxIndex][i].y;
              }
              if(totalX != 0 && totalY != 0)
              {
                  totalX = totalX/costList[maxIndex].size();
                  totalY = totalY/costList[maxIndex].size();

                  totalAngle = costListAngle[maxIndex][0];
              }


              putText(frame, "40*40", Point(totalX,totalY),
                  FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200, 200, 250), 1, CV_AA);

                  search_x_y.push_back(Point(totalX,totalY));
                  search_Angle.push_back(totalAngle);
             //if(obj_selector.size()>5){
                  /*ROS_INFO("%s","erfan1");
                vector<vector<Point> > city(obj_selector.size(), vector<Point>());
                 ROS_INFO("%s","erfan02");
                for(int obj=0;obj<obj_selector.size();obj++){
                 for(int obj1=0;obj1<obj_selector.size();obj1++){

                 if(abs(norm(obj_selector[obj]-obj_selector[obj1]))<15&&obj!=obj1){
                     city[obj].push_back(obj_selector[obj]);
                     city[obj].push_back(obj_selector[obj1]);
                 }}}
                 result=city;*/
            //}

         }
            /*
                 int maximum=0;

for(int maxi=0;maxi<result.size();maxi++){
if(result[maxi].size()>maximum){
    maximum=result[maxi].size();
    index=maxi;
}

}
if(!result.empty()){

ROS_INFO("%f - %f - %f",result[index][0].x,result[index][0].y, 0);

}*/

	//cv::imshow("imgOriginal01", frame);
	obj_selector.clear();
	return frame;
}
///************************************************************



//**************************************************************************************THEME_MATCH
void MatchingMethod( Mat img,Mat templ,int match_method )
{
  /// Source image to display
  Mat img_display;
  img.copyTo( img_display );

  /// Create the result matrix
  int result_cols =  img.cols - templ.cols + 1;
  int result_rows = img.rows - templ.rows + 1;

  result.create( result_rows, result_cols, CV_32FC1 );

  /// Do the Matching and Normalize
  matchTemplate( img, templ, result, match_method );
  normalize( result, result, 0, 1, NORM_MINMAX, -1, Mat() );

  /// Localizing the best match with minMaxLoc
  double minVal; double maxVal; Point minLoc; Point maxLoc;
  Point matchLoc;

  minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );

  /// For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
  if( match_method  == CV_TM_SQDIFF || match_method == CV_TM_SQDIFF_NORMED )
    { matchLoc = minLoc; }
  else
    { matchLoc = maxLoc; }

  /// Show me what you got
  rectangle( img_display, matchLoc, Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), Scalar::all(0), 2, 8, 0 );
  rectangle( result, matchLoc, Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), Scalar::all(0), 2, 8, 0 );

  imshow( "dispaly", img_display );
  imshow( "result", result );

  return;
}
//*******************************************THEME_MATCH_END

//********************************************************************************************HULL_DEC
vector<Point> Hull_dec(vector<Point> mycontour, Mat &original)
{
  vector<Vec4i> convexityDefectsSet;
  vector<int>hull;
	convexHull(mycontour, hull, false);
	convexityDefects(Mat(mycontour), hull, convexityDefectsSet);
	vector<Point> allpoint;
	allpoint.resize(convexityDefectsSet.size());
	for (int cDefIt = 0; cDefIt < convexityDefectsSet.size(); cDefIt++)
  {
		int startIdx = convexityDefectsSet[cDefIt].val[0]; Point ptStart(mycontour[startIdx]);
		int endIdx = convexityDefectsSet[cDefIt].val[1]; Point ptEnd(mycontour[endIdx]);
		int farIdx = convexityDefectsSet[cDefIt].val[2]; Point ptFar(mycontour[farIdx]);
		double depth = static_cast<double>(convexityDefectsSet[cDefIt].val[3]) / 256;
		//cout << "depth" << depth << endl;
	   	circle(original, ptStart, 5, CV_RGB(255, 0, 0), 2, 8);
		//display all end points
	   	circle(original, ptEnd, 5, CV_RGB(255, 255, 0), 2, 8);
	    	circle(original, ptFar, 5, CV_RGB(0, 0, 255), 2, 8);
		allpoint[cDefIt] = ptFar;
	}

	//	cout << std::to_string(maxiu)<<'\n';
	return allpoint;
}

//*******************************************************HULL_DEC

//************************************************************************************************CIRCLE_DEC

Mat circle_dec(Mat frame)
{
     vector <Point> allpoint;
  Mat gray;
  cv::cvtColor(frame, gray, CV_BGR2GRAY);
  ///blur
  //GaussianBlur(gray, gray, Size(1, 1), 1, 1);
  vector<Vec3f> circles;
  vector<vector<Point> > contours;
  bool ber=false;
  bool ber1=false;
  /// circle
	cv::HoughCircles(gray, circles, CV_HOUGH_GRADIENT, 1.2,80, 40, 50, 0,50);
  for (int i = 0; i < circles.size(); i++)
  {
  		Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
  		int radius = cvRound(circles[i][2]);
                 circle(frame, center, radius, Scalar(255, 255, 255), 3, 8, 0);
                if((center.x<250||center.x>380)||(center.y<213||center.y>329)){

                  //
            ROS_INFO("---->>>>>%d",  radius);
             if((radius>20&radius<35)||(radius>5&radius<12)){
                  if (center.x - radius>40 && center.y - radius>40&&center.x + radius+40<frame.cols && center.y + radius+40<frame.rows)
          {
              //ROS_INFO("%d",  radius);
    			     // Crop the full image to that image contained by the rectangle myROI
    			     // Note that this doesn't copy the

    			     cv::Mat mmb;
    			     //	Canny(cropped, edge, 50, 200, 3
    			    //	vector<vector<Point> > cont = findColor(cropped, 1, b_l, b_h);
    			     //	drawContours(cropped, cont, -1, Scalar(0, 255, 0), 1, 8);
    			    cv::Mat crop;

              cv::Rect myROI(Point(center.x - radius-18, center.y - radius-18), Point(center.x + radius+18, center.y + radius+18));

    					cv::Mat croppedRef(frame, myROI);
    					croppedRef.copyTo(crop);
              contours=color_dec(crop,CIRCLE_MODE);
              cv::Mat gray_c;
              for (size_t k = 0; k < contours.size(); k++)
  {
       RotatedRect r = minAreaRect( Mat(contours[k]));
     
      if(abs(r.size.width-r.size.height)>20)
      {
          //ROS_INFO("eeeeeeeeeeeeeeeee>>>%f",  abs(r.size.width-r.size.height));
          if(search_o==3){
        putText(frame, "RING_box", center,FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200, 200, 250), 1, CV_AA);
          ROS_INFO("%s",  "RING_box");
                     search_x_y.push_back(Point(center));
                  search_Angle.push_back(float(3));}
          
    }else{
        if(search_o==4){
             //  search_x_y.push_back(Point(center));
                //  search_Angle.push_back(float(3));
            
        }
        
        
    }

}}


            }

      if(radius>15&radius<27)
      {

          //  cv::Rect(Point(center.x - radius, center.y - radius), Point(center.x + radius, center.y + radius));
          if (center.x - radius>50 && center.y - radius>50&&center.x + radius+50<frame.cols && center.y + radius+50<frame.rows)
          {
              //ROS_INFO("%d",  radius);
    			     // Crop the full image to that image contained by the rectangle myROI
    			     // Note that this doesn't copy the

    			     cv::Mat mmb;
    			     //	Canny(cropped, edge, 50, 200, 3
    			    //	vector<vector<Point> > cont = findColor(cropped, 1, b_l, b_h);
    			     //	drawContours(cropped, cont, -1, Scalar(0, 255, 0), 1, 8);
    			    cv::Mat crop;

              cv::Rect myROI(Point(center.x - radius-18, center.y - radius-18), Point(center.x + radius+18, center.y + radius+18));

    					cv::Mat croppedRef(frame, myROI);
    					croppedRef.copyTo(crop);
              contours=color_dec(crop,CIRCLE_MODE);
              cv::Mat gray_c;
              for (size_t k = 0; k < contours.size(); k++)
  {
    //  if(  contourArea(contours0[k])>ARC){
  	 RotatedRect r = minAreaRect( Mat(contours[k]));
//r.size.width > r.size.height
if(abs(r.size.width-r.size.height)<3) ber1=true;
     if(r.size.width>20&&r.size.width<40&&r.size.height>20&&r.size.height<40) ber=true; else{

         if(r.size.width>70&&r.size.width<100&&r.size.height>70&&r.size.height<100);else
         if(r.size.width>70&&r.size.width<100||r.size.height>70&&r.size.height<100)
         ROS_INFO("%s",  "boolbox");}




  }
              if(contours.size()>0&&allpoint.empty())
              {
                vector<Vec3f> circles1;
                cv::cvtColor(crop, gray_c, CV_BGR2GRAY);
                /// circle
              	cv::HoughCircles(gray_c, circles1, CV_HOUGH_GRADIENT, 1.5,60, 30, 60, 0, radius);
                for (int k = 0; k < circles1.size(); k++){
                  //  Point center(cvRound(circles1[i0][0]), cvRound(circles1[i0][1]));
                    Point center1(cvRound(circles1[k][0]), cvRound(circles1[k][1]));

                    int radius1 = cvRound(circles1[k][2]);
               // ROS_INFO("%d",  radius1);
                //circle(frame, center1, radius1, Scalar(255, 255, 255), 3, 8, 0);
                     ROS_INFO("ttt %d",  radius1);
                    if(radius1>12&&radius1<23&&search_o==4&&ber&&ber1){
                    ROS_INFO("%s",  "bool");
                     search_x_y.push_back(Point(center));
                  search_Angle.push_back(float(3));
                    putText(frame, "BOOL", center,
                    FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200, 200, 250), 1, CV_AA);
                    cx=center.x;
                    cy=center.y;
                }}
              }
              else
              {
                  if(search_o==7&&allpoint.empty()){
                      Mat src;
                       vector<Vec3f> circles1;
                       vector<Vec4i> lines;
                      cv::HoughCircles(gray, circles1, CV_HOUGH_GRADIENT, 1.2,60, 30, 60, 0,50);
                      for (int k = 0; k < circles1.size(); k++){
                  //  Point center(cvRound(circles1[i0][0]), cvRound(circles1[i0][1]));
                    Point center1(cvRound(circles1[k][0]), cvRound(circles1[k][1]));
                         Canny(crop, src, 50, 200, 3);
                         HoughLinesP(src, lines, 1, CV_PI/180, 50, 50, 10 );
                         for( size_t i = 0; i < lines.size(); i++ )
  {
    Vec4i l = lines[i];
   // line( crop, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
  }
                    int radius1 = cvRound(circles1[k][2]);
               // ROS_INFO("%d",  radius1);
                //circle(frame, center1, radius1, Scalar(255, 255, 255), 3, 8, 0);
                    if(radius1>15&&radius1<27){
                          ROS_INFO("%s",  "ring");
               
                  search_x_y.push_back(Point(center));
                  search_Angle.push_back(float(3));
                    }}

              }}
              cv::imshow( "Display window11", crop );
          }

      }
    }


}

    return frame;
}


//*****************************************************CIRCLE_DEC
//*********************************************************************************************************COLOR_BOX(0)
vector<vector<Point> > BOX_dec(Mat frame,int Mode)
{
  cv::Scalar b_l;
  cv::Scalar b_h;
  int DP=0;
  int ARC=0;
  cv::Mat imgThreshold, imgThreshold0, imgThreshold1;//

    if(Mode==0){/////RED
      b_l = Scalar(0, 0, 0);
      b_h = Scalar(10, 255, 255);
      DP=1;
      ARC=50;

    }else{
      /////BLUE
      b_l = Scalar(100, 0, 0);
      b_h = Scalar(120, 255, 255);
      DP=1;
    }



  Mat hierarchy,hsv;
  vector<vector<Point> > contours0;
  vector<Point> contour;
  cv::cvtColor(frame, hsv, CV_BGR2HSV);
  GaussianBlur(hsv, hsv, Size(5, 5), 9, 9);
  cv::inRange(hsv, b_l, b_h, imgThreshold0);
  cv::inRange(hsv, b_l, b_h, imgThreshold1);
	cv::bitwise_or(imgThreshold0, imgThreshold1, imgThreshold);
  findContours(imgThreshold, contours0, hierarchy, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

  //contours.resize(contours0.size());

	double maxiu = 0;
	for (size_t k = 0; k < contours0.size(); k++)
  {
     if(  contourArea(contours0[k])>maxiu){

   maxiu= contourArea(contours0[k]);
   contour=contours0[k];
    }
  }
  if(!contour.empty()){
   RotatedRect r = minAreaRect( Mat(contour));
   double max_p=0;
 double max_l=0;
 if(r.size.height>=r.size.width){

   max_p =r.size.height;
   max_l =r.size.width;
 }else{

 max_l =r.size.height;
 max_p =r.size.width;
 }

 if(max_p>100 && max_l>100&&max_p<500 && max_l<500){
     //drawContours(frame, contour, 0, CV_RGB(0, 255, 0), 0.01, 8, hierarchy);
       search_x_y.push_back(Point(r.center));
              search_Angle.push_back(float(3));
              putText(frame, "box"+Mode, r.center,
								FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200, 200, 250), 1, CV_AA);
 }

    }
      cv::imshow( "meter eee", imgThreshold );
  return contours0;
}
//*****************************************************************COLOR_DEC(0)_END

//*********************************************************************************************************COLOR_DEC(0)
vector<vector<Point> > color_dec(Mat frame,int Mode)
{
  cv::Scalar b_l;
  cv::Scalar b_h;
  int DP=1;
  int ARC=0;
  cv::Mat imgThreshold, imgThreshold0, imgThreshold1;//
  switch(Mode)
  {
    case 0:
      b_l = Scalar(0, 0, 0);
      b_h = Scalar(255, 255, 50);
      DP=1;
      ARC=50;
    break;
    case 1:
      b_l = Scalar(0, 0, 0);
      b_h = Scalar(255, 255, 50);
      DP=1;
    break;
    case 2:
      b_l = Scalar(0, 0, 0);
      b_h = Scalar(255, 255, 50);
    break;
  }

  Mat hierarchy,hsv;
  vector<vector<Point> > contours0;
  vector<vector<Point> > contours;
  cv::cvtColor(frame, hsv, CV_BGR2HSV);
  GaussianBlur(hsv, hsv, Size(5, 5), 9, 9);
  cv::inRange(hsv, b_l, b_h, imgThreshold0);
	cv::inRange(hsv, b_l, b_h, imgThreshold1);
	cv::bitwise_or(imgThreshold0, imgThreshold1, imgThreshold);
  findContours(imgThreshold, contours0, hierarchy, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

  contours.resize(contours0.size());

  for (size_t k = 0; k < contours0.size(); k++)
  {
    //  if(  contourArea(contours0[k])>ARC){

  	approxPolyDP(Mat(contours0[k]), contours[k], DP, true);
    //drawContours(frame, contours, k, CV_RGB(0, 255, 0), 0.01, 8, hierarchy);
    //}
  }
  if(Mode==0)return contours;else
  return contours;
}
//*****************************************************************COLOR_DEC(0)_END
void rot_table(Mat frame,int R_table,float D_R_T,float D_H_C,float D_W_C)
{



}
//***************************************************************************************************************COLOR_DEC(1)
vector<Point>  color_dec1(Mat frame)
{
  cv::Scalar b_l;
  cv::Scalar b_h;
  b_l = Scalar(0, 0, 0);
  b_h = Scalar(255, 255, 50);
  cv::Mat imgThreshold, imgThreshold0, imgThreshold1;//
  Mat hierarchy,hsv;
  vector<vector<Point> > contours0;
  vector<vector<Point> > contours;
  cv::cvtColor(frame, hsv, CV_BGR2HSV);
  GaussianBlur(hsv, hsv, Size(5, 5), 9, 9);
  cv::inRange(hsv, b_l, b_h, imgThreshold0);
  cv::inRange(hsv, b_l, b_h, imgThreshold1);
  cv::bitwise_or(imgThreshold0, imgThreshold1, imgThreshold);
  findContours(imgThreshold, contours0, hierarchy, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
  vector<Point> contour;
  contours.resize(contours0.size());
  int max_=0;
  for (size_t k = 0; k < contours0.size(); k++)
  {
      //ROS_INFO("errrrrrr %d",contourArea(contours0[k]));
      contour=contours[k];
      approxPolyDP(Mat(contours0[k]), contours[k], 1.5, true);
    //  max_=contourArea(contours0[k]);
      contour=contours[k];
//drawContours(frame, contours0, k, CV_RGB(0, 255, 0), 0.01, 8, hierarchy);


  }
return contour;

}
//****************************************************************************END_COLOR_DEC
//****************************************************************************************************************GET_PPT
bool getPPT(vector<Point> allpoint,Point center){

  double res;
	double maxiu = 0;
	for (int i = 0; i < allpoint.size(); i++)
  {
		for (int j = 0; j < allpoint.size(); j++)
    {
			res = cv::norm(allpoint[i] - allpoint[j]);
			if (res>25&&res<40){
                            if(center.y<(allpoint[j].y+allpoint[i].y)/2)
                                return true;



                        }
      {
				maxiu = res;
			}
		}
	}

  return false;
}
//*******************************************************************************************************************GET_PPT_PICH
bool GET_PPT_PICH(Mat frame,Point center,float height,float width){

     if (center.x - width/2>30 && center.y - height/2>30&&center.x + width/2+20<frame.cols && center.y + height/2+20<frame.rows)
          {
              //ROS_INFO("%d",  radius);
    			     // Crop the full image to that image contained by the rectangle myROI
    			     // Note that this doesn't copy the

    			     cv::Mat mmb;
    			     //	Canny(cropped, edge, 50, 200, 3
    			    //	vector<vector<Point> > cont = findColor(cropped, 1, b_l, b_h);
    			     //	drawContours(cropped, cont, -1, Scalar(0, 255, 0), 1, 8);
    			    cv::Mat crop;

              cv::Rect myROI(Point(center.x - width/2, center.y - height/2), Point(center.x + width/2, center.y + height/2));

    					cv::Mat croppedRef(frame, myROI);
    					croppedRef.copyTo(crop);

                                          cv::imshow( "Display window11", crop );
        }



  return false;
}

//****************************************************************************************************************GET_MAX
double getMax(vector<Point> allpoint){

  double res;
	double maxiu = 0;
	for (int i = 0; i < allpoint.size(); i++)
  {
		for (int j = 0; j < allpoint.size(); j++)
    {
			res = cv::norm(allpoint[i] - allpoint[j]);
			if (res > maxiu)
      {
				maxiu = res;
			}
		}
	}

  return maxiu;
}
//***********************************************************************GET_MAX_END
//***************************************************************************************************************SHAPE_DEC
Mat shape_dec(Mat frame)
{
    Rect boundRect;
  vector<vector<Point> > contours;
  contours=color_dec(frame,1);
    //ROS_INFO("er %d : %d",  contourArea(O40x40), contourArea(contours[0]));
    double qp=0;
  for (size_t k = 0; k < contours.size(); k++)
  {
    if(  contourArea(contours[k])>100)
    {
////////////////////////////////////////////////////////////////////////////////40x40//**
        if (search_o==2){
            RotatedRect r = minAreaRect( Mat(contours[k]));
            vector <Point> allpoint1=Hull_dec(contours[k],frame);
  qp = matchShapes(contours[k],O40x40, CV_CONTOURS_MATCH_I2, 0.00);

///  GET_PPT_PICH(frame,r.center,r.size.height,r.size.width);
    if(r.size.width<180&&r.size.width>170&&r.size.height<80&&r.size.height>70){
                   ROS_INFO("%s",  "s40-40");
                  putText(frame, "s40-40black", r.center,
								FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200, 200, 250), 1, CV_AA);

                   if (r.size.width > r.size.height)
              {
                r.angle += 90;
              }

              r.angle = (((r.angle+90)*3.141592)/180) + 1.4229;

                   // ROS_INFO("%f - %f - %f",r.size.height,r.size.width, r.angle);
              search_x_y.push_back(Point(r.center));
              search_Angle.push_back(float(r.angle));

            }
             if(r.size.height<180&&r.size.height>170&&r.size.width<80&&r.size.width>70){
                  ROS_INFO("%s",  "s40-40");
                  putText(frame, "s40-40black", r.center,
								FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200, 200, 250), 1, CV_AA);
                   if (r.size.width > r.size.height)
              {
                r.angle += 90;
              }

              r.angle = (((r.angle+90)*3.141592)/180) + 1.4229;

                    // ROS_INFO("%f - %f - %f",r.size.height,r.size.width, r.angle);
                       search_x_y.push_back(Point(r.center));
              search_Angle.push_back(float(r.angle));

            }
    if(qp<0.4){

     RotatedRect r = minAreaRect( Mat(contours[k]));
      vector <Point> allpoint=Hull_dec(contours[k],frame);
      double max_p =getMax(allpoint);
    //   ROS_INFO("%f",max_p);

if(max_p>140*Scale &&max_p<195*Scale ){
      Moments mu;
      Rect boundRect;
							mu = moments(contours[k], false);
							Point2f mc = Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);

              boundRect = boundingRect( contours[k]);
              rectangle( frame, boundRect.tl(), boundRect.br(), cvScalar(200, 200, 250), 2, 8, 0 );
              if (r.size.width > r.size.height)
              {
                r.angle += 90;
              }

              r.angle = (((r.angle+90)*3.141592)/180) + 1.4229;
                ROS_INFO("%s",  "40black");

							putText(frame, "s40-40black", mc,
								FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200, 200, 250), 1, CV_AA);
     search_x_y.push_back(Point(r.center));
              search_Angle.push_back(float(r.angle));



}}}
///***************************************************************************************
///////////////////////////////////////////////////////////////////////////////20x20//**
if (search_o==1){
RotatedRect r = minAreaRect( Mat(contours[k]));
             // boundRect = boundingRect( contours[k]);
              //rectangle( frame, boundRect.tl(), boundRect.br(), cvScalar(200, 200, 250), 2, 8, 0 );

              //ROS_INFO("%f",r.size.width);
              //ROS_INFO("%f",r.size.height);
              if(r.size.width<200&&r.size.width>160&&r.size.height<50&&r.size.height>30){
                   ROS_INFO("%s",  "s20-20");
                  putText(frame, "s20-20black", r.center,
								FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200, 200, 250), 1, CV_AA);

                   if (r.size.width > r.size.height)
              {
                r.angle += 90;
              }

              r.angle = (((r.angle+90)*3.141592)/180) + 1.4229;

                   // ROS_INFO("%f - %f - %f",r.size.height,r.size.width, r.angle);
              search_x_y.push_back(Point(r.center));
              search_Angle.push_back(float(r.angle));

            }
             if(r.size.height<200&&r.size.height>160&&r.size.width<50&&r.size.width>30){
                  ROS_INFO("%s",  "s20-20");
                  putText(frame, "s20-20black", r.center,
								FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200, 200, 250), 1, CV_AA);
                   if (r.size.width > r.size.height)
              {
                r.angle += 90;
              }

              r.angle = (((r.angle+90)*3.141592)/180) + 1.4229;

                    // ROS_INFO("%f - %f - %f",r.size.height,r.size.width, r.angle);
                       search_x_y.push_back(Point(r.center));
              search_Angle.push_back(float(r.angle));

            }

  qp = matchShapes(contours[k],O20x20, CV_CONTOURS_MATCH_I3, 0.00);
    if(qp<0.5){
      vector <Point> allpoint=Hull_dec(contours[k],frame);

     double max_p =getMax(allpoint);

if(false){


 RotatedRect r = minAreaRect( Mat(contours[k]));
              boundRect = boundingRect( contours[k]);
              rectangle( frame, boundRect.tl(), boundRect.br(), cvScalar(200, 200, 250), 2, 8, 0 );
              if (r.size.width > r.size.height)
              {
                r.angle += 90;
              }

              r.angle = (((r.angle+90)*3.141592)/180) + 1.4229;

             //ROS_INFO("%f - %f - %f",r.size.height,r.size.width, r.angle);
              search_x_y.push_back(Point(r.center));
              search_Angle.push_back(float(r.angle));
               ROS_INFO("%s",  "s20-20");
      Moments mu;
							mu = moments(contours[k], false);
							Point2f mc = Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);
							putText(frame, "s20-20black", mc,
								FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200, 200, 250), 1, CV_AA);}}}
///***************************************************************************************

///////////////////////////////////////////////////////////////////////////////mohre//**

if (search_o==5||search_o==6||search_o==4){








  qp = matchShapes(contours[k],mohre_G, CV_CONTOURS_MATCH_I1, 0.00);
    if(qp<0.5){
      vector <Point> allpoint=Hull_dec(contours[k],frame);
      double max_p =getMax(allpoint);
 //RotatedRect r = minAreaRect( Mat(contours[k]));
        double max_A=contourArea(contours[k]);

if(max_A>4300*Scale &&max_A<7500*Scale&&search_o==5 ){
    RotatedRect r = minAreaRect( Mat(contours[k]));
      Moments mu;
if((r.size.height<100&&r.size.height>82 && r.size.width>75 && r.size.width<90)||(r.size.width<100&&r.size.width>82 && r.size.height>75 && r.size.height<90)){

      ROS_INFO("%s",  "mohre_G");
							mu = moments(contours[k], false);
							Point2f mc = Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);
							putText(frame, "mohre_B", mc,
								FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200, 200, 250), 1, CV_AA);


                                           search_x_y.push_back(Point(mc));
              search_Angle.push_back(3);

}}
                Moments mu;
                mu = moments(contours[k], false);
                Point2f mc = Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);
                 ROS_INFO("%s",  "WWWWWWWWW");
    if(max_A>1200*Scale &&max_A<3000*Scale && (search_o==6||search_o==4)){
         RotatedRect r = minAreaRect( Mat(contours[k]));
        // ROS_INFO("abs %f",abs(r.size.height-r.size.width));
        if(  ((r.size.height<65&&r.size.height > 50 && r.size.width > 40 && r.size.width<60 )  ||  (r.size.width<65&&r.size.width>50 && r.size.height > 40 && r.size.height<60 ))){
        if(abs(r.size.height-r.size.width)>4){
        double max_A=contourArea(contours[k]);
      //  ROS_INFO("%f",r.size.height);
if(search_o==6){
search_x_y.push_back(Point(mc));
              search_Angle.push_back(3);
 ROS_INFO("%s",  "mohre_s");
                							putText(frame, "mohre_S", mc,
                								FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200, 200, 250), 1, CV_AA);
}
        }else{
if(search_o==4&&abs(r.size.height-r.size.width)<4){
    search_x_y.push_back(Point(mc));
              search_Angle.push_back(3);
    
       putText(frame, "bool", mc,
                								FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200, 200, 250), 1, CV_AA);       
              
              
}
            
                							


        }


        }}


              }}
///***************************************************************************************

///////////////////////////////////////////////////////////////////////////////motor//**
if (search_o==12){
  qp = matchShapes(contours[k],motor, CV_CONTOURS_MATCH_I2, 0.00);
    if(qp<0.8){
      vector <Point> allpoint=Hull_dec(contours[k],frame);
      double max_p =getMax(allpoint);
//ROS_INFO("%f",max_p);
      RotatedRect r = minAreaRect( Mat(contours[k]));
      if((r.size.height<180&&r.size.height>140&& r.size.width>68 && r.size.width<82)||(r.size.width<180&&r.size.width>140 && r.size.height>68 && r.size.height<82)){
      Moments mu;

   //ROS_INFO("%f",r.size.height);
   // ROS_INFO("%f",r.size.width);
              boundRect = boundingRect( contours[k]);
              rectangle( frame, boundRect.tl(), boundRect.br(), cvScalar(200, 200, 250), 2, 8, 0 );
              if (r.size.width > r.size.height)
              {
                r.angle += 90;
              }

              r.angle = (((r.angle+90)*3.141592)/180) + 1.4229;

              //ROS_INFO("%f - %f - %f",r.size.height,r.size.width, r.angle);
               ROS_INFO("%s",  "motor");
							mu = moments(contours[k], false);
							Point2f mc = Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);
							putText(frame, "motor", mc,
								FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200, 200, 250), 1, CV_AA);
     search_x_y.push_back(Point(r.center));
              search_Angle.push_back(float(r.angle));



}}}
///***************************************************************************************
///////////////////////////////////////////////////////////////////////////////tube//**
if (search_o==8){
  qp = matchShapes(contours[k],motor, CV_CONTOURS_MATCH_I3, 0.00);
    if(qp<0.8){
      vector <Point> allpoint=Hull_dec(contours[k],frame);

      RotatedRect r = minAreaRect( Mat(contours[k]));
 double max_p=0;
 double max_l=0;
if(r.size.height>=r.size.width){

   max_p =r.size.height;
   max_l =r.size.width;
}else{

 max_l =r.size.height;
max_p =r.size.width;
}
//ROS_INFO("%f",r.size.height);
//ROS_INFO("%f",r.size.width);
if((r.size.height<100&&r.size.height>78 && r.size.width>48 && r.size.width<60)||(r.size.width<100&&r.size.width>78 && r.size.height>48 && r.size.height<60)){
      Moments mu;

              boundRect = boundingRect( contours[k]);
              rectangle( frame, boundRect.tl(), boundRect.br(), cvScalar(200, 200, 250), 2, 8, 0 );
              if (r.size.width > r.size.height)
              {
                r.angle += 90;
              }


              r.angle = (((r.angle+90)*3.141592)/180) + 1.4229;

              //ROS_INFO("%f - %f - %f",r.size.height,r.size.width, r.angle);
               ROS_INFO("%s",  "tube");
							mu = moments(contours[k], false);
							Point2f mc = Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);
							putText(frame, "tube", mc,
								FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200, 200, 250), 1, CV_AA);
     search_x_y.push_back(Point(r.center));
              search_Angle.push_back(float(r.angle));



}}}
///***************************************************************************************
///////////////////////////////////////////////////////////////////////////////pich//**
if (search_o==26){
 RotatedRect r = minAreaRect( Mat(contours[k]));

  if((r.size.height<220&&r.size.height>160&&r.size.width<75&&r.size.width>45)||(r.size.width<220&&r.size.width>160&&r.size.height<75&&r.size.height>45)){
       vector <Point> allpoint=Hull_dec(contours[k],frame);
      if (r.size.width > r.size.height)
              {
                r.angle += 90;
              }

              r.angle = (((r.angle+90)*3.141592)/180) + 1.4229;
              if(getPPT(allpoint,r.center)){
                  if(r.angle>1.3529&&r.angle<2.6){

                      r.angle+=3.1415;
                }else if(r.angle>3.25&&r.angle<4.4929){
                    r.angle-=3.1415;
                }

            }

         //     ROS_INFO("%f - %f - %f",r.size.height,r.size.width, r.angle);
                     // ROS_INFO("%s",  "pich");

                        search_x_y.push_back(Point(r.center));
              search_Angle.push_back(float(r.angle));


							putText(frame, "pich_box", r.center,
								FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200, 200, 250), 1, CV_AA);}}
if (search_o==13){
     RotatedRect r = minAreaRect( Mat(contours[k]));
  qp = matchShapes(contours[k],pich, CV_CONTOURS_MATCH_I1, 0.00);
  //ROS_INFO("%f ",qp);
    if(qp<1.2){
      vector <Point> allpoint=Hull_dec(contours[k],frame);
      double max_p =getMax(allpoint);

if(true){
      Moments mu;

              boundRect = boundingRect( contours[k]);
            //  rectangle( frame, boundRect.tl(), boundRect.br(), cvScalar(200, 200, 250), 2, 8, 0 );
              if (r.size.width > r.size.height)
              {
                r.angle += 90;
              }

              r.angle = (((r.angle+90)*3.141592)/180) + 1.4229;
              if(getPPT(allpoint,r.center)){
                  if(r.angle>1.3529&&r.angle<2.6){

                      r.angle+=3.1415;
                }else if(r.angle>3.25&&r.angle<4.4929){
                    r.angle-=3.1415;
                }

            }

             // ROS_INFO("%f - %f - %f",r.size.height,r.size.width, r.angle);


               if((r.size.height<210&&r.size.height>160&&r.size.width<75&&r.size.width>45)||(r.size.width<210&&r.size.width>160&&r.size.height<75&&r.size.height>45)){
                      ROS_INFO("%s",  "pich");

                        search_x_y.push_back(Point(r.center));
              search_Angle.push_back(float(r.angle));
							mu = moments(contours[k], false);
							Point2f mc = Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);
							putText(frame, "pich", mc,
								FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200, 200, 250), 1, CV_AA);}}}}
///***************************************************************************************
  }}
  return frame;
}

//***********************************************************************************GET_OBJ
int get_obj()
{
  Mat image;
  image = imread("/home/iauk/catkin_ws/src/beginner_tutorials/40x40.png", CV_LOAD_IMAGE_COLOR);
O40x40=color_dec1(image);
ROS_INFO("40x40.png----->OK!!!!!");
///////////////////////////////////////////////////////////////////////////////////////////////
image = imread("/home/iauk/catkin_ws/src/beginner_tutorials/20x20.png", CV_LOAD_IMAGE_COLOR);
O20x20=color_dec1(image);
ROS_INFO("20x20.png----->OK!!!!!");
///////////////////////////////////////////////////////////////////////////////////////////////
image = imread("/home/iauk/catkin_ws/src/beginner_tutorials/pich.png", CV_LOAD_IMAGE_COLOR);
pich=color_dec1(image);
ROS_INFO("pich.png----->OK!!!!!");
//////////////////////////////////////////////////////////////////////////////////////////////
image = imread("/home/iauk/catkin_ws/src/beginner_tutorials/mohre_B.png", CV_LOAD_IMAGE_COLOR);
mohre_G=color_dec1(image);
ROS_INFO("mohre_B.png----->OK!!!!!");
//////////////////////////////////////////////////////////////////////////////////////////////
image = imread("/home/iauk/catkin_ws/src/beginner_tutorials/mohre_S.png", CV_LOAD_IMAGE_COLOR);
mohre_S=color_dec1(image);
ROS_INFO("mohre_S.png----->OK!!!!!");
////////////////////////////////////////////////////////////////////////////////////////////
image = imread("/home/iauk/catkin_ws/src/beginner_tutorials/motor.png", CV_LOAD_IMAGE_COLOR);
motor=color_dec1(image);
ROS_INFO("motor.png----->OK!!!!!");
////////////////////////////////////////////////////////////////////////////////////////////
image = imread("/home/iauk/catkin_ws/src/beginner_tutorials/20x20W0.png", CV_LOAD_IMAGE_COLOR);
W20x20=image.clone();
ROS_INFO("W20x20.png----->OK!!!!!");
///////////////////////////////////////////////////////////////////////////////////////////
image = imread("/home/iauk/catkin_ws/src/beginner_tutorials/tube.png", CV_LOAD_IMAGE_COLOR);
tube=color_dec1(image);
ROS_INFO("tube.png----->OK!!!!!");




  /*image = imread("~/catkin_ws/src/beginner_tutorials/20x20.png", CV_LOAD_IMAGE_COLOR);
  O20x20=color_dec(image);
  image = imread("~/catkin_ws/src/beginner_tutorials/mohre_B.png", CV_LOAD_IMAGE_COLOR);
  mohre_G=color_dec(image);
  image = imread("~/catkin_ws/src/beginner_tutorials/mohre_S.png", CV_LOAD_IMAGE_COLOR);
  mohre_S=color_dec(image);
  image = imread("~/catkin_ws/src/beginner_tutorials/motor.png", CV_LOAD_IMAGE_COLOR);
  motor=color_dec(image);*/
  return 0;
}
//***********************************************************************************GET_OBJ_END


void publish_meter(ros::NodeHandle n)
{
    cv::Mat image;
    cap >> image;

    Mat hsv;
    cvtColor(image, hsv, COLOR_BGR2HSV);

    GaussianBlur( hsv, hsv, Size( 5, 5 ), 9, 9 );


    int row_image,col_image;
    Mat1b mask;
    Mat1b mask_y;
    Mat1b mask_b;
    inRange(hsv, Scalar(minH_y, minS_y, minV_y), Scalar(maxH_y, maxS_y, maxV_y), mask_y);
    inRange(hsv, Scalar(minH_b, minS_b, minV_b), Scalar(maxH_b, maxS_b, maxV_b), mask_b);
		//bitwise_or(mask_y, mask_b, mask);
		mask = mask_y;


    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    cv::Size s=mask.size();
    row_image=mask.rows;
    col_image=mask.cols;
    int rec=30;
    float w_size=mask.cols/rec;
    float h_size=mask.rows/rec;


    //////////////////////////////////////////////
    /*cv::Rect myROI(w_size*0,h_size*0, w_size*(0+1)-w_size*0, h_size*(0+1)-h_size*0);
    cv::Mat croppedImage = mask(myROI);
     findContours( croppedImage, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE );
            for (size_t idx = 0; idx < contours.size(); idx++) {
            int bbb=cv::contourArea(contours[idx]);
           // ROS_INFO("%d",bbb );
    }
    cv::imshow( "Display window11", croppedImage );
    */
    ////////////////////////////

    for(int i=0;i<360;i++)
    {
        ranges[i] = NAN;
    }

		for(int i=142;i<=220;i++)
    {
        ranges[i] = 10;
    }


    for(int i=0;i<rec;i++)
    {
        for(int j=0;j< rec;j++)
        {
            vector<vector<Point> > contours;
            vector<Vec4i> hierarchy;

            cv::Rect myROI(w_size*j,h_size*i, w_size*(j+1)-w_size*j, h_size*(i+1)-h_size*i);
            cv::Mat croppedImage = mask(myROI);
            findContours( croppedImage, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE );
            for (size_t idx = 0; idx < contours.size(); idx++)
            {
                int bbb=cv::contourArea(contours[idx]);
                if(bbb>50)
                {

                    //ROS_INFO("i: %d  j: %d  ",i,j);
                    ranges[obs_points[i][j][0]] = (double)(obs_points[i][j][1]) / 100;
                    //intensities[obs_points[i][j][0]] = 24;
                }

            }

            rectangle(image,Point(w_size*j,h_size*i), Point(w_size*(j+1),h_size*(i+1)), Scalar(0, 255, 0), 1, 8, 0);
        }
    }

    //ROS_INFO("209: %f  210: %f  ",ranges[209],ranges[210]);

    ros::Time scan_time = ros::Time::now();
    sensor_msgs::LaserScan scan;
    scan.header.stamp = scan_time;
    scan.header.frame_id = "laser_frame2";
    //scan.angle_min = -3.1241;
    //scan.angle_max = 3.141592;
    //scan.angle_increment = 0.017453;
		scan.angle_min = -0.75;
    scan.angle_max = 0.82;
    scan.angle_increment = 0.017453;

    scan.time_increment = 0.0006;
    scan.range_min = 0.015;
    scan.range_max = 16.0;


    /*scan.ranges.resize(360);
    scan.intensities.resize(360);

    for(unsigned int i = 0; i < 360; i++)
    {
      scan.ranges[i] = ranges[i];
      scan.intensities[i] = intensities[i];
    }*/
		scan.ranges.resize(90);
    scan.intensities.resize(90);

		for(unsigned int i = 0; i < 90; i++)
    {
      scan.ranges[i] = ranges[i+137];
      scan.intensities[i] = intensities[i+137];
    }

    scan_pub.publish(scan);

    cv::imshow( "Display window", image );

    cv::waitKey(1);


    //ROS_INFO("%d", minH);
}

void findObjectCallback(const std_msgs::String::ConstPtr& msg)
{
  if (msg->data == "Small Black Alu. Profile") search_o=1;
  if (msg->data == "Large Black Alu. Profile") search_o=2;
  if (msg->data == "Bearing Box") search_o=3;
  if (msg->data == "Bearing") search_o=4;
  if (msg->data == "Large Nut") search_o=5;
  if (msg->data == "Small Nut") search_o=6;
  if (msg->data == "Distance Tube") search_o=7;
  if (msg->data == "Plastic Tube") search_o=8;
  if (msg->data == "Small Grey Alu. Profile") search_o=9;
  if (msg->data == "Large Grey Alu. Profile") search_o=10;
  if (msg->data == "Axis") search_o=11;
  if (msg->data == "Motor") search_o=12;
  if (msg->data == "Bolt") search_o=13;
  if (msg->data == "Small Black Alu. Profile_place") search_o=1;
  if (msg->data == "Large Black Alu. Profile_place") search_o=2;
  if (msg->data == "Bearing Box_place") search_o=16;
  if (msg->data == "Bearing_place") search_o=17;
  if (msg->data == "Large Nut_place") search_o=5;
  if (msg->data == "Small Nut_place") search_o=6;
  if (msg->data == "Distance Tube_place") search_o=4;
  if (msg->data == "Plastic Tube_place") search_o=8;
  if (msg->data == "Small Grey Alu. Profile_place") search_o=1;
  if (msg->data == "Large Grey Alu. Profile_place") search_o=2;
  if (msg->data == "Axis_place") search_o=24;
  if (msg->data == "Motor_place") search_o=12;
  if (msg->data == "Bolt_place") search_o=26;
  if (msg->data == "Blue Container") search_o=27;
  if (msg->data == "Red Container") search_o=28;


  n_global->setParam("/vision/object_status", "search" );


    for(size_t fr0=0;fr0<5;fr0++)
    {
         cv::Mat dst;
 cv::Mat frame;
      //  
        cap>>frame;}
    for(size_t fr0=0;fr0<30;fr0++)
    {
        cv::Mat dst;
        cv::Mat frame;
       // ROS_INFO("%s", "fuck" );
        cap>>frame;
        //search= msg->data;
        if(search_o==7||search_o==4||search_o==3)
        {
            try{
            GaussianBlur( frame, frame, Size( 3, 3),3, 3 );
            frame=circle_dec(frame);
            }catch(cv::Exception e){}
            frame=shape_dec(frame);
        }
        else if(search_o==27)
        {
            BOX_dec(frame,1);
            //ROS_INFO("%s", "blue" );
            //  ROS_INFO("%d",  BOX_dec(frame,1).size() );
            if(!search_x_y.empty())
                ROS_INFO("%d",search_x_y[0].x);
        }
        else if(search_o==28)
        {
            BOX_dec(frame,0);
            //ROS_INFO("%s", "red" );
            // ROS_INFO("%d",  BOX_dec(frame,0).size() );
            if(!search_x_y.empty())
                ROS_INFO("%d",search_x_y[0].x);

        }
        else if(search_o==9)
        {
            ROS_INFO("%s", "search_GRAY_S" );
            frame=finder_white_show(frame);
            if(fr0<15)fr0=15;

        } 
        else if(search_o==10)
        {
             ROS_INFO("%s", "search_GRAY_B" );
            frame=finder_white_show(frame);
            if(fr0<15)fr0=15;
        }
        else if(search_o==11)
        {
            frame=finder_white_shaft(frame);
           // if(fr0<40)fr0=40;
        }
        else if(search_o!=0)
        {
            ROS_INFO("%s", "search_SHAPE" );
            frame.convertTo(dst,CV_8U,1,1);
            frame=shape_dec(dst);

        }else{
            ROS_INFO("%s", "ID NOT FOUND" );
        }

        if(!search_x_y.empty())
        {
            if(search_x_y[0].x<60||search_x_y[0].y<0||search_x_y[0].y>frame.size().height-100|| search_x_y[0].x>frame.size().width-60)
            {
                search_x_y.clear();
            }
        }
         rectangle( frame, Point(60,0), Point(frame.size().width-60,frame.size().height-60), cvScalar(0, 200, 250), 2, 8, 0 );
         if(!search_x_y.empty())
         {

                    n_global->setParam("/vision/object_x",search_x_y[0].x );
                    n_global->setParam("/vision/object_y",search_x_y[0].y );
                    n_global->setParam("/vision/object_a",search_Angle[0] );
                    n_global->setParam("/vision/object_status", "found" );

                    ROS_INFO("%d",search_x_y[0].x);
                    cv::imshow( "fuckend", frame );
                    break;
        }






    }
    ROS_INFO("%s", "fuck end" );

    cv::waitKey(1);

    if(search_x_y.empty())
    {
        ROS_INFO("%s", "fuck not found" );
        n_global->setParam("/vision/object_status","notFound" );
    }
    else
    {
        ROS_INFO("%s", "fuck ok " );
        search_o =0;
        search_x_y.clear();
        search_Angle.clear();
    }


  //}
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  scan_pub = n.advertise<sensor_msgs::LaserScan>("scan2", 50);

  n_global = &n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("vision/fine", 100);
  ros::Subscriber find_sub = n.subscribe("vision/find", 100, findObjectCallback);
  ROS_INFO("ok");

  if(!cap.open(1))
  {
      return 0;
  }

  ros::Rate loop_rate(10);

  int count = 0;


  //ros::spin();


  ///////////////////////////////////////////////////
  /*cv::Mat image;
  image = imread("/home/user/catkin_ws/src/beginner_tutorials/1.png", CV_LOAD_IMAGE_COLOR);   // Read the file

    if(! image.data )                              // Check for invalid input
    {
        ROS_INFO("%s", "fuck" );
    }



    namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
    cv::imshow( "Display window", image );

  cv::waitKey(1);

  imwrite( "/home/user/catkin_ws/src/beginner_tutorials/2.png", image );
  */
  ///////////////////////////////////////////////////

  get_obj();
  Mat src_gray;
  //Mat dst;
  while (ros::ok())
  {
      n.getParam("/vision/meterEnable", meterEnable);

      if(meterEnable == true)
      {
          publish_meter(n);
      }
      if (true)
      {
         // for(int qw=0;qw<100;qw++){
  Mat dst;
        cv::Mat frame;
        cap>>frame;
      //  frame=finder_white_shaft(frame);
       // cv::Mat dst
        //ROS_INFO("%s", "fuck" );
        

       // MatchingMethod(frame,W20x20,CV_TM_SQDIFF_NORMED);
       //frame = finder_white_shaft(frame);
    //   GaussianBlur( frame, dst, Size(3.5,3.5),3, 3 );
       // search_o=3;
   /// frame=circle_dec(frame);
    //   //
   // frame=shape_dec(frame);
      //finder_white_shaft(frame);


       //s BOX_dec(frame,1);
 //rectangle( frame, Point(search_x_y.250), 2, 8, 0 );

       cv::imshow( "Display meter", frame );
        cv::waitKey(1);
     // }

      }


      if (false)
      {
        cv::Mat image;
         cv::Mat frame;
        cap >> image;


        int maxH = 255;
        int minH = 0;
        int maxS = 255;
        int minS = 0;
        int maxV = 255;
        int minV = 0;

        n.getParam("/vision/minH", minH);
        n.getParam("/vision/maxH", maxH);
        n.getParam("/vision/minS", minS);
        n.getParam("/vision/maxS", maxS);
        n.getParam("/vision/minV", minV);
        n.getParam("/vision/maxV", maxV);
    //  image.convertTo(frame,CV_8U,0.5,2);

        Mat hsv;

        cvtColor(image, hsv, COLOR_BGR2HSV);
        //cvtColor( image, hsv, CV_BGR2GRAY );
      //  GaussianBlur( hsv, hsv, Size( 17, 17 ), 17, 17 );
        erode( hsv, hsv, 55 );
        dilate( hsv, hsv, 55 );


        Mat1b mask;
        inRange(hsv, Scalar(minH, minS, minV), Scalar(maxH, maxS, maxV), mask);

     //   threshold( hsv, mask, minS,maxS,2);
   // threshold( hsv, mask, minH,maxH,1);

        cv::imshow( "meter window", mask );
        cv::waitKey(1);
      }

      if(false)
      {
        std_msgs::String msg;

        std::stringstream ss;
        ss << "hello world " << count;
        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str());

        chatter_pub.publish(msg);
        loop_rate.sleep();

      }

      //ROS_INFO("%x",  meterEnable);
      ros::spinOnce();
  }

  /*while (ros::ok())
  {
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    //ROS_INFO("%s", msg.data.c_str());

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }*/


  return 0;
}
 //RotatedRect r = minAreaRect( Mat(cont[i]));
            ///s  ROS_INFO("%f - %f - %f",r.size.height,r.size.width, r.angle);
    /*
              if(r.size.height>120&&r.size.height<200&&r.size.width>00&&r.size.width<8){
                    putText(frame, "mile", r.center,
                    FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200, 200, 250), 1, CV_AA);

            }
           /// &&frame.rows-r.center.y>50&&frame.cols-r.center.x>50
           ///////////   ROS_INFO("%f - %f - %f",r.size.height,r.size.width, r.angle);
              if(r.size.width>100&&r.size.width<180&&r.size.height>0&&r.size.width<8){
                    putText(frame, "mile", r.center,
                    FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200, 200, 250), 1, CV_AA);

            }*/
		//drawContours(frame, cont, i, Scalar(255, 0, 0), CV_FILLED);
		/*
		if (i != cont.size() - 1&&false) {
			for (int j = 0; j < cont.size() - 1; j++) {
				vector<Point> cnt1 = cont[i];
				vector<Point> cnt2 = cont[i + 1];
				x = x + 1;
				bool dist = find_if_close(cnt1, cnt2);
				if (x<cont.size()) {
					if (dist == true) {

						double val = min(status[i], status[x]);
						status[x] = status[i] = val;
					}
					else {
						if (status[x] == status[i]) {
							status[x] = i + 1;
						}
					}
				}
			}
		}
	}

	int maximum = 0;
	for (int m = 0; m < status.size(); m++) {
		if (status[m] > maximum)
			maximum = status[m];
	}
	vector<Point> contours01;
	maximum += 1;
	vector<Point>hull;
	vector<vector<Point> > contours_qq;
	vector<vector<Point> > unified1;
	int step = 0;
	for (int b = 0; b < maximum; b++) {
		vector <int> pos;

		vector<vector<Point> > unified;
		for (int j = 0; j < status.size(); j++)
		{

			if (status[j] == b)
			{

				//	approxPolyDP(cv::Mat(cont[j]), , 10, true);
				//contours01 = cont[j];

				unified.push_back(cont[j]);
				//pos.push_back(j);

			}

			else {
				if (!unified.empty()) {
					vector<Point> merge;
					vector<Point> merge1;
					vector<Point> merge2;
					for (size_t pb = 0; pb<unified.size() - 1; pb++) {
						merge1 = unified[pb];
						merge2 = unified[pb + 1];
						merge.reserve(unified[pb].size() + unified[pb + 1].size()); // preallocate memory
						merge.insert(merge.end(), unified[pb].begin(), unified[pb].end());
						merge.insert(merge.end(), unified[pb + 1].begin(), unified[pb + 1].end());
						unified[pb + 1] = merge;
					}
					if (!merge.empty()) {
						//approxPolyDP(merge, contours01, arcLength(Mat(cont[j]), true)*5, true);
						//cv::convexHull(merge, hull,true);


					}

					unified.clear();
				}

			}



		}

		//
	}

	for (size_t zu = 0; zu<unified1.size(); zu++)
		drawContours(frame, unified1, zu, Scalar(255, 0, 0), CV_FILLED);


	//GaussianBlur(gray, gray, Size(1, 1), 1, 1);
*/
