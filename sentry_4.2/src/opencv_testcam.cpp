/***************************************************************************
*  Copyright (C), 2018, Lesley
*  File name : opencv_testcam.cpp
*  Author : Lesley
*  Description : Project entry.
		 Gets the image using camera node of ROS.
		 Compute the coodrinate(x, y) and depth of target.
		 Compute the final coordinate according to ballistic curve.
		 Send the final coordinate to PLC using serial node of ROS.
***************************************************************************/
#include <math.h>
#include <time.h>
#include <iostream>
#include <queue>
#include <vector>
#include <sys/time.h>

#include "kalman.h"
#include "track_armor.h"
#include "adjust_color.h"
#include "contours.h"
#include "fittingPre.h"
#include "angle_solver.h"
//#include "MGM_Pre.h"
#include <ros/ros.h>
#include<image_transport/image_transport.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/image_encodings.h>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>
//#include</home/xjturm/acatkin_guard/src/serial_common/include/serial_common/Guard.h>
#include </home/xjturm/acatkin_guard/devel/include/serial_common/Guard.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

static const std::string OPENCV_WINDOW = "Image window";

using namespace std;
using namespace cv;

cv::VideoWriter writer;

#define M_PI 3.14159265358979323846
cv_bridge::CvImagePtr cv_ptr;

int two_threshval = 100;//105;
int two_number_threshval = 150;//185;//195;//120;
Mat element = getStructuringElement(MORPH_RECT, Size(1, 5));
struct timeval tv, tv1;
Rect boundbox;

bool send_flag;

bool next_process;
bool next;
bool priority = false;

Point Center(0,0);
bool distance_flag;
//int num = 0;
//int lastdepth = 0;
//vector<double> last_finaly5;
//bool priority = false;
int frame = 0;
int numf = 0;
//const int winHeight = cv_ptr->image.rows;
//const int winWidth = cv_ptr->image.cols;
serial_common::Guard result2;
string outFile1; 
string outFile; 

int main_last() {
{
	gettimeofday(&tv,NULL);
	  RGBpic = cv_ptr->image;
		
		if(RGBpic.empty())
		{
          return 0;
    }

		cvtColor(RGBpic, hsv, COLOR_RGB2HSV);
		cvtColor(RGBpic, gray, CV_BGR2GRAY);

		two = (gray >= two_threshval);
				morphologyEx(two, two, MORPH_OPEN, element);
				morphologyEx(two, two, MORPH_CLOSE, element);//闭操作 (连接一些连通域)
		//imshow("two",two);

		gray.copyTo(gray_number);
		equalizeHist(gray_number, gray_number);//增强对比度->直方图均衡化
		gamma_correct(gray_number, gray_number, 1);
		//imshow("gray_number",gray_number);
				two_number = (gray_number >= two_number_threshval);
		//imshow("two_number",two_number);

		/*two.copyTo(twocopy);
		full_small_contours(two, twocopy);//在twocopy中删去不符合要求的亮点
		imshow("twocopy", twocopy);*/
		full_small_contours(two, two);
		//imshow("two2",two);

				next_process = false;
				next = true;
		if(!priority)
		{
			{
				//定义轮廓和层次结构
				vector<vector<Point> > contours;
				vector<Vec4i> hierarchy;

				//findContours(twocopy, contours, hierarchy, CV_RETR_LIST, CHAIN_APPROX_TC89_KCOS);
				findContours(two, contours, hierarchy, CV_RETR_LIST, CHAIN_APPROX_TC89_KCOS);

				vector<vector<Point> > hull(contours.size());//寻找凸包 --> 多个点可能组成一片区域
				for (unsigned int i = 0; i<contours.size(); i++) {
						convexHull(Mat(contours[i]), hull[i], false);
				}

				//对每一个凸包均均求出其面积等信息，对于那些不符合光柱要求的凸包将其所有信息设置为0
				vector<vector<double> > kbarray(hull.size(),vector<double>(8));//信息存在kbarray中
				save_hull_info(kbarray, hull, contours);
				vector<point_flag> point_flag_list;

				int hull_num = hull.size();
				if(hull_num >= 2)
				{
						search_rect(hull, point_flag_list, hull_num, kbarray);
						if(!point_flag_list.empty())
						{
								distance_flag = sort_list(point_flag_list, kbarray, Center, hull, boundbox);
								if(!distance_flag) //检测到，不用识别
								{
									 priority = true;
									 track_num = 0;//一旦重置，下次追踪又可以追最多连续5帧
								}
								else
								{
									 next_process = true;
								}
						}
						else
						{
														next_process = true;
						}
				}
				else
				{
										next_process = true;
				}
		}

		if(next_process)
		{
				Center = Point(0,0);
				rect_2d[0] = Point2f(0,0);
				rect_2d[1] = Point2f(0,0);
				rect_2d[2] = Point2f(0,0);
				rect_2d[3] = Point2f(0,0);

				if(armor_categray_num[0] == 0)
					armor_categray_num[1]++;
				else
				{
					armor_categray_num[0]=0;
					armor_categray_num[1]=0;
				}

		}
		next = false;
		}
		else if(next)
		{
			bool track = false;
			if(px!=0 && py!=0)
			{
								Mat temp = formalGrayPic(boundbox);
								track = template_track_roi(temp, gray_number, Center, boundbox);
								//circle(RGBpic, Center, 4, Scalar(0, 255, 255), -1);
			}
			else
			{
								track_num = 0;
			}
			
			if(!track)
			{
				armor_categray_num[1]++;
			}

			if(px==0 || py==0 || track)//if(y_length_last == 0 || small_formal_flag == true || (y_length_last > 300 && x_dis_last > 400))//上一帧没有目标或上一帧小范围内没有目标
			{
				next_process = false;
				priority = false;

				vector<vector<Point> > contours;
				vector<Vec4i> hierarchy;
				//findContours(twocopy, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_TC89_KCOS);
				findContours(two, contours, hierarchy, CV_RETR_LIST, CHAIN_APPROX_TC89_KCOS);

				vector<vector<Point> > hull(contours.size());//寻找凸包 --> 多个点可能组成一片区域
				for (unsigned int i = 0; i<contours.size(); i++) {
						convexHull(Mat(contours[i]), hull[i], false);
				}

				//对每一个凸包均均求出其面积等信息，对于那些不符合光柱要求的凸包将其所有信息设置为0
				vector<vector<double> > kbarray(hull.size(),vector<double>(8));//信息存在kbarray中
				save_hull_info(kbarray, hull, contours);

				vector<point_flag> point_flag_list;

				int hull_num = hull.size();
				if(hull_num >= 2)
				{
						search_rect(hull, point_flag_list, hull_num, kbarray);
						if(!point_flag_list.empty())
						{
								distance_flag = sort_list(point_flag_list, kbarray, Center, hull, boundbox);
								if(!distance_flag) //检测到，不用识别
								{
									 priority = true;
									 track_num = 0;//一旦重置，下次追踪又可以追最多连续5帧
								}
								else
								{
									 next_process = true;
								}
						}
						else{
								next_process = true;
						}
				}
				else{
						next_process = true;
				}
		}

		if(next_process)
		{
				Center = Point(0,0);
				rect_2d[0] = Point2f(0,0);
				rect_2d[1] = Point2f(0,0);
				rect_2d[2] = Point2f(0,0);
				rect_2d[3] = Point2f(0,0);
				if(armor_categray_num[0] == 0)
					armor_categray_num[1]++;
				else
				{
					armor_categray_num[0]=0;
					armor_categray_num[1]=0;
				}

		}
		//circle(RGBpic, Center, 4, Scalar(0, 255, 255), -1);
		}

		px = Center.x;
        	py = Center.y;
		//!---Kalman Filtering---------------------------------------------------------
		//if(px!=0 || py!=0)
		//{
				//if(pow((pow((px_last-px),2)+pow((py_last-py),2)),0.5) > 70)
							//Kalman_init(Center);
				//Kalman_prediction( RGBpic, Center);
		//}
		//!----------------------------------------------------------------------------
		//!********************拟合预测*******************************************
		/*double distance =pow((pow((px_last-px),2)+pow((py_last-py),2)),0.5);
		if(Px_y_last_five.size() < 5)
		{
					Px_y_last_five.push_back(Point(px,py));
		}
		else
		{
					Px_y_last_five.erase(Px_y_last_five.begin());
					Px_y_last_five.push_back(Point(px,py));
		}
		if((px == 0 && py == 0) || distance > 40 || distance < 1.5)
		{
			    Px_y_last_five.clear();
		}
		//cout<<Px_y_last_five<<endl;
		if(Px_y_last_five.size() == 5)
		{
					double arry1[5]={Px_y_last_five[0].x,Px_y_last_five[1].x,Px_y_last_five[2].x,Px_y_last_five[3].x,Px_y_last_five[4].x};
					//double arry2[5]={Px_y_last_five[0].y,Px_y_last_five[1].y,Px_y_last_five[2].y,Px_y_last_five[3].y,Px_y_last_five[4].y};
					//MGMpre mgm;
					//mgm.prediction();
					fitting_Pre(arry1,px_next);
					//fitting_Pre(arry2,py_next);
					
			}*/
			//!**************************************************************************************/
	//}
		/*else{
           		 px_next = Center.x;
      		  }
	circle(RGBpic, Point(px_next, py), 8, Scalar(0, 0, 255), 2);*/
		formalGrayPic = gray_number.clone();
		cout<<"armor_categray_num"<<armor_categray_num[0]<<endl;
		cout<<"armor_categray_num_number"<<armor_categray_num[1]<<endl;

				cout<<"final x y"<<px<<"  "<<py<<endl;
 px_last = px;
        py_last = py;
				if(rect_2d[0]!=Point2f(0,0))
				{
						/*double dy=0;
						if(length_last > 85)
								dy = (length_last - 85) * 0.03;
						else
								dy = (length_last - 85) * 0.06;
						cout<<"dy"<<dy<<endl;

						rect_2d[0].y -= dy;
						rect_2d[1].y += dy;
						rect_2d[2].y += dy;
						rect_2d[3].y -= dy;*/
						angle_solver_test(rect_2d, RGBpic,depth,armor_categray_num[0]);
				}

cout<<"depth: "<<depth<<endl;
		double x1 = px;//_next;
		double y1 = py - (-0.158*depth +0.000275*depth*depth + 100);

		//if(y1 > 300)
		//	y1 -= 0.1 * depth;
		//if(y1 >300)
			//y1 -= 0.05 * depth;
		Point Centerfinal;
		Centerfinal.x = x1;
		Centerfinal.y = y1;

		circle(RGBpic, Centerfinal, 4, Scalar(255, 100, 155), -1);

		cout<<"final x y"<<px<<"  "<<py<<endl;

		circle(RGBpic, Center, 4, Scalar(255, 0, 255), -1);

		//if(armor_categray_num[0] == 1)
			//last_small = finalxx;
				//else
						//last_small = Point(0, 0);

				//Size asize = Size(RGBpic.cols*0.3, RGBpic.rows*0.3);
				//resize(RGBpic, RGBpic, asize);
				//imshow("last", RGBpic);
				//outfile<<depth<<endl;
				//waitKey(200);
		/*
				if(distance_flag && !small_formal_flag)
				{
						int yy;
						cin>>yy;
						if(yy == 1)
						{
								double my_real;
								cin>>my_real;
								outfile<<point_flag_last.angle_sum<<" "<<point_flag_last.x_dis_current<<" "<<point_flag_last.area_current<<" "<<point_flag_last.point_a_center<<" "<<my_real<<endl;

						}
				}
		*/
				gettimeofday(&tv1,NULL);
				cout<<"time:"<<( tv1.tv_usec -  tv.tv_usec)<<" us"<<endl;

			//cout<<endl<<endl<<endl;
			//lastdepth = depth;

			result2.xlocation = x1 - winWidth/2;
			result2.ylocation = y1 - winHeight/2;
			result2.depth = 3;//int(depth);

			if (result2.xlocation > -320 || depth < 480)
				 send_flag = 1 ;
			//if(depth>580 || lastdepth > 580)
				//result2.xlocation = -320;

			ROS_INFO_STREAM("OriginalData_x_next:"<<(int16_t)result2.xlocation);
			ROS_INFO_STREAM("OriginalData_y_next:"<<(int16_t)result2.ylocation);
			ROS_INFO_STREAM("OriginalData_depth:"<<(int16_t)result2.depth);
			cout<<endl;
			//waitKey(30);
	}
	//cout << "dsf" << endl;
	//outfile.close();
	//outkalman.close();
	//return 0;
}

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
	ros::Publisher guard_sub = nh_.advertise<serial_common::Guard>("write", 20);
public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1,
      &ImageConverter::imageCb, this);
    //image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {

    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

		
    
	
	if(frame == 0)
	{
		stringstream s;
		s << numf;
		string number = s.str();
		outFile = outFile1 + "-" + number + ".avi";
		numf ++;cout<<numf<<endl;
		writer.open(outFile, CV_FOURCC('M', 'J', 'P', 'G'), 120, Size(640, 480));
		
	if(!writer.isOpened())
  {
  cout<< "Error : fail to open video writer\n"<<endl;
}
	}
	
	writer << cv_ptr->image;
	main_last();
	frame ++;
	if(frame == 8000)
	{
		frame = 0;
        }
    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(1);

		if(send_flag)
		{
			guard_sub.publish(result2);
			send_flag = 0;
		}
  }
};

int main(int argc, char** argv)
{
	struct timeval tv;
gettimeofday(&tv,NULL);
int mmm = tv.tv_sec*1000000 + tv.tv_usec;
	string buf;
	time_t now = time(0);
	tm *ltm = localtime(&now);

	stringstream t11, t12, t13, t14, t15;
	t11<<ltm -> tm_mday;
	t12<<ltm -> tm_hour;
	t13<<ltm -> tm_min;
  t14<<ltm -> tm_sec;
	string t1 = t11.str();
	string t2 = t12.str();
	string t3 = t13.str();
	string t4 = t14.str();
for(int i=0; i<1000; i++)
	cout<<"ready"<<endl;
	gettimeofday(&tv,NULL);
	int mye = tv.tv_sec*1000000 + tv.tv_usec;
	t15<< mye - mmm;
	string t5 = t15.str();

	outFile1 = "mainvideo/" + t1 +"-" + t2 +"-"+ t3 +"-"+ t4 +"s_"+ t5;// + ".avi";
	


	//Point initpoint = Point(0,0);
	//Kalman_init(initpoint);
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
