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
#include</home/xjturm01/final_catkin/devel/include/serial_common/Guard.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>


static const std::string OPENCV_WINDOW = "Image window";

using namespace std;
using namespace cv;
struct timeval tv;

cv::VideoWriter writer;

#define M_PI 3.14159265358979323846
cv_bridge::CvImagePtr cv_ptr;

Point Center(0,0);
Mat formalRGBpic; //存取前一张图片
Scalar color(255, 0, 0);
Scalar colorhull(0, 255, 0);
float ellthre = 0.9;//其是否是椭圆
float ellareathre = 1; //其面积是否过小
bool distance_flag;
int num = 0;
int lastdepth = 0;
vector<double> last_finaly5;
bool priority = false;
int frame = 0;

serial_common::Guard result2;
int main_last() {
	  const int winHeight = cv_ptr->image.rows;
    const int winWidth = cv_ptr->image.cols;
{
	  RGBpic = cv_ptr->image;

		struct timeval tv;
		gettimeofday(&tv,NULL);
		int mmm = tv.tv_sec*1000000 + tv.tv_usec;

		if(RGBpic.empty())
		{
          return 0;
    }

		//Mat gray;
		cvtColor(RGBpic, gray, CV_BGR2GRAY);
		//Mat channels[3];
		//split(RGBpic, channels);
		//Mat copygray;
		//gray = channels[2];
		gray.copyTo(copygray);

		//define_filter_contrast_ratio(gray);

		//Mat gray_number;
		//gray.copyTo(gray_number);
		//equalizeHist(gray_number, gray_number);//增强对比度->直方图均衡化
        //imshow("sad",gray_number);


		//gamma_correct(gray, gray, 2.0);

		//Mat Structure1 = getStructuringElement(MORPH_RECT, Size(1, 1));
		//Mat Structure1 = getStructuringElement(MORPH_RECT, Size(0, 8));
		//dilate(gray, gray, Structure1, Point(-1, -1));//高亮部分膨胀
		//imshow("gray", gray);

		//二值化
		int threshval = 23;//35 ;//200;
		two = (gray >= threshval);

	  //imshow("two", two);

        //!在twocopy中删去过小的亮点
		twocopy;
		two.copyTo(twocopy);
		full_small_contours(two, twocopy, winHeight, winWidth);

        //gamma_correct(twocopy, twocopy, 3.0);
		Mat Structure2 = getStructuringElement(MORPH_RECT, Size(2, 5));
		dilate(twocopy, twocopy, Structure2, Point(-1, -1));//高亮部分膨胀

		/*Rect searchWindow;//模板匹配的矩形框大小
		Point templcenter;//记录的是模板中心距左上角点(x,y)的dx和dy距离
		Point forCenter = Point(0,0);//裁剪图片左上角相对于原图左上角的位置
		int height, width;
		double forScale;
		if(y_length_last>150)
				forScale = forbigScale;
		else
				forScale = forsmallScale;
		height = min(int(x_dis_last * forScale), winHeight);
		width = min(int(y_length_last * forScale), winWidth);

		find_rect_template2(searchWindow, templcenter, winHeight, winWidth, height, width);
		forCenter.x = searchWindow.x;
		forCenter.y = searchWindow.y;*/

		bool small_formal_flag = false;//false->small   true->big
		Point forCenterb = Point(0,0);
		bool next = true;
		if(!priority)
		{
			{
				//定义轮廓和层次结构
				vector<vector<Point> > contours;
				vector<Vec4i> hierarchy;
				//findContours(twocopy, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_TC89_KCOS);
				findContours(twocopy, contours, hierarchy, CV_RETR_LIST, CHAIN_APPROX_TC89_KCOS);

				vector<vector<Point> > hull(contours.size());//寻找凸包 --> 多个点可能组成一片区域
				for (unsigned int i = 0; i<contours.size(); i++) {
						convexHull(Mat(contours[i]), hull[i], false);
				}

				//对每一个凸包均均求出其面积等信息，对于那些不符合光柱要求的凸包将其所有信息设置为0
				vector<vector<double> > kbarray(hull.size(),vector<double>(8));//信息存在kbarray中
				save_hull_info(forCenterb, kbarray, hull, contours);

				vector<point_flag> point_flag_list;

				int hull_num = hull.size();
				if(hull_num >= 2)
				{
						search_rect(forCenterb, hull, point_flag_list, hull_num, kbarray,winHeight, winWidth);
						if(!point_flag_list.empty())
						{
								distance_flag = sort_list(point_flag_list, kbarray, Center, hull);
								if(!distance_flag) //检测到，不用识别
								{
									 priority = true;
									 track_num = 0;//一旦重置，下次追踪又可以追最多连续5帧
								}
								else
								{
								   small_formal_flag = true;
								}
						}
						else{
								small_formal_flag = true;
						}
				}
				else
				{
						small_formal_flag = true;
				}
		}

		/*if(small_formal_flag && px!=0 && py!=0)
		{
				//Mat copygray_roi = copygray;//copygray(searchWindow);
				//imshow("tets", copygray_roi);
				//small_formal_flag = template_track2(formalRGBpic, copygray_roi, forCenter, winHeight, winWidth, searchWindow.height, searchWindow.width, Center);
				//small_formal_flag = template_track2(formalRGBpic, copygray, forCenterb, winHeight, winWidth, winHeight, winWidth, Center);
				small_formal_flag = template_track(formalRGBpic, copygray,  winHeight,  winWidth, Center);
		}*/

		if(small_formal_flag)
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
		//circle(RGBpic, Center, 4, Scalar(255, 0, 0), -1);
	}
	else if(next)
	{
			bool track = false;
			//cout<<"priority"<<priority<<endl;
			if(px!=0 && py!=0)
			{
					//Mat copygray_roi = copygray(searchWindow);
					//track = template_track2(formalRGBpic, copygray_roi, forCenter, winHeight, winWidth, searchWindow.height, searchWindow.width, Center);
					//track = template_track2(formalRGBpic, copygray, forCenterb, winHeight, winWidth, winHeight, winWidth, Center);
					track = template_track(formalRGBpic, copygray,  winHeight,  winWidth, Center);
					//circle(RGBpic, Center, 4, Scalar(0, 255, 255), -1);
			}

			if(px==0 || py==0 || track)//if(y_length_last == 0 || small_formal_flag == true || (y_length_last > 300 && x_dis_last > 400))//上一帧没有目标或上一帧小范围内没有目标
			{
				small_formal_flag = false;
				priority = false;
				//定义轮廓和层次结构
				vector<vector<Point> > contours;
				vector<Vec4i> hierarchy;
				//findContours(twocopy, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_TC89_KCOS);
				findContours(twocopy, contours, hierarchy, CV_RETR_LIST, CHAIN_APPROX_TC89_KCOS);

				vector<vector<Point> > hull(contours.size());//寻找凸包 --> 多个点可能组成一片区域
				for (unsigned int i = 0; i<contours.size(); i++) {
						convexHull(Mat(contours[i]), hull[i], false);
				}

				//对每一个凸包均均求出其面积等信息，对于那些不符合光柱要求的凸包将其所有信息设置为0
				vector<vector<double> > kbarray(hull.size(),vector<double>(8));//信息存在kbarray中
				save_hull_info(forCenterb, kbarray, hull, contours );

				vector<point_flag> point_flag_list;

				int hull_num = hull.size();
				if(hull_num >= 2)
				{
						search_rect(forCenterb, hull, point_flag_list, hull_num, kbarray,winHeight, winWidth);
						if(!point_flag_list.empty())
						{
								distance_flag = sort_list(point_flag_list, kbarray, Center, hull);
								if(!distance_flag) //检测到，不用识别
								{
									 priority = true;
									 track_num = 0;//一旦重置，下次追踪又可以追最多连续5帧
								}
								else
								{
								   small_formal_flag = true;
								}
						}
						else{
								small_formal_flag = true;
						}
				}
				else{
						small_formal_flag = true;
				}
		}

		if(small_formal_flag)
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

				//!---Kalman Filtering---------------------------------------------------------
				/*if(px!=0 || py!=0)
				{
						if(pow((pow((Center.x-px),2)+pow((Center.y-py),2)),0.5) > 50)
								Kalman_init(Center);
						Kalman_prediction(KF, RGBpic, Center, winWidth, winHeight);
				}*/
				//!----------------------------------------------------------------------------
				//!********************拟合预测*******************************************

				//!**************************************************************************************/
		//}
		copygray.copyTo(formalRGBpic);
		cout<<"armor_categray_num"<<armor_categray_num[0]<<endl;

		//double depth = 0;
		//depth =1452.5455 * exp(-0.1002 * y_length_last) + 1.1814 * y_length_last;
		//cout<<"depth"<<depth<<endl;

		if(rect_2d[0]!=Point2f(0,0))
		{
			double dy=0;
			if(length_last > 85)
					dy = (length_last - 85) * 0.03;
			else
					dy = (length_last - 85) * 0.06;
			cout<<"dy"<<dy<<endl;

			rect_2d[0].y -= dy;
			rect_2d[1].y += dy;
			rect_2d[2].y += dy;
			rect_2d[3].y -= dy;
				angle_solver_test(rect_2d, RGBpic,depth,armor_categray_num[0]);

			}
		//circle(RGBpic, Center, 4, Scalar(255, 0, 255), -1);
		double x1 = Center.x ;
		double y1 = Center.y - 0.047*depth - 22.5429;
		if(depth > 340 )
			y1 = y1 - 0.005*depth - 8;
		if(depth >= 400)
			y1 = y1 - 7;

		if(Px_y_last_five.size() < 5)
		{
				Px_y_last_five.push_back(Point(x1, y1));
		}
		else
		{
				Px_y_last_five.erase(Px_y_last_five.begin());
				Px_y_last_five.push_back(Point(x1, y1));
		}
		if((px == 0 && py == 0) || (pow((pow((Center.x-px),2)+pow((Center.y-py),2)),0.5) > 10))
		{
				Px_y_last_five.clear();
		}
		//cout<<Px_y_last_five<<endl;
		int px_next = x1;
		if(Px_y_last_five.size() == 5)
		{
				double arry1[5]={Px_y_last_five[0].x,Px_y_last_five[1].x,Px_y_last_five[2].x,Px_y_last_five[3].x,Px_y_last_five[4].x};
					//double arry2[5]={Px_y_last_five[0].y,Px_y_last_five[1].y,Px_y_last_five[2].y,Px_y_last_five[3].y,Px_y_last_five[4].y};
				//MGMpre mgm;
				//mgm.prediction();
				//,py_next;
				fitting_Pre(arry1,px_next);
				//fitting_Pre(arry2,py_next);

				circle(RGBpic, Point(px_next, y1), 8, Scalar(0, 0, 255), 2);
		}

		px = Center.x;
		py = Center.y;
		cout<<"final x y"<<px<<"  "<<py<<endl;

		Point finalxx(0,0);
		finalxx.x = x1;
		finalxx.y = y1;
		cout<<x1<<"   "<<y1<<endl;
		circle(RGBpic, finalxx, 4, Scalar(255, 0, 255), -1);

		if(armor_categray_num[0] == 1)
			last_small = finalxx;


		/*double sum = 0;
		for(int i=0; i<last_finaly5.size(); i++)
		{
				sum += last_finaly5[i];
		}
		sum = sum/last_finaly5.size();

		if(last_finaly5.size()<3)
			last_finaly5.push_back(y1);
		else
		{
				last_finaly5.erase(last_finaly5.begin());
				last_finaly5.push_back(y1);
		}

		if(min(y1, sum)/max(y1, sum)<0.7 && last_finaly5.size()!=0)
			y1 = sum;*/

			gettimeofday(&tv,NULL);
			int mye = tv.tv_sec*1000000 + tv.tv_usec;
			cout<<"time:"<<(mye - mmm)<<" us"<<endl;
			cout<<endl<<endl<<endl;
			lastdepth = depth;

			result2.xlocation = px_next - winWidth/2;
			result2.ylocation = y1 - winHeight/2;
			result2.depth = depth;//int(depth);
			if(depth>580 || lastdepth > 580)
				result2.xlocation = -320;

			ROS_INFO_STREAM("OriginalData_x_next:"<<(int16_t)result2.xlocation);
			ROS_INFO_STREAM("OriginalData_y_next:"<<(int16_t)result2.ylocation);
			ROS_INFO_STREAM("OriginalData_depth:"<<(int16_t)result2.depth);
			cout<<endl;
			//waitKey(30);
/*
Mat finalpic;
Mat cameraMatrix = (Mat_<double>(3,3) << 799.4882112289697, 0, 335.7201090683797, 0, 798.2745396421427, 244.4640310157802, 0, 0, 1);
Mat distCoeffs = (Mat_<double>(1,5) <<-0.4304634395248472, 0.4652451609572892, -0.002170757080254447, -0.001416479077749636, -0.7229891661708902);
undistort( RGBpic, finalpic, cameraMatrix, distCoeffs, noArray() );
imshow("final picture",finalpic);*/

/*
		fitting_list.push_back(Center);
		if(fitting_list.size() == 31)
        {
            fitting_list.erase(fitting_list.begin());
        }
*/
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
  image_transport::Publisher image_pub_;
	ros::Publisher guard_sub = nh_.advertise<serial_common::Guard>("write", 20);
public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

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

		writer << cv_ptr->image;
    main_last();
    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(1);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
		guard_sub.publish(result2);
  }
};

int main(int argc, char** argv)
{struct timeval tv;
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

	string outFile = t1 + t2 + t3 + t4 +"_"+ t5 + ".avi";
	writer.open(outFile, CV_FOURCC('M', 'J', 'P', 'G'), 120, Size(640, 480));

	if(!writer.isOpened())
  {
  cout<< "Error : fail to open video writer\n"<<endl;
  //return -1;
  }
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
