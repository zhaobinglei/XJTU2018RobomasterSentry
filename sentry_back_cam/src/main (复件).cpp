#include <iostream>
#include "contours.h"
#include <stdio.h>
#include <sys/time.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
//#include</home/xjturm/acatkin_guard/src/serial_common/include/serial_common/Guard.h>
#include </home/xjturm/acatkin_guard/devel/include/serial_common/Guard.h>
using namespace std;
using namespace cv;

serial_common::Guard result1;
serial_common::Guard result2;
bool img_flag1=0, img_flag2=0;
bool send_flag1 = 0, send_flag2 = 0;
cv_bridge::CvImagePtr cv_ptr1;
cv_bridge::CvImagePtr cv_ptr2;
int two_threshval = 23;
//static const std::string OPENCV_WINDOW = "Image window";
//static const std::string OPENCV_WINDOW = "Image window";
cv::VideoWriter writer1;
int autoshot(
            Mat& rgbPic,
            Mat& formalPic,
            int& pX,
            int& pY,
            double& yLengthLast,
            double& distanceLast,
            vector<double>& distanceLastArray,
            int* armorCategrayNum
            );

int main_last()
{
    //ros::init(argc, argv, "image_converter_back");
    //ros::NodeHandle nh_;
    //ros::Publisher guard_sub = nh_.advertise<serial_common::Guard>("write", 20);
    //VideoCapture capture1(1);//"video/ceshi/1.mkv");//video/reshen/1.avi
    //VideoCapture capture2(2);//"video/ceshi/1.mkv");//video/reshen/1.avi
	  //if(!capture1.isOpened() || !capture2.isOpened())
    //{
    //   return -1;
    //}
    Mat RGBpic1, formal_pic1;//存取前一张图片
    if(img_flag1)
    	RGBpic1 = cv_ptr1->image;
    int px1=0, py1=0;
    double y_length_last1 = 0, distance_last1 = 0;
    vector <double> distance_last_array1;
    int armor_categray_num1[2] = {0,0}; //1-->small; 2-->big


    Mat RGBpic2, formal_pic2;//存取前一张图片
    if(img_flag2)
    	 RGBpic2 = cv_ptr2->image;
    int px2=0, py2=0;
    double y_length_last2 = 0, distance_last2 = 0;
    vector <double> distance_last_array2;
    int armor_categray_num2[2] = {0,0}; //1-->small; 2-->big


    //const int frame = (int)capture1.get(CV_CAP_PROP_FPS);
    //cout<<winHeight<<"  "<<winWidth<<"  "<<frame<<endl;

	//while (true)
	{
        //capture1 >> RGBpic1;
        //capture2 >> RGBpic2;


	if(!RGBpic1.empty())
  {
      autoshot(RGBpic1, formal_pic1, px1, py1, y_length_last1,
                distance_last1, distance_last_array1, armor_categray_num1);

      cout<<"final x1 y1:"<<px1<<"  "<<py1<<endl;
      imshow("last1", RGBpic1);
	}

	if(!RGBpic2.empty())
  {

      autoshot(RGBpic2, formal_pic2,px2, py2, y_length_last2,
                distance_last2, distance_last_array2, armor_categray_num2);

      cout<<"final x2 y2:"<<px2<<"  "<<py2<<endl;
      //imshow("last2", RGBpic2);
	}

	if(img_flag1)
	{
    if(px1 == 0 )
			result1.xlocation = 0;
		else if(px1 > 0 && px1 < 210)
			result1.xlocation = 1;
		else if(px1 >= 210 && px1 < 430)
			result1.xlocation = 2;
		else if(px1 >= 430 && px1 < 640)
			result1.xlocation = 3;

		result1.ylocation = -1;
    result1.depth = 1;
    if(result1.xlocation != 0 )//&& armor_categray_num1[1]>=2)
       send_flag1 = 1;
	}

	if(img_flag2)
	{
    if(px2 == 0 )
			result2.xlocation = 0;
		else if(px2 > 0 && px2 < 210)
			result2.xlocation = 1;
		else if(px2 >= 210 && px2 < 430)
			result2.xlocation = 2;
		else if(px2 >= 430 && px2 < 640)
			result2.xlocation = 3;
		result2.ylocation = -1;
    result2.depth = 2;
    if(result2.xlocation != 0)// && armor_categray_num2[1]>=2)
       send_flag2 = 1;
	}

        //waitKey(1);
	}
}



class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_1;
  image_transport::Subscriber image_sub_2;
  //image_transport::Publisher image_pub_;
	ros::Publisher guard_sub = nh_.advertise<serial_common::Guard>("write", 20);
public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_1 = it_.subscribe("/usb_cam1/image_raw1", 1,
      &ImageConverter::imageCb1, this);
    image_sub_2 = it_.subscribe("/usb_cam2/image_raw2", 1,
      &ImageConverter::imageCb2, this);
    //image_pub_ = it_.advertise("/image_converter/output_video", 1);

    //cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    //cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb1(const sensor_msgs::ImageConstPtr& msg)
  {

    try
    {
      cv_ptr1 = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    writer1 << cv_ptr1->image;
    img_flag1 = 1;
    main_last();
    img_flag1 = 0;
    // Update GUI Window
    //cv::imshow(OPENCV_WINDOW, cv_ptr1->image);
    cv::waitKey(1);

    // Output modified video stream
    //image_pub_.publish(cv_ptr->toImageMsg());
    if(send_flag1 == 1)
    {
      guard_sub.publish(result1);
      send_flag1 = 0;
    }
  }

void imageCb2(const sensor_msgs::ImageConstPtr& msg)
  {

    try
    {
      cv_ptr2 = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    img_flag2 = 1;
    main_last();
    img_flag2 = 0;

    // Update GUI Window
    //cv::imshow(OPENCV_WINDOW, cv_ptr2->image);
    cv::waitKey(1);

    // Output modified video stream
    //image_pub_.publish(cv_ptr->toImageMsg());
    if(send_flag2 == 1)
    {
      guard_sub.publish(result2);
      send_flag2 = 0;
    }
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


	gettimeofday(&tv,NULL);
	int mye = tv.tv_sec*1000000 + tv.tv_usec;
	t15<< mye - mmm;
	string t5 = t15.str();

	string outFile1 = "leftvideo/"+t1 + t2 + t3 + t4 +"_"+ t5 + ".avi";
	writer1.open(outFile1, CV_FOURCC('M', 'J', 'P', 'G'), 120, Size(640, 480));

	if(!writer1.isOpened() )
  {
  cout<< "Error : fail to open video writer\n"<<endl;
  return -1;
  }
  ros::init(argc, argv, "image_converter_back");
  ImageConverter ic;

  ros::spin();
  return 0;
}



int autoshot(
            Mat& rgbPic,
            Mat& formalPic,
            int& pX,
            int& pY,
            double& yLengthLast,
            double& distanceLast,
            vector<double>& distanceLastArray,
            int* armorCategrayNum
            )
{

//waitKey(0);
    Mat gray, two, twocopy, copygray;

	cvtColor(rgbPic, gray, CV_BGR2GRAY);
	gray.copyTo(copygray);
	two = (gray >= two_threshval);

    //!在twocopy中删去过小的亮点
	two.copyTo(twocopy);
	fullSmallContours(&two, twocopy);

    Mat Structure1 = getStructuringElement(MORPH_RECT, Size(2, 5));
	dilate(twocopy, twocopy, Structure1, Point(-1, -1));//高亮部分膨胀
  Point picCenter;
    bool next_process = false;

	 {
        vector<vector<Point> > contours;
        vector<Vec4i> hierarchy;
        findContours(twocopy, contours, hierarchy, CV_RETR_LIST, CHAIN_APPROX_TC89_KCOS);
        vector<vector<Point> > hull(contours.size());//寻找凸包 --> 多个点可能组成一片区域
        for (int i = 0; i<contours.size(); i++)
        {
            convexHull(Mat(contours[i]), hull[i], false);
        }
        //对每一个凸包均均求出其面积等信息，对于那些不符合光柱要求的凸包将其所有信息设置为0
        vector<vector<double> > kbarray(hull.size(),vector<double>(8));//信息存在kbarray中
        saveHullInfo(rgbPic, kbarray, hull, contours);
        vector<point_flag> point_flag_list;

        int hull_num = hull.size();
        if(hull_num >= 2)
        {
            searchRect(rgbPic, hull, point_flag_list, hull_num, kbarray, pX, pY, distanceLast, armorCategrayNum);
            if(!point_flag_list.empty())
            {

                bool detectFlag = sort_list(point_flag_list, kbarray, picCenter, hull, yLengthLast,
                                        distanceLast, distanceLastArray, armorCategrayNum);

                if(detectFlag) //没有检测到
                    next_process = true;
            }
            else{
                next_process = true;
            }
        }
        else
            next_process = true;

        if(next_process)
        {
            picCenter = Point(0,0);

            if(armorCategrayNum[0] == 0)
                armorCategrayNum[1]++;
            else
            {
                armorCategrayNum[0]=0;
                armorCategrayNum[1]=0;
            }
        }
    }

    copygray.copyTo(formalPic);

    pX = picCenter.x;
    pY = picCenter.y;

    circle(rgbPic, picCenter, 4, Scalar(255, 0, 255), -1);
    //imshow("last"+str(), *rgbPic);
}
