/******************************************************************
*  Copyright (C), 2018, Lesley
*  File name : fittingPre.h
*  Author : Lesley
*  Description : If last image find the target, use this program to
		 track the target to keep the shooting target the 
		 same.
******************************************************************/
#ifndef ARMOR_H_INCLUDED
#define ARMOR_H_INCLUDED

#include<opencv2/opencv.hpp>
#include<math.h>
#include<time.h>
#include <opencv2/core.hpp>
#include <cxcore.h>
#include <highgui.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/videoio.hpp>
#include <iostream>
#include "contours.h"
using namespace std;
using namespace cv;
int height,width;

double template_thre = 0.79;
int track_num = 0;
double fScale = 1;//0.25;//0.8;//1;//0.8;//5;
int tempScaleW = 2, tempScaleH = 3;
int currScaleW = 4, currScaleH = 3;
//double forbigScale = 8;
//double forsmallScale = 10;

class Optical
{
private:
    Mat curgray;//当前图片
    Mat pregray;//先前图片
    vector<Point2f> point[2];//point[0]是特征点的原来位置，point[1]是特征点的新位置
    vector<Point2f> initpoint;//初始化跟踪点的位置
    vector<Point> features;//检测到的特征点
    int maxCounts;//检测的最大特征点数
    double qLevel;//检测特征的等级
    double minDist;//检测特征的最小距离
    vector<uchar> status;//跟踪特征的状态，发现特征的流为1，否则为0
    vector<float> err;//误差矢量
public:
    Optical(int max_Counts = 1, double q_Level = 0.01, double min_Dist = 10.0)
    {
        maxCounts = max_Counts;//检测的最大特征点数
        qLevel = q_Level;//检测特征的等级
        minDist = min_Dist;//检测特征的最小距离
    }
    bool addNewPoint()
    {
        return point[0].size() <= 10;
    }
    bool acceptTrackPoint(int i)
    {
        return status[i] && ((abs(point[0][i].x - point[1][i].x) + abs(point[0][i].y - point[1][i].y)) > 2);
    }
    void Optical_track(Mat& frame, Mat& result, Point& Center);
};

void Optical::Optical_track(Mat& frame, Mat& result, Point& Center)
{
    cvtColor(frame, curgray, COLOR_BGR2GRAY);
	//frame.copyTo(result);
	if (addNewPoint())
	{
		goodFeaturesToTrack(curgray, features, maxCounts, qLevel, minDist);//!图像强角点,放在features中
		features[0].x = Center.x;
		features[0].y = Center.y;
		features[1].x = Center.x+2;
		features[1].y = Center.y+2;
		features[2].x = Center.x-2;
		features[2].y = Center.y-2; //三个角点
		//把feature里的角点重头到尾存到point[0]的末尾中
		point[0].insert(point[0].end(), features.begin(), features.end());
		//把feature里的角点重头到尾存到initpoint的末尾中
		initpoint.insert(initpoint.end(), features.begin(), features.end());
	}
	if (pregray.empty())
		curgray.copyTo(pregray);
	//计算稀疏征集的光流
	calcOpticalFlowPyrLK(pregray, curgray, point[0], point[1], status, err);
	int k = 0;
	for (int i = 0; i < point[1].size(); i++){
		if (acceptTrackPoint(i)){
			initpoint[k] = initpoint[i];
			point[1][k++] = point[1][i];
		}
	}
	point[1].resize(k);//把point[1]中所含有的数量重新变成含有k个
	initpoint.resize(k);//把initpoint中所含有的数量重新变成含有k个
						//绘制角点和运动轨迹
    Point center;
    center.x=0,center.y=0;
    int optnum = 0;
	for (int i = 0; i < point[1].size(); i++)
	{

        if(fabs(point[1][i].x-Center.x)<50 && fabs(point[1][i].y-Center.y)<50){

            center.x += point[1][i].x;
            center.y += point[1][i].y;
            //line(result, initpoint[i], point[1][i], Scalar(0, 0, 255));
            //circle(result, point[1][i], 3, Scalar(0, 155, 155), -1);
            optnum ++;
        }
	}
	if(optnum != 0){
        center.x = center.x/optnum;
        center.y = center.y/optnum;
	}
	circle(result, center, 8, Scalar(0, 155, 155), -1);
	swap(point[1], point[0]);//把point[0]和point[1]的元素交换
	swap(pregray, curgray);//把两张图片交换
	Center.x = center.x;
	Center.y = center.y;
	px = Center.x;
    py = Center.y;
}


bool find_rect_template(Rect& searchWindow, Point& templcenter, double x_dis, double y_length)
{
    if( (px - x_dis/2 >= 0) && (px + x_dis/2 <= winWidth) &&
           (py - y_length/2 >= 0) && (py + y_length/2 <= winHeight) )
    {
        searchWindow.x = px - x_dis/2;
        templcenter.x = x_dis/2;
        searchWindow.width = x_dis;

        searchWindow.y = py - y_length/2;
        templcenter.y = y_length/2;
        searchWindow.height = y_length;
        return true;
    }
    else
        return false;

}

void find_rect_curr(Rect& searchWindow, Point& templcenter, double x_dis, double y_length)
{
    cout<<"px"<<px<<"py"<<py<<endl;
    if(px - x_dis/2 < 0)
    {
        searchWindow.x = 0;
        templcenter.x = px;
        searchWindow.width = x_dis/2 + px;
    }
    else if(px + x_dis/2 > winWidth)
    {
        searchWindow.x = px - x_dis/2;
        templcenter.x = x_dis/2;
        searchWindow.width = winWidth - px + x_dis/2;
    }
    else
    {
        searchWindow.x = px - x_dis/2;
        templcenter.x = x_dis/2;
        searchWindow.width = x_dis;
    }

    if(py - y_length/2 < 0)
    {
        searchWindow.y = 0;
        templcenter.y = py;
        searchWindow.height = y_length/2 + py;
    }
    else if(py + y_length/2 > winHeight)
    {
        searchWindow.y = py - y_length/2;
        templcenter.y = y_length/2;
        searchWindow.height = winHeight - py + y_length/2;
    }
    else
    {
        searchWindow.y = py - y_length/2;
        templcenter.y = y_length/2;
        searchWindow.height = y_length;
    }

}

bool template_track(Mat& formalimg, Mat& currentimg, Point& Center)
{
    if ((track_num <= 100 && armor_categray_num[0] != 2) || (track_num <= 50 && armor_categray_num[0] == 2))
    {
        Rect searchWindow;//模板匹配的矩形框大小
        Point templcenter;//记录的是模板中心距左上角点(x,y)的dx和dy距离
        //find_rect_template(searchWindow, templcenter, winHeight, winWidth);
        height = y_length_last * 3;
        width =x_dis_last * 2;
      //height = min(int(x_dis_last * tempScale) , winHeight);
      //  width = min(int(y_length_last * tempScale) , winWidth);
        /*if(y_length_last <30)
        {
          height = 140;
          width = 180;
        }
        else if(y_length_last <= 40 && y_length_last > 30)
        {
          height = 180;
          width = 220;
        }

        else if (y_length_last <= 50 && y_length_last > 40)
        {
          height = 220;
          width = 250;
        }

        else if(y_length_last <= 60 && y_length_last > 50)
        {
          height = 260;
          width = 300;
        }
        else if(y_length_last <= 70 && y_length_last > 60)
        {
          height = 300;
          width = 380;
        }
        else if(y_length_last <= 100 && y_length_last > 70)
        {
          height = 380;
          width = 420;
        }
        else if(y_length_last > 100)
        {
          height = winHeight - 50;
          width = winWidth - 70;
        }*/
        bool temp_flag = find_rect_template(searchWindow, templcenter, width, height);
        //cout<<"ddddddddddddddd"<<searchWindow.x<<" "<<searchWindow.y<<endl;
        if(temp_flag == false)
        {
            depth = 0;
            //length_last = 0;
            track_num =0;
            return true;
        }
        Mat roi = formalimg(searchWindow);

        //图片缩放 提高匹配速度
        Mat roi_last;
        Mat currentimg_last;
        resize(roi, roi_last, Size( fScale*searchWindow.width, fScale*searchWindow.height), (0, 0), (0, 0), INTER_LINEAR);
        resize(currentimg, currentimg_last, Size(fScale*winWidth, fScale*winHeight), (0, 0), (0, 0), INTER_LINEAR);
        //resize(roi, roi_last, Size( searchWindow.width>>2, searchWindow.height>>2), (0, 0), (0, 0), INTER_LINEAR);
        //resize(currentimg, currentimg_last, Size(winWidth>>2, winHeight>>2), (0, 0), (0, 0), INTER_LINEAR);
        //imshow("roi_last",roi_last);
        //imshow("roi",roi);

        Mat similarity;
        //matchTemplate(currentimg_last, roi_last, similarity, CV_TM_CCOEFF_NORMED);
        matchTemplate(currentimg_last, roi_last, similarity, CV_TM_CCOEFF);
        //matchTemplate(currentimg_last, roi_last, similarity, CV_TM_CCOEFF);
        double max_p;
        Point max_point;
        minMaxLoc(similarity, 0, &max_p, 0, &max_point);//CV_TM_CCORR_NORMED方法，最小值对应点最小点不是最优，设为0
        //cout<<max_p<<endl;
        if (max_p >= template_thre )//&& (pow((pow(py-py_last,2)+pow((px-px_last),2)),0.5)/distance_last<1 ||pow((pow(py-py_last,2)+pow((px-px_last),2)),0.5)/distance_last>2) )
        {
          /*Point test;
          test.x = max_point.x + templcenter.x * fScale;
          test.y = max_point.y + templcenter.y * fScale;

          circle(currentimg_last, test, 4, Scalar(255, 255, 255), -1);*/
            max_point.x = max_point.x/fScale + templcenter.x;
            max_point.y = max_point.y/fScale + templcenter.y;

            //imshow("pic",currentimg_last);
            armor_categray_num[1] ++;
            /*rect_2d[0] = Point2f(rect_2d[0].x + max_point.x - px, rect_2d[0].y + max_point.y - py);
            rect_2d[1] = Point2f(rect_2d[1].x + max_point.x - px, rect_2d[1].y + max_point.y - py);
            rect_2d[2] = Point2f(rect_2d[2].x + max_point.x - px, rect_2d[2].y + max_point.y - py);
            rect_2d[3] = Point2f(rect_2d[3].x + max_point.x - px, rect_2d[3].y + max_point.y - py);*/
            rect_2d[0] = Point2f(0,0);
            rect_2d[1] = Point2f(0,0);
            rect_2d[2] = Point2f(0,0);
            rect_2d[3] = Point2f(0,0);
            Center = max_point;

            track_num ++;
            return false;
        }
        else
        {
          /*Center = Point(0,0);
          rect_2d[0] = Point2f(0,0);
          rect_2d[1] = Point2f(0,0);
          rect_2d[2] = Point2f(0,0);
          rect_2d[3] = Point2f(0,0);
          armor_categray_num[0]=0;*/
          depth = 0;
          //length_last = 0;
          track_num ++;
          return true;
        }

    }
    else{
      track_num = 0;
      depth = 0;
      /*Center = Point(0,0);
      rect_2d[0] = Point2f(0,0);
      rect_2d[1] = Point2f(0,0);
      rect_2d[2] = Point2f(0,0);
      rect_2d[3] = Point2f(0,0);
      armor_categray_num[0]=0;*/
      //length_last = 0;
      return true;
    }
}

bool template_track_roi(Mat& formalimg, Mat& currentimg, Point& Center, Rect& box)
{
    if ((track_num <= 50 && armor_categray_num[0] != 2) || (track_num <= 20 && armor_categray_num[0] == 2))
    {
        //imshow("current",currentimg);
        //imshow("formal",formalimg);
        //waitKey(0);
        track_num ++;
        Rect searchWindowRoi;//模板匹配的矩形框大小
        Point templcenterRoi;//记录的是模板中心距左上角点(x,y)的dx和dy距离

        //int roi_height, roi_width;
        //roi_width = min(int(x_dis_last * tempScaleW) , winWidth);
       // roi_height = min(int(y_length_last * tempScaleH) , winHeight);
        //cout<<"sdsd"<<roi_width<<"  "<<roi_height<<endl;

        /*bool temp_flag = find_rect_template(searchWindowRoi, templcenterRoi, roi_width, roi_height);
        if(temp_flag == false)
        {
            depth = 0;
            track_num =0;
            return true;
        }*/
       // find_rect_curr(searchWindowRoi, templcenterRoi, roi_width, roi_height);
        //cout<<"sddddddddddddddddddddddddd"<<searchWindowRoi.x<<" "<<searchWindowRoi.y<<" "<<searchWindowRoi.width<<"  "<<searchWindowRoi.height<<endl;
        //Mat formalroi = formalimg(searchWindowRoi);
        //cout<<"sddddddddddddddddddddddddd1"<<endl;
        Mat formalroi = formalimg;
        //imshow("roi",formalroi);

        Rect searchWindowCurr;
        Point templcenterCurr;
        int curr_height, curr_width;
        //curr_width = min(int(x_dis_last * currScaleW) , winWidth);
        //curr_height = min(int(y_length_last * currScaleH) , winHeight);

        curr_width = min(box.width * currScaleW, winWidth);
        curr_height = min(box.height * currScaleH, winHeight);
        find_rect_curr(searchWindowCurr, templcenterCurr, curr_width, curr_height);
        Mat currroi = currentimg(searchWindowCurr);
        //imshow("curr",currroi);
        //imshow("currroi",currroi);
        //Mat formalroi_test;//图片缩放 提高匹配速度
        //Mat currroi_test;
        //resize(formalroi, formalroi_test, Size( fScale*searchWindowRoi.width, fScale*searchWindowRoi.height), (0, 0), (0, 0), INTER_LINEAR);
        //resize(currroi, currroi_test, Size(fScale*searchWindowCurr.width, fScale*searchWindowCurr.height), (0, 0), (0, 0), INTER_LINEAR);

        Mat similarity;// = Mat::zeros(roi_width, roi_height, CV_32FC1);
        //matchTemplate(currroi_test, formalroi_test, similarity, CV_TM_CCOEFF_NORMED);
        matchTemplate(currroi , formalroi, similarity, CV_TM_CCOEFF_NORMED);
        //imshow("result",similarity);
        double max_p;
        Point max_point;
        minMaxLoc(similarity, 0, &max_p, 0, &max_point);//CV_TM_CCORR_NORMED方法，最小值对应点最小点不是最优，设为0
        //cout<<max_p<<endl;
        //cout<<"similarity"<<max_p<<endl;

        if (max_p >= template_thre)
        {
            /*Mat currroi_copy;
            currroi.copyTo(currroi_copy);
            Point test;
            test.x = max_point.x ;//+ templcenterRoi.x ;
            test.y = max_point.y ;//+ templcenterRoi.y ;
            rectangle(currroi_copy, test, Point(test.x + formalroi.cols, test.y + formalroi.rows), Scalar(255, 255, 255), 2);
            circle(currroi_copy, test, 4, Scalar(255, 255, 255), -1);

            circle(currroi_copy, Point(currroi_copy.cols/2, currroi_copy.rows/2), 4, Scalar(255, 255, 255), -1);
            imshow("sdds",currroi_copy);*/

            //waitKey(0);
            box.x = max_point.x + searchWindowCurr.x;
            box.y = max_point.y + searchWindowCurr.y;
            box.width = formalroi.cols;
            box.height = formalroi.rows;
            //Mat currcopy = currentimg.clone();
            //rectangle(currcopy, box, Scalar(0,100,100), 3);
            //imshow("sdsdsd",currcopy);

            max_point.x = max_point.x + formalroi.cols/2 + searchWindowCurr.x;//+ templcenterRoi.x
            max_point.y = max_point.y + formalroi.rows/2 + searchWindowCurr.y;

            /*
            rect_2d[0] = Point2f(rect_2d[0].x + max_point.x - px, rect_2d[0].y + max_point.y - py);
            rect_2d[1] = Point2f(rect_2d[1].x + max_point.x - px, rect_2d[1].y + max_point.y - py);
            rect_2d[2] = Point2f(rect_2d[2].x + max_point.x - px, rect_2d[2].y + max_point.y - py);
            rect_2d[3] = Point2f(rect_2d[3].x + max_point.x - px, rect_2d[3].y + max_point.y - py);*/
            rect_2d[0] = Point2f(0,0);
            rect_2d[1] = Point2f(0,0);
            rect_2d[2] = Point2f(0,0);
            rect_2d[3] = Point2f(0,0);
            Center = max_point;
            return false;
        }
        else
        {
            track_num = 0;
            depth = 0;
            return true;
        }

    }
    else
    {
        track_num = 0;
        depth = 0;
        return true;
    }
}

#endif // ARMOR_H_INCLUDED
