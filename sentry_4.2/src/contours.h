/*****************************************************************
*  Copyright (C), 2018, Lesley
*  File name : contours.h
*  Author : Lesley

*  Description : This programm is used to find the contour of the 
		 target armor. 
******************************************************************/

#ifndef CONTOURS_H_INCLUDED
#define CONTOURS_H_INCLUDED

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <cxcore.h>
#include <highgui.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/videoio.hpp>

#include "adjust_color.h"
#include <iostream>
using namespace std;
using namespace cv;

int winHeight = 480;
int winWidth = 640;
double thresholdarea = 0.25; //0.5拟合矩形面积和原本凸包面积相差过大删去
int num_thre ;//= 10;

Mat RGBpic, gray, two, two_copy, hsv, two_number, gray_number;//, twocopy
Mat formalGrayPic; //存取前一张图片


int armor_categray_num[2] = {0,0}; //1-->small; 2-->big
int px=0, py=0;
Point2f rect_2d[4];

double distance_last = 0;
vector <double> distance_last_array;
double px_last, py_last, px_next;
double x_dis_last = 0;
double y_length_last = 0;
double area_last = 0;
//double length_last = 0;

double depth = 0;

struct point_flag
{
    double flag;
    double test_big_small;
    double area_current;
    double distance_current;
    //double length_current;
    double x_dis_current;
    int point_a, point_b;
};

struct num_have
{
    int num=0;
    bool have_or_not=0;//1->have
};

point_flag point_flag_last;

/**
*  @Function : full_small_contours(Mat& formalimg, Mat& finalimg)
*  @Description : Find the small or mismatched contours in the 
		  binary graph and full the contours.
*/
void full_small_contours(Mat& formalimg, Mat& finalimg)
{
    vector<vector<Point> > contoursinit;
    vector<Vec4i> hierarchyinit;
    double rectarea, arearadio;
    findContours(formalimg, contoursinit, hierarchyinit, RETR_CCOMP, CHAIN_APPROX_SIMPLE);
   // imshow("test",gray);
    if (!contoursinit.empty() & !hierarchyinit.empty()) {
        for (int idx = 0; idx < contoursinit.size(); idx++) {
            RotatedRect box = minAreaRect(contoursinit[idx]);
            Point2f vertex[4];//矩形的四个顶点
            box.points(vertex);

            double angle;
            if(box.size.width <= box.size.height)
            {
                angle = box.angle;
            }
            else
            {
                if(box.angle >= 0 )
                    angle = -(90 - fabs(box.angle));
                else if(box.angle < 0 )
                    angle = (90 - fabs(box.angle));
            }
            double minbox = MIN(box.size.height, box.size.width);
            double maxbox = MAX(box.size.height, box.size.width);

            double width, height;
            if(angle>0)//顺时针遇到的第一个边是height（angle为正）
            {
                width = box.size.height; //偏短的那个
                height = box.size.width;
            }
            else if(angle<0)//逆时针遇到的第一个边是width（angle为负）
            {
                width = box.size.width;
                height = box.size.height;
            }
            else
            {
                width = minbox;
                height = maxbox;
            }


            int color=0;
            //int red = 0;
            //int blue = 0;
            int color_sum = 0;

            for(int k=box.center.x - 2*minbox/3; k < box.center.x + 2*minbox/3 ; k++)
            {
                for(int t=box.center.y - 2*maxbox/3; t<box.center.y + 2*maxbox/3; t++)
                {
                    if(t<0 || k<0 || t>winHeight || k>winWidth)
                        break;
                    else
                    {
                        color_sum ++;
                        color += int(hsv.at<Vec3b>(t, k)[0]); //at访问（k,t）点元素像素，与像素点正好颠倒
                        //red += int(RGB.at<Vec3b>(t, k)[2]);
                        //blue += int(RGB.at<Vec3b>(t, k)[1]);
                    }
                }
            }

            rectarea = box.size.width * box.size.height;
            arearadio = contourArea(contoursinit[idx], false) / rectarea;

            if (contourArea(contoursinit[idx], false)<=2 || width/height > 1.5//minbox/maxbox>1.5
            || contourArea(contoursinit[idx], false)>=2000 || angle > 30 || angle < -30
            || maxbox < 9 || arearadio < thresholdarea || minbox < 2.5 || color_sum==0 ||
	    //敌方是红方，识别红色，自己是蓝方
	    (color_sum !=0 && !(color/color_sum>100 && color/color_sum<165)))
	    //敌方是蓝方，识别蓝色，自己是红方
	    //(color_sum !=0 && !(color/color_sum<55 || color/color_sum>170)))
            {
                drawContours(finalimg, contoursinit, idx, Scalar(0, 0, 0), -1, 8);//填充轮廓内部
            }

        }
    }
}

/**
*  @Function : save_hull_info..
*  @Description : Get the information of convex hulls accroding to contours.
		  Save the information in the struct kbarray.
		  Set kbarray[2] is 0 if the hull dismatch the conditon.
*/
void save_hull_info(vector<vector<double> >& kbarray, vector<vector<Point> >& hull, vector<vector<Point> >& contours)
{
    double box_w, box_h ;
    double rectarea, arearadio;
    double angle;
    for (int i = 0; i < hull.size(); i++)
    {
        RotatedRect box = minAreaRect(hull[i]);
        Point2f vertex[4];//矩形的四个顶点
        box.points(vertex);

        rectarea = box.size.width * box.size.height;
        arearadio = contourArea(contours[i], false) / rectarea;
        box_w = box.size.width;
        box_h = box.size.height;

        double minbox = MIN(box.size.height, box.size.width);
        double maxbox = MAX(box.size.height, box.size.width);

        if(box_w <= box_h)
        {
            angle = box.angle;
        }
        else
        {
            if(box.angle >= 0 )
                angle = -(90 - fabs(box.angle));
            else if(box.angle < 0 )
                angle = (90 - fabs(box.angle));
        }
/*
        double width, height;
            if(angle>0)//顺时针遇到的第一个边是height（angle为正）
            {
                width = box.size.height; //偏短的那个
                height = box.size.width;
            }
            else if(angle<0)//逆时针遇到的第一个边是width（angle为负）
            {
                width = box.size.width;
                height = box.size.height;
            }
            else
            {
                width = minbox;
                height = maxbox;
            }
        cout<<"angle"<<box.angle<<"  "<<angle<<" height "<<box.size.height<<" "<<box.size.width<<endl;
        cout<<"width  "<<width <<" height"<<height<<endl;
*/
        //if(arearadio >= thresholdarea && angle <= 30 && angle >= -30)
        {
            for(int j=0;j<4;j++){
            line(RGBpic,Point(vertex[j].x, vertex[j].y),Point(vertex[(j+1)%4].x, vertex[(j+1)%4].y),Scalar(200,0,100),2,LINE_AA);
            }

            if(contours[i].size() > 5)//fitEllipse使用前判断是否大于5个点,小于5个点要报错
            {
                RotatedRect fit_ellipse = fitEllipse(Mat(contours[i]));
                double minell = MIN(fit_ellipse.size.height, fit_ellipse.size.width);
                double maxell = MAX(fit_ellipse.size.height, fit_ellipse.size.width);

                if( maxbox/maxell > 0.75 && maxbox/maxell < 1.25)
                {
                    if(fit_ellipse.angle > 90)
                        angle = fit_ellipse.angle - 180;
                    else
                        angle = fit_ellipse.angle;
                    kbarray[i][0] = angle;			    //与垂直方向夹角
                    kbarray[i][1] = pow( (pow(fit_ellipse.size.height, 2) + pow(fit_ellipse.size.width, 2)), 0.5); //长度
                    kbarray[i][2] = contourArea(contours[i], false);		//实际面积
                    kbarray[i][3] = fit_ellipse.center.x; 		//矩形中心点的x坐标
                    kbarray[i][4] = fit_ellipse.center.y;		//矩形中心点的y坐标
                    kbarray[i][5] = minell;
                    kbarray[i][6] = maxell;
                    kbarray[i][7] = i ;
                    //ellipse(RGBpic, fit_ellipse.center, Size2f(fit_ellipse.size.width/2, fit_ellipse.size.height/2), fit_ellipse.angle,0,360,255,2,1);
                }
                else
                {
                    kbarray[i][0] = angle;			    //与垂直方向夹角
                    kbarray[i][1] = pow( (pow(box.size.height, 2) + pow(box.size.width, 2)), 0.5); //长度
                    kbarray[i][2] = contourArea(contours[i], false);		//实际面积
                    kbarray[i][3] = box.center.x; 		//矩形中心点的x坐标
                    kbarray[i][4] = box.center.y;		//矩形中心点的y坐标
                    kbarray[i][5] = minbox;
                    kbarray[i][6] = maxbox;
                    kbarray[i][7] = i;
                }
            }
            else
            {
                kbarray[i][0] = angle;			    //与垂直方向夹角
                kbarray[i][1] = pow( (pow(box.size.height, 2) + pow(box.size.width, 2)), 0.5); //长度
                kbarray[i][2] = contourArea(contours[i], false);		//实际面积
                kbarray[i][3] = box.center.x; 		//矩形中心点的x坐标
                kbarray[i][4] = box.center.y;		//矩形中心点的y坐标
                kbarray[i][5] = minbox;
                kbarray[i][6] = maxbox;
                kbarray[i][7] = i;
            }
        }

        /*else
        {
            kbarray[i][2]=0;
        }*/
    }
}

/**
*  @Function : search_rect..
*  @Description : Find the matched highlight.
*/
void search_rect(vector<vector<Point> >& hull, vector<point_flag>& mylist, int num_hull, vector<vector<double> >& kbarray)
{
    float x_max, x_min;
    int hie_num;
    double area, xdis, ydis, distance, xn, yn, pointdis, areasimilar, area_diff,
                length, dis_center, div_y_length, div_x_length, length_max, length_min;
    double p1, p2, p3, p4, p5, p6, p7, f;
    double real_distance, test_big_small, test_big_small_number, big_small, y_divide_x_dis;
    bool num_have_curr = 0, num_have_last = 0;
    int pixel;
    int middle_circle;
    vector<num_have> num_have_all;
    for (int i=0; i < num_hull-1 ; i++)
    {
        //if (kbarray[i][2] != 0)
        {
            for (int j = i+1; j < num_hull; j++)
            {
                if (//kbarray[j][2] != 0 &&
                fabs(kbarray[i][0]-kbarray[j][0])<=18 && kbarray[j][0] * kbarray[i][0] > -80)//(fabs(kbarray[i][0]-kbarray[j][0])<=4 && )kbarray[j][0] * kbarray[i][0] > -3) //直线尽量竖直
                {
                    int maxpoint, minpoint;
                    if( kbarray[i][3]>kbarray[j][3])
                    {
                        maxpoint = i;
                        minpoint = j;
                    }
                    else
                    {
                        maxpoint = j;
                        minpoint = i;
                    }

                    x_max = kbarray[maxpoint][3] + kbarray[maxpoint][5]/2;
                    x_min = kbarray[minpoint][3] - kbarray[minpoint][5]/2;
                    hie_num = x_max - x_min;
                    if(hie_num > 18)
                    {
                        area = kbarray[i][2] + kbarray[j][2] ;//两个包围区域的面积之和
                        xdis = fabs(kbarray[i][3] - kbarray[j][3]);//两装甲片中心点x坐标差值
                        ydis = fabs(kbarray[i][4] - kbarray[j][4]);//两装甲片中心点高度差值
                        distance = pow(pow(xdis, 2) + pow(ydis, 2), 0.5);//两矩形中心点之间的距离
                        xn = (kbarray[i][3] + kbarray[j][3]) / 2;
                        yn = (kbarray[i][4] + kbarray[j][4]) / 2; //两矩形间的中心
                        pointdis = pow(pow((xn-px), 2) + pow((yn-py), 2), 0.5);//距上次距离
                        dis_center = pow(pow((xn-winWidth/2), 2) + pow((yn-winHeight/2), 2), 0.5);//距中心点距离
                        areasimilar = MIN(kbarray[i][2] / kbarray[j][2], kbarray[j][2]/ kbarray[i][2]);
                        area_diff = MIN(area/area_last, area_last/area);
                        length = kbarray[i][1]+kbarray[j][1];
                        div_y_length = ydis/length;
                        div_x_length = xdis/length;
                        length_max = MAX(kbarray[i][1], kbarray[j][1]);
                        length_min = MIN(kbarray[i][1], kbarray[j][1]);

                        //!第1个决策变量 --> 面积越大越好 -->越大越好打 700->0.8 300->0.4
                        p1 = MIN(0.001*area+0.1, 1);

                        //!第2个决策变量 -->距上次位置越小越好 -->云台移动较小距离 0->1 200->0.1
                        p2 = MAX(-0.0045*pointdis + 1, 0);

                        real_distance = pow(pow(fabs(1300*(exp(-0.1002 * (kbarray[i][1]-2)) - exp(-0.1002 * (kbarray[j][1]-2))) + 1.1814 * (kbarray[i][1]-kbarray[j][1])) , 0.5)
                                                    *pow(length,2)/36+pow(distance,2) , 0.5);

                        test_big_small = (kbarray[i][6]+kbarray[j][6])/real_distance;
                        ///0.2-0.7是大装甲 0.7-1.2是小装甲

                        int hie_single[hie_num]={0};
                        //Point2f vertex[4];//矩形的四个顶点
                        //Mat ttt = imread("timg.png");

                        middle_circle = 0;
                        for(int k=x_min; k < x_max; k++)
                        {
                            int curr_num = k-x_min;
                            for(int t=yn - length*0.5; t<yn + length*0.5; t++)
                            {
                                if(k<0 || k>winWidth || t<0 || t>winHeight)
                                    continue;
                                pixel = int(two_number.at<uchar>(t,k));
                                middle_circle += pixel;
                                hie_single[curr_num] += pixel;
                            }

                            //hie_single[curr_num] =hie_single[curr_num]>4000? 125:(hie_single[curr_num] >> 5);//除以32
                            hie_single[curr_num] /= 255;
/*
                            vertex[0] = Point2f((curr_num-1)*5+1, hie_single[curr_num]);
                            vertex[3] = Point2f(curr_num*5, hie_single[curr_num]);
                            vertex[1] = Point2f((curr_num-1)*5+1, 0);
                            vertex[2] = Point2f(curr_num*5, 0);
                            for(int j=0;j<4;j++)
                            {
                                line(ttt,Point(vertex[j].x, vertex[j].y),Point(vertex[(j+1)%4].x, vertex[(j+1)%4].y),Scalar(0,2*curr_num,5*curr_num),1,LINE_AA);
                            }
*/
                        }
/*
                        vertex[0] = Point2f((x_max-x_min)*5+1, 12);
                        vertex[3] = Point2f((x_max-x_min+1)*5, 12);
                        vertex[1] = Point2f((x_max-x_min)*5+1, 0);
                        vertex[2] = Point2f((x_max-x_min+1)*5, 0);
                        for(int j=0;j<4;j++)
                        {
                            line(ttt,Point(vertex[j].x, vertex[j].y),Point(vertex[(j+1)%4].x, vertex[(j+1)%4].y),Scalar(0,0,0),1,LINE_AA);
                        }
*/

                        num_have tmp;
                        num_thre = 0.15*length;
                        //cout<<"num_thre"<<num_thre<<endl;

                        num_have_curr = hie_single[0] > num_thre? 1:0;
                        tmp.have_or_not = num_have_curr;
                        tmp.num++;
                        for(int index=1; index<hie_num; index++)
                        {
                            num_have_last = num_have_curr;
                            num_have_curr = hie_single[index] > num_thre? 1:0;
                            if(num_have_curr == num_have_last)
                                tmp.num++;
                            else if(tmp.num>=0.055*hie_num)
                            {
                                //cout<<0.05*hie_num<<endl;
                                num_have_all.push_back(tmp);
                                tmp.num = 0;
                                tmp.have_or_not = num_have_curr;
                                tmp.num++;
                            }
                            else if(num_have_all.size()>=1)
                            {
                                tmp = num_have_all.back();
                                num_have_all.erase(num_have_all.end()-1);
                                tmp.num++;
                            }
                            else
                            {
                                tmp.num = 0;
                                tmp.have_or_not = num_have_curr;
                                tmp.num++;
                            }
                        }
                        if(tmp.num >= 0.05*hie_num)
                            num_have_all.push_back(tmp);

                        bool number_of_num_all = 0;
                        double test_big_small_number = 0; //0不确定 1是小装甲 0.5是大装甲
                        if(num_have_all.size() == 5 && num_have_all[2].have_or_not==1)
                        {
                            number_of_num_all = 1;
                            float sum1 = num_have_all[1].num + num_have_all[3].num;
                            float sum2 = num_have_all[2].num;
                            float have_not_ratio = sum2 / sum1;
                            cout<<"have_not_ratio"<<have_not_ratio<<endl;
                            if(have_not_ratio >= 2.5 && have_not_ratio <= 8)
                            {
                                test_big_small_number = 1;
                            }
                            else if(have_not_ratio >= 0.3 && have_not_ratio <= 2.5)
                            {
                                test_big_small_number = 0.7;
                            }
                            else if(have_not_ratio >= 0.08 && have_not_ratio < 0.3 )
                            {
                                test_big_small_number = 0.5;
                            }

                            if(have_not_ratio >= 8)
                            {
                              number_of_num_all = 0;
                            }
                        }
                        else if(num_have_all.size() == 3 && num_have_all[1].have_or_not==0)
                        {
                            number_of_num_all = 1;
                            test_big_small_number = 0.7;
                            float sum1 = num_have_all[0].num + num_have_all[2].num;
                            float sum2 = num_have_all[1].num;
                            float have_not_ratio = sum2 / sum1;
                            //cout<<"have_not_ratio"<<have_not_ratio<<endl;
                            if(have_not_ratio >= 0.7)
                            {
                                number_of_num_all = 0;
                            }
                            if(have_not_ratio >= 4 && div_x_length > 1.8 && div_x_length <= 2.3 && div_x_length + div_y_length < 2.6)
                            {
                                number_of_num_all = 1;
                                test_big_small_number = 0.5;
                            }
                        }
                        /*else if(num_have_all.size() == 3 && num_have_all[0].have_or_not==0
                                    && div_x_length >= 1.8 && div_x_length <= 2.1 && div_x_length + div_y_length < 2.3)
                        {
                            number_of_num_all = 1;
                            test_big_small_number = 0.5;
                        }*/
                        else if(num_have_all.size() == 1 && num_have_all[0].have_or_not==1)
                        {
                            number_of_num_all = 1;
                            test_big_small_number = 0.7;
                        }
                        /*else if(num_have_all.size() == 1 && num_have_all[0].have_or_not==0 )
                        {
                            if(div_x_length >= 1.8 && div_x_length <= 2.1 && div_x_length + div_y_length < 2.3)
                            {
                                number_of_num_all = 1;
                                test_big_small_number = 0.5;
                            }
                        }*/
                        else if(num_have_all.size() == 4)
                        {
                            if(num_have_all[0].have_or_not==1)
                            {
                                float sum1 = num_have_all[0].num + num_have_all[2].num;
                                float sum2 = num_have_all[1].num + num_have_all[3].num;
                                float have_not_ratio = sum2 / sum1;
                                if(have_not_ratio < 1.1)
                                {
                                    number_of_num_all = 1;
                                    test_big_small_number = 0.7;
                                }
                                else if(have_not_ratio >= 1.1)
                                {
                                    number_of_num_all = 1;
                                    test_big_small_number = 0.5;
                                }
                            }
                            else
                            {
                                float sum2 = num_have_all[0].num + num_have_all[2].num;
                                float sum1 = num_have_all[1].num + num_have_all[3].num;
                                float have_not_ratio = sum2 / sum1;
                                if(have_not_ratio < 1.1)
                                {
                                    number_of_num_all = 1;
                                    test_big_small_number = 0.7;
                                }
                                else if(have_not_ratio >= 1.1)
                                {
                                    number_of_num_all = 1;
                                    test_big_small_number = 0.5;
                                }
                            }
                        }



//{
/*
                    cout<<"num_have_all.size():"<<num_have_all.size()<<"  "<<x_max-x_min<<endl;
                    for(int kk=0; kk<num_have_all.size(); kk++)
                    {
                        cout<<"sdssadadad  "<<num_have_all[kk].num<<"  "<< num_have_all[kk].have_or_not<<endl;
                    }
*/
/*
                    namedWindow("sdfdsfdsf",CV_WINDOW_NORMAL);
                    imshow("sdfdsfdsf",ttt);
                    Mat RGBpiccopy;
                    RGBpic.copyTo(RGBpiccopy);
                    drawContours(RGBpiccopy, hull, i, Scalar(255,255,0), -1,LINE_8,noArray(),INT_MAX);
                    drawContours(RGBpiccopy, hull, j, Scalar(255,255,0), -1,LINE_8,noArray(),INT_MAX);
                    imshow("sfdsfdsfdssd",RGBpiccopy);
                    waitKey(0);
}*/

                        num_have_all.clear();
                        middle_circle = middle_circle / (length * (kbarray[maxpoint][3] - kbarray[minpoint][3]));
                        big_small = 0.3*test_big_small_number + 0.7*test_big_small;  //test_big_small_number!=0 ? (0.2*test_big_small + 0.8*test_big_small_number):test_big_small;

                        //!第3个决策变量 --> 面积距离比 小装甲约是1.5×大装甲  越接近其各自的最优值 越有可能是装甲 也越好瞄
                        if(armor_categray_num[0] == 1)// && armor_categray_num[1]<=120)//小装甲
                        {
                            if(big_small >=0.7 && big_small<=1) //0.7->0.25 1->1
                                p3 = 2.5 * test_big_small - 1.5;
                            else if(big_small >1 && big_small<=1.4)  //1->1 1.4->0.5
                                p3 = -1.25 * big_small + 2.25;
                            else
                                p3 = 0;
                        }
                        else if(armor_categray_num[0] == 2)// && armor_categray_num[1]<=120)//大装甲
                        {
                            if(big_small >=0.3 && big_small<=0.5) //0.5->1 0.4->0.6
                                p3 = 4 * big_small - 1;
                            else if(big_small >0.6 && big_small<0.7)//0.5->1 0.7->0.75
                                p3 = -1.25 * big_small + 1.625;
                            else
                                p3 = 0;
                        }
                        else
                        {
                            if(big_small >=0.7 && big_small<=1) //0.7->0.25 1->1
                                p3 = 2.5 * big_small - 1.5;
                            else if(big_small >1 && big_small<=1.4)  //1->1 1.4->0.5
                                p3 = -1.25 * big_small + 2.25;
                            else if(big_small >=0.3 && big_small<=0.5) //0.5->1 0.4->0.6
                                p3 = 4 * big_small - 1;
                            else if(big_small >0.6 && big_small<0.7)//0.5->1 0.7->0.75
                                p3 = -1.25 * big_small + 1.625;
                            else
                                p3 = 0;
                        }

                        //y_divide_x_dis = xdis > 10? ydis/xdis : 0;
                        //!第4个决策变量 兩個裝甲片中心點的x差值越大,ydis越小 越容易瞄準
                        if(xdis >= 10)
                        {
                            y_divide_x_dis = ydis/xdis;
                            if(big_small>=0.3 && big_small<0.7)//大装甲
                            {
                                if(y_divide_x_dis >= 0.25)
                                    p4 = 0;
                                else if(y_divide_x_dis <= 0.12)
                                    p4 = 1;
                                else
                                    p4 = -7.692*y_divide_x_dis + 1.923;

                            }
                            else if(big_small>=0.7 && big_small<1.7)//小装甲
                            {
                                if(y_divide_x_dis >= 0.45)
                                    p4 = 0;
                                else if(y_divide_x_dis <= 0.2)
                                    p4 = 1;
                                else
                                    p4 = -4*y_divide_x_dis + 1.8;
                            }
                            else //上一幀未檢測出 （大小装甲优先级一样）
                            {
                                if(y_divide_x_dis >= 0.45)
                                    p4 = 0;
                                else if(y_divide_x_dis <= 0.12)
                                    p4 = 1;
                                else
                                    p4 = -3*y_divide_x_dis + 1.35;
                            }
                        }
                        else
                        {
                            p4 = 0; //不希望xdis太小 无法打中
                        }

                        //!第5个决策变量 如果这一帧装甲片之间的距离与上一帧之间的变化小 那就是它惹。。
                        p5 = distance_last == 0? 1:MIN(distance_last/real_distance, real_distance/distance_last);

                        //!第6个决策变量 如果两个灯柱面积相似度高 那就是它惹。。 1->1 0.25->0
                        p6 = 4/3 *areasimilar - 1/3;

                        //!第7个决策变量 距中心点越近越好
                        p7 = 1 - dis_center / 400;
                        f = 0.5*p1 + 3*p2 + 2*p3 + 2*p4 + 2*p5 + 0.5*p6 + p7 + area_diff;

                        if (test_big_small_number!=0)
                        {
                            f *= 2;
                        }
                        if (big_small < 1.7 && big_small >= 0.7 && armor_categray_num[0] == 2)
                        {
                            f /= 1.5;
                        }
                        if (big_small < 0.7 && big_small > 0.3 && armor_categray_num[0] == 1)
                        {
                            f /= 1.5;
                        }

                        if (kbarray[i][5] > 10 && kbarray[j][5] > 10 && areasimilar<0.2){}//cout<<"fail1:areasimilar"<<areasimilar<<endl;}
                        else if((kbarray[i][5] > 12 || kbarray[j][5] > 12) && min(kbarray[i][5],kbarray[j][5])/max(kbarray[i][5],kbarray[j][5])<0.75){}
			else if(area<4){}
                        else if(f<4){}//cout<<"fail:f"<<f<<endl;}
                        else if(length < 20){}
                        else if(middle_circle < 15 && test_big_small_number == 1){}//cout<<"fail1:middle_circle"<<middle_circle<<endl;}
                        else if(middle_circle < 10 && test_big_small_number != 1){}//cout<<"fail2:middle_circle"<<middle_circle<<endl;}
                        //else if(middle_circle > 260){cout<<"fail1:middle_circle too big"<<middle_circle<<endl;}
                        else if(div_x_length > 2.6){}//cout<<"div_x_length1:"<<div_x_length<<endl;}
                        else if(div_x_length + div_y_length > 2.9){}//cout<<"div_xy_length1:"<<div_y_length<<"length"<<length<<endl;}
                        else if(div_x_length < 0.4){}//cout<<"div_x_length3"<<endl;}
                        else if(div_y_length > 1.2){}//cout<<"div_y_length1"<<endl;}
                        else if(div_y_length > 0.75 && length<40){}//cout<<"div_y_length1"<<endl;}
                        //else if(length < 26){cout<<"formal6"<<endl;}

                        //else if(length < 40 && !(fabs(kbarray[i][0]-kbarray[j][0])<=5 && kbarray[j][0] * kbarray[i][0] > -8)){}
                        //else if(distance/area > 1){cout<<"formal8"<<endl;}
                        //else if(length_min/length_max<0.45 && length_max>20){cout<<"formal9"<<endl;}
                        else if(length_min/length_max<0.4){}//cout<<"length_min/length_max"<<length_min/length_max<<endl;}
                        else if(big_small < 0.3 || big_small > 1.7){}//cout<<"formal11"<<endl;}
                        else if(!number_of_num_all){}//cout<<"sdsssssssssssssssssss"<<endl;}
                        //else if(big_small > 0.3 && big_small <0.7 && (armor_categray_num[0] == 1 || (armor_categray_num[0] == 0 && armor_categray_num[1] <3)))
                                                                           // {cout<<"test"<<endl;}
                        else if(div_x_length + div_y_length > 1.7 && test_big_small_number == 1){}//cout<<"div_x_length"<<div_x_length<<endl;}
                        else if(div_x_length + div_y_length > 1.7 && !(fabs(kbarray[i][0]-kbarray[j][0])<=10 && kbarray[j][0] * kbarray[i][0] > -25))
                                                          {}//cout<<"div_x_length"<<div_x_length<<"angle"<<kbarray[i][0]<<"  "<<kbarray[j][0]<<endl;}

                        //else if(test_big_small >= 0.3 && test_big_small <0.87 && div_x_length>1.03 && div_x_length<1.23 && length < 31){cout<<"formal12"<<endl;}
                        //else if(test_big_small >= 0.3 && test_big_small <0.86 && div_x_length>0.8 && div_x_length<1.35 && length < 42){cout<<"formal12"<<endl;}
                        //else if(((test_big_small >= 0.3 && test_big_small <0.7) || armor_categray_num[0] == 2) && length < 45){cout<<"formal12"<<endl;}
                        //else if(armor_categray_num[0] == 1 && test_big_small > 0.3 && test_big_small < 0.7){cout<<"demo1"<<endl;}
                        //else if(armor_categray_num[0] == 2 && test_big_small >= 0.7 && test_big_small < 1.7){cout<<"demo2"<<endl;}
                        //else if(ydis/xdis > 0.62){cout<<"demo4"<<endl;}
/*
                        else if((armor_categray_num[0] == 1 || (armor_categray_num[0] == 0 && armor_categray_num[1] <3)) && div_x_length>1.38 && length >= 80 ){cout<<"test1"<<endl;}
                        else if((armor_categray_num[0] == 1 || (armor_categray_num[0] == 0 && armor_categray_num[1] <3)) && div_x_length>1.35 && length < 80 && length >=65){cout<<"test2"<<endl;}
                        else if((armor_categray_num[0] == 1 || (armor_categray_num[0] == 0 && armor_categray_num[1] <3)) && div_x_length>1.32 && length >= 50 && length < 65){cout<<"test3"<<endl;}
                        else if((armor_categray_num[0] == 1 || (armor_categray_num[0] == 0 && armor_categray_num[1] <3)) && div_x_length>1.28 && length >=46 && length < 50){cout<<"test4"<<endl;}//1.13
                        else if((armor_categray_num[0] == 1 || (armor_categray_num[0] == 0 && armor_categray_num[1] <3)) && div_x_length>1.25 && length >=42 && length < 46 ){cout<<"test5"<<endl;}
                        else if((armor_categray_num[0] == 1 || (armor_categray_num[0] == 0 && armor_categray_num[1] <3)) && div_x_length>1.2 && length >=38 && length <42 ){cout<<"test6"<<endl;}
                        //else if(armor_categray_num[0] == 1 && div_x_length>1.07 && length >= 33.5 && length < 35 ){cout<<"test16"<<endl;}
                        else if((armor_categray_num[0] == 1 || (armor_categray_num[0] == 0 && armor_categray_num[1] <3)) && div_x_length>1.25 && length < 38 ){cout<<"test7"<<endl;}
                        else if((armor_categray_num[0] == 1 || (armor_categray_num[0] == 0 && armor_categray_num[1] <3)) && div_x_length>1.05 &&length_min/length_max >0.9 && length < 38 ){cout<<"test8"<<endl;}
                        //else if(armor_categray_num[0] == 1 && min(kbarray[i][1], kbarray[j][1]) <18.5 && div_x_length >1){}

                        else if((armor_categray_num[0] == 1 || (armor_categray_num[0] == 0 && armor_categray_num[1] <3)) && div_x_length>1.32 && div_y_length < 0.05 && length >= 80 ){cout<<"test9"<<endl;}
                        else if((armor_categray_num[0] == 1 || (armor_categray_num[0] == 0 && armor_categray_num[1] <3)) && div_x_length>1.27 && div_y_length < 0.05 && length < 80 && length >=65){cout<<"test10"<<endl;}
                        else if((armor_categray_num[0] == 1 || (armor_categray_num[0] == 0 && armor_categray_num[1] <3)) && div_x_length>1.24 && div_y_length < 0.05 && length >= 50 && length < 65){cout<<"test11"<<endl;}
                        else if((armor_categray_num[0] == 1 || (armor_categray_num[0] == 0 && armor_categray_num[1] <3)) && div_x_length>1.22 && div_y_length < 0.05 && length >= 45 && length < 50){cout<<"test12"<<endl;}
                        else if((armor_categray_num[0] == 1 || (armor_categray_num[0] == 0 && armor_categray_num[1] <3)) && div_x_length>1.2 && div_y_length < 0.05 && length >= 42&& length < 45 ){cout<<"test13"<<endl;}

                        else if((armor_categray_num[0] == 1 || (armor_categray_num[0] == 0 && armor_categray_num[1] <3)) && div_x_length + div_y_length > 1.75 && length >= 89){cout<<"test14"<<endl;}
                        else if((armor_categray_num[0] == 1 || (armor_categray_num[0] == 0 && armor_categray_num[1] <3)) && div_x_length + div_y_length > 1.67 && length < 89 && length >= 80){cout<<"test15"<<endl;}
                        else if((armor_categray_num[0] == 1 || (armor_categray_num[0] == 0 && armor_categray_num[1] <3)) && div_x_length + div_y_length > 1.62 && length < 80 && length >=65){cout<<"test16"<<endl;}
                        else if((armor_categray_num[0] == 1 || (armor_categray_num[0] == 0 && armor_categray_num[1] <3)) && div_x_length + div_y_length > 1.57 && length < 65 && length >=50){cout<<"test17"<<endl;}
                        else if((armor_categray_num[0] == 1 || (armor_categray_num[0] == 0 && armor_categray_num[1] <3)) && div_x_length + div_y_length > 1.52 && length > 42 && length < 50){cout<<"test18"<<endl;}
                        else if((armor_categray_num[0] == 1 || (armor_categray_num[0] == 0 && armor_categray_num[1] <3)) && div_x_length + div_y_length > 1.48 &&length >= 35 && length <= 42 ){cout<<"test19"<<endl;}
                        else if((armor_categray_num[0] == 1 || (armor_categray_num[0] == 0 && armor_categray_num[1] <3)) && div_x_length + div_y_length > 1.4 && length < 35 ){cout<<"test20"<<endl;}
*/

                        else if((y_divide_x_dis>0.8 || xdis==0) && length <= 50){}//cout<<"demo1"<<endl;}
                        else if((y_divide_x_dis>0.35 || xdis==0) && armor_categray_num[0] == 2 && length <= 50){}//cout<<"demo2"<<endl;}
                        else if(div_x_length<0.2 && div_y_length>0.5){}//cout<<"demo3"<<endl;}
/*
                        else if(armor_categray_num[0] == 2 && length>60 && xdis+ydis<64){cout<<"tttt1"<<endl;}
                        else if(armor_categray_num[0] == 2 && length>50 && length<=60 && xdis+ydis<60){cout<<"tttt1"<<endl;}
                        else if(armor_categray_num[0] == 2 && length>46 && length<=50 && xdis+ydis<56){cout<<"tttt1"<<endl;}
                        else if(armor_categray_num[0] == 2 && length>43 && length<=46 && xdis+ydis<54){cout<<"tttt2"<<endl;}
                        else if(armor_categray_num[0] == 2 && length>40 && length<=43 && xdis+ydis<52){cout<<"tttt20"<<endl;}
                        else if(armor_categray_num[0] == 2 && length>35 && length <=40 && xdis+ydis<50){cout<<"ttttttt3"<<endl;}
                        else if(armor_categray_num[0] == 2 && length<=35 && xdis+ydis<46){cout<<"ttttttt4"<<endl;}
                        else if(armor_categray_num[0] == 2 && ydis/xdis >0.16){cout<<"ttttttt5"<<endl;}

                        //else if(armor_categray_num[0] == 1 && length<60 && length>=50 && xdis+ydis>68){cout<<"ttttttt16"<<endl;}
                        else if(armor_categray_num[0] == 1 && length<60 && length>=50 && xdis+ydis>63){cout<<"ttttttt26"<<endl;}
                        else if(armor_categray_num[0] == 1 && length<50 && length>=43 && xdis+ydis>56){cout<<"ttttttt36"<<endl;}
                        else if(armor_categray_num[0] == 1 && length<43 && length>=40 && xdis+ydis>49){cout<<"ttttttt7"<<endl;}
                        else if(armor_categray_num[0] == 1 && length<40 && length>=35 && xdis+ydis>45){cout<<"ttttttt8"<<endl;}
                        else if(armor_categray_num[0] == 1 && length<35 && xdis+ydis>43){cout<<"ttttttt9"<<endl;}*/


                        //else if(test_big_small<0.7 && div_x_length<1.2 && div_y_length<0.1){cout<<"ttttttt12"<<endl;}
                        //else if(test_big_small<0.7 && length < 47 && div_x_length+div_y_length<1.7){}

                        //else if(test_big_small>0.3 && test_big_small <0.7 && lastsmalldis < 20*pow(length,0.5) && px!=0){cout<<"ttttttt13"<<endl;}
                      //  else if(pointdis<0.8 * distance && pointdis<1.2 * distance ){}
                        else if(xdis/area < 0.07){}//cout<<"fail:xdis/area < 0.07"<<endl;}
                        //else if(xdis < 12){}
                        //else if(xdis_small <10){}
                        else
                        {
                            drawContours(RGBpic, hull, i, Scalar(255,0,255), -1, LINE_8, noArray(), INT_MAX);
                            drawContours(RGBpic, hull, j, Scalar(255,0,255), -1, LINE_8, noArray(), INT_MAX);
                            //cout<<"angle"<<kbarray[i][0]<<"  "<<kbarray[j][0]<<endl;
                            //cout<<"middle_circle"<<middle_circle<<endl;
                            //cout<<"div_y_length/areasimilar  "<<div_y_length/areasimilar<<endl;
                            //cout<<"div_x_length"<<div_x_length <<"  div_ylength  "<< div_y_length<< "  length "<<length<<endl;
                            //cout<<"xdis/area"<<xdis/area<<endl;
                            //cout<<"y_divide_x_dis"<<y_divide_x_dis<<endl;
                            //cout<<"length"<<length<<"   length_diff"<<length_diff<<endl;

                            //cout<<"dis_last_distance"<<dis_last_distance<<endl;

                            //cout<<"angle"<<kbarray[i][0]<<"  "<<kbarray[j][0]<<endl;
                            //cout<<"distance"<<distance<<"  area"<<area<<"  / "<<distance/area<<endl;
                            //cout<<"pointdis"<<pointdis<<endl;
                            //cout<<"ydis"<<ydis<<"  xdis"<<xdis<<"  / "<<ydis/xdis<<endl;
                            //cout<<"test_big_small"<<test_big_small<<"   realdistance "<<real_distance<<" distance"<<distance<<endl;
                            cout<<"big_small"<<big_small<<endl;
                            //cout<<"area"<<area<<"areasimilar"<<areasimilar<<endl;
                            cout<<"f"<<f<<"  p1:"<<p1<<"  p2:"<<p2<<"  p3:"<<p3<<"  p4:"<<p4<<"  p5:"<<p5<<"  p6:"<<p6<<"  p7:"<<p7<<endl;
                            point_flag demo;
                            demo.flag = f;
                            demo.test_big_small = big_small;
                            demo.area_current = area;
                            demo.distance_current = real_distance;
                            //demo.length_current = length;
                            demo.x_dis_current = xdis;
                            demo.point_a = i;
                            demo.point_b = j;
                            //demo.point_a_center = kbarray[i][3];
                            //demo.point_b_center = kbarray[j][3];
                            mylist.push_back(demo);
                        }
                    }
                }
            }
        }
    }
}

bool comp(const point_flag &a,const point_flag &b)
{
    return a.flag>b.flag; //从大到小排序
}
/**
*  @Function : sort_list..
*  @Description : Select the most matched highlight.
*/
bool sort_list(vector<point_flag>& mylist, vector<vector<double> >& kbarray, Point& Center, vector<vector<Point> >& hull, Rect &box)
{
    if(mylist.empty())
        return true;//需要再追踪
    sort(mylist.begin(), mylist.end(),comp);
    point_flag_last = mylist[0];

    int px_current = (kbarray[point_flag_last.point_a][3] + kbarray[point_flag_last.point_b][3]) / 2;
    int py_current = (kbarray[point_flag_last.point_a][4] + kbarray[point_flag_last.point_b][4]) / 2;//+ (kbarray[mylist[0].point_a][1] + kbarray[mylist[0].point_b][1]) / 4;
    //判断大小装甲
    if(armor_categray_num[1]>150)
    {
        armor_categray_num[0] = 0;
        armor_categray_num[1] = 0;
    }

    if(point_flag_last.test_big_small >= 0.7 && point_flag_last.test_big_small <= 1.7 ) //小装甲
    {
        if(armor_categray_num[0] != 2)
        {
            armor_categray_num[1] ++;
        }
        else
        {
            armor_categray_num[1] = 0;
        }
        armor_categray_num[0] = 1;
    }
    else if(point_flag_last.test_big_small > 0.3 && point_flag_last.test_big_small < 0.7 ) //大装甲
    {
        if(armor_categray_num[0] != 1)
        {
            armor_categray_num[1] ++;
        }
        else
        {
            armor_categray_num[1] = 0;
        }
        armor_categray_num[0] = 2;
    }

    if(armor_categray_num[1]>=2)
    {
        if(distance_last_array.size() == 0 )
        {
            distance_last_array.push_back(point_flag_last.distance_current);
            distance_last = point_flag_last.distance_current;
        }
        else if(distance_last_array.size() == 1 )
        {
            distance_last_array.push_back(point_flag_last.distance_current);
            distance_last = (distance_last_array[0] + point_flag_last.distance_current)/2;
        }
        else if(distance_last_array.size() == 2)
        {
            distance_last_array.push_back(point_flag_last.distance_current);
            distance_last = (distance_last_array[0] + distance_last_array[1] + point_flag_last.distance_current)/3;
        }
        else if(distance_last_array.size() >= 3)
        {
            distance_last_array.erase(distance_last_array.begin());
            distance_last_array.push_back(point_flag_last.distance_current);
            distance_last = (distance_last_array[0] + distance_last_array[1] + point_flag_last.distance_current)/3;
        }
    }
    else
    {
        distance_last_array.clear();
        distance_last = 0;
    }

    y_length_last = (kbarray[point_flag_last.point_a][1] + kbarray[point_flag_last.point_b][1]) / 2;
    area_last = point_flag_last.area_current;
    //length_last = mylist[0].length_current;
    x_dis_last = point_flag_last.x_dis_current;

    int point_small, point_big;
    point_small = kbarray[point_flag_last.point_a][3] <= kbarray[point_flag_last.point_b][3]? \
                                                        point_flag_last.point_a: point_flag_last.point_b;
    point_big = kbarray[point_flag_last.point_a][3] > kbarray[point_flag_last.point_b][3]?    \
                                                        point_flag_last.point_a: point_flag_last.point_b;
    rect_2d[0] = Point2f(kbarray[point_small][3] , kbarray[point_small][4] - 0.5*kbarray[point_small][1]);//*cos(kbarray[point_small][0]));
    rect_2d[1] = Point2f(kbarray[point_small][3] , kbarray[point_small][4] + 0.5*kbarray[point_small][1]);//*cos(kbarray[point_small][0]));
    rect_2d[2] = Point2f(kbarray[point_big][3] , kbarray[point_big][4] + 0.5*kbarray[point_big][1]);//*cos(kbarray[point_big][0]));
    rect_2d[3] = Point2f(kbarray[point_big][3] , kbarray[point_big][4] - 0.5*kbarray[point_big][1]);//*cos(kbarray[point_big][0]));

    Center.x = px_current;
    Center.y = py_current;

    vector <Point> hh;
    hh = hull[point_small];
    hh.insert(hh.end(),hull[point_small].begin(),hull[point_small].end());//将vec1压入
    hh.insert(hh.end(),hull[point_big].begin(),hull[point_big].end());//继续将vec2压入

    box = boundingRect(hh);

    return false;

}


#endif // CONTOURS_H_INCLUDED
