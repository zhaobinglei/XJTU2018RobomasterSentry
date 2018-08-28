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
#include <iostream>
using namespace std;
using namespace cv;

int winHeight = 480;
int winWidth = 640;
double area_threshold = 0.4;

struct point_flag
{
    double flag;
    double test_big_small;
    double distance_current;
    //double x_dis_current;
    //double area_current;
    //double length_current;
    //double point_a_center,point_b_center;
    int point_a, point_b;
};


void fullSmallContours(
                        Mat* formalimg,
                        Mat& finalimg
                        )
{
    vector<vector<Point> > contoursinit;
    vector<Vec4i> hierarchyinit;
    findContours(*formalimg, contoursinit, hierarchyinit, RETR_CCOMP, CHAIN_APPROX_SIMPLE);

    RotatedRect box;
    Point2f vertex[4];//矩形的四个顶点
    double angle, minbox, maxbox;
    bool cover_flag;
    int near_sum = 0, col, row, t, k;

    if (!contoursinit.empty() & !hierarchyinit.empty()) {
        for (int idx = 0; idx < contoursinit.size(); idx++) {
            box = minAreaRect(contoursinit[idx]);
            box.points(vertex);

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
            minbox = MIN(box.size.height, box.size.width);
            maxbox = MAX(box.size.height, box.size.width);

            for(t = 0; t<5*maxbox; t++)
            {
                for(k=0; k<5*maxbox; k++)
                {
                    col = box.center.y + k-2.5*maxbox;
                    row = box.center.x + t-2.5*maxbox;
                    if(col < 0 || row < 0 ||col > winHeight || row > winWidth)
                        break;
                    if((int)formalimg->at<uchar>(col, row) >= 4)
                        near_sum += 1;
                }
            }

            near_sum -= contourArea(contoursinit[idx], false);

            cover_flag = contourArea(contoursinit[idx], false)<=2 ||
                        (contourArea(contoursinit[idx], false)>=600 &&minbox/maxbox>1) ||
                        contourArea(contoursinit[idx], false)>=2000 ||
                        angle > 22 || angle < -22 || near_sum/contourArea(contoursinit[idx], false) > 150;

            if (cover_flag)
            {
                drawContours(finalimg, contoursinit, idx, Scalar(0, 0, 0), -1, 8);//填充轮廓内部
            }

        }
    }
}

//对每一个凸包均均求出其面积等信息，对于那些不符合光柱要求的凸包将其所有信息设置为0
//!信息放在kbarray中
void saveHullInfo(  Mat& rgbPic,
                    vector<vector<double> >& kbarray,
                    vector<vector<Point> >& hull,
                    vector<vector<Point> >& contours
                    )
{
    RotatedRect box;
    Point2f vertex[4];//矩形的四个顶点
    double rectarea, arearadio, minbox, maxbox, angle;
    bool box_flag;
    double minell, maxell;

    for (int i = 0; i < hull.size(); i++)
    {
        box = minAreaRect(hull[i]);
        box.points(vertex);

        rectarea = box.size.width * box.size.height;
        arearadio = contourArea(contours[i], false) / rectarea;
        minbox = MIN(box.size.height, box.size.width);
        maxbox = MAX(box.size.height, box.size.width);

        box_flag = (arearadio >= area_threshold || (minbox/maxbox<=0.22 && arearadio > 0.23))
                    && angle <= 25 && angle >= -25 && (minbox/maxbox <=0.9 ||(maxbox<20));
        if( box_flag )
        {
            /*for(int j=0;j<4;j++){
            line(rgbPic,Point(vertex[j].x, vertex[j].y),Point(vertex[(j+1)%4].x , vertex[(j+1)%4].y),Scalar(0,100,100),2,LINE_AA);
            }*/

           /* if(contours[i].size() > 5)//fitEllipse使用前判断是否大于5个点,小于5个点要报错
            {
                RotatedRect fit_ellipse = fitEllipse(Mat(contours[i]));
                minell = MIN(fit_ellipse.size.height, fit_ellipse.size.width);
                maxell = MAX(fit_ellipse.size.height, fit_ellipse.size.width);

                if( maxbox/maxell > 0.8 && maxbox/maxell < 1.2)
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
                    //ellipse(RGBpic, fit_ellipse.center+Point2f(Center), Size2f(fit_ellipse.size.width/2, fit_ellipse.size.height/2), fit_ellipse.angle,0,360,255,2,1);
                }
                else
                {
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
            else*/
            {
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
            kbarray[i][2]=0;
        }

    }
}

void searchRect(Mat& rgbPic,
                vector<vector<Point> >& hull,
                vector<point_flag>& mylist,
                int num_hull,
                vector<vector<double> >& kbarray,
                int pX,
                int pY,
                double distanceLast,
                int* armorCategrayNum
                )
{
    //double area, xdis, ydis, diatance, xn, yn, pointdis, areasimilar, length, length_diff, angle_sum;
    double real_distance;
    double test_big_small;
    double y_divide_x_dis ; //ydis/xdis
    double p1, p2, p3, p4, p5, p6, p7, p8, f;
    double dif_angle, dis_center;//距中心点距离
    double div_y_length, div_x_length;
    for (int i=0; i < num_hull-1 ; i++)
    {
        if (kbarray[i][2] != 0 )
        {
            for (int j = i+1; j < num_hull; j++)
            {
                if (kbarray[j][2] != 0 && fabs(kbarray[i][0]-kbarray[j][0])<=18 && kbarray[j][0] * kbarray[i][0] > -80)//(fabs(kbarray[i][0]-kbarray[j][0])<=4 && )kbarray[j][0] * kbarray[i][0] > -3) //直线尽量竖直
                {
                    double area = kbarray[i][2] + kbarray[j][2] ;//两个包围区域的面积之和
                    double xdis = fabs(kbarray[i][3] - kbarray[j][3]);//两装甲片中心点x坐标差值
                    double ydis = fabs(kbarray[i][4] - kbarray[j][4]);//两装甲片中心点高度差值
                    double distance = pow(pow(xdis, 2) + pow(ydis, 2), 0.5);//两矩形中心点之间的距离
                    double xn = (kbarray[i][3] + kbarray[j][3]) / 2;
                    double yn = (kbarray[i][4] + kbarray[j][4]) / 2; //两矩形间的中心
                    double pointdis = pow(pow((xn-pX), 2) + pow((yn-pY), 2), 0.5);//距上次距离
                    double areasimilar = MIN(kbarray[i][2] / kbarray[j][2], kbarray[j][2]/ kbarray[i][2]);
                    double length = kbarray[i][1]+kbarray[j][1];
                    double length_diff = fabs(kbarray[i][1]-kbarray[j][1]);
                    double angle_sum = fabs(kbarray[i][0])+fabs(kbarray[j][0]);

                    //!第1个决策变量 --> 面积越大越好 -->越大越好打 700->0.8 300->0.4
                    p1 = MIN(0.001*area+0.1, 1);

                    //!第2个决策变量 -->距上次位置越小越好 -->云台移动较小距离 0->1 200->0.1
                    p2 = MAX(-0.0045*pointdis + 1, 0);

                    real_distance = pow(pow(fabs(1452.5455*(exp(-0.1002 * (kbarray[i][1]-2)) - exp(-0.1002 * (kbarray[j][1]-2))) + 1.1814 * (kbarray[i][1]-kbarray[j][1])) , 0.5)*pow(length,2)/32+pow(distance,2) , 0.5);

                    if(length < 48 || (length<60 && ydis<25))
                        test_big_small = (kbarray[i][6]+kbarray[j][6])/real_distance;
                    else
                        test_big_small = 1.1*(kbarray[i][6]+kbarray[j][6])/real_distance;
                    //0.2-0.7是大装甲 0.7-1.2是小装甲
                    //!第3个决策变量 --> 面积距离比 小装甲约是1.5×大装甲  越接近其各自的最优值 越有可能是装甲 也越好瞄准
                    p3 = 0;
                    if(armorCategrayNum[0] == 1)// && armor_categray_num[1]<=120)//小装甲
                    {
                        if(test_big_small>=0.7 && test_big_small<=1.4)
                        {
                            if(test_big_small >=0.72 && test_big_small<=1) //0.7->0.25 1->1
                                p3 = 2.5 * test_big_small - 1.5;
                            else//1->1 1.4->0.5
                                p3 = -1.25 * test_big_small + 2.25;
                        }
                        else
                            p3 = 0;
                    }
                    else if(armorCategrayNum[0] == 2)// && armor_categray_num[1]<=120)//大装甲
                    {
                        if(test_big_small>=0.3 && test_big_small<0.7)
                        {
                            if(test_big_small >=0.35 && test_big_small<=0.6) //0.6->1 0.4->0.6
                                p3 = 2 * test_big_small - 0.2;
                            else//0.6->1 0.7->0.75
                                p3 = -2.5 * test_big_small + 2.5;
                        }
                        else
                            p3 = 0;
                    }
                    else
                    {
                        if(test_big_small>=0.7 && test_big_small<=1.4)
                        {
                            if(test_big_small >=0.7 && test_big_small<=1) //0.7->0.25 1->1
                                p3 = 2.5 * test_big_small - 1.5;
                            else//1->1 1.4->0.5
                                p3 = -1.25 * test_big_small + 2.25;
                        }
                        else if(test_big_small>=0.3 && test_big_small<0.7)
                        {
                            if(test_big_small >=0.35 && test_big_small<=0.6) //0.6->1 0.4->0.6
                                p3 = 2 * test_big_small - 0.2;
                            else//0.6->1 0.7->0.75
                                p3 = -2.5 * test_big_small + 2.5;
                        }
                        else
                            p3 = 0;
                    }


                    if(xdis != 0)
                        y_divide_x_dis = ydis/xdis;
                    else
                        y_divide_x_dis = 0;

                    //!第4个决策变量 兩個裝甲片中心點的x差值越大,ydis越小 越容易瞄準
                    if(xdis >= 3)
                    {
                        if(test_big_small>=0.3 && test_big_small<0.7)//大装甲
                        {
                            if(y_divide_x_dis >= 0.25)
                                p4 = 0;
                            else if(y_divide_x_dis <= 0.12)
                                p4 = 1;
                            else
                                p4 = -7.692*y_divide_x_dis + 1.923;

                        }
                        if(test_big_small>=0.7 && test_big_small<=1.35)//小装甲
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
                    if(distanceLast == 0)
                        p5 = 1;
                    else
                        p5 = MIN(distanceLast/real_distance, real_distance/distanceLast);//distance_dif

                    //!第6个决策变量 如果两个灯柱面积相似度高 那就是它惹。。 1->1 0.25->0
                    p6 = 4/3 *areasimilar - 1/3;

                    dif_angle = fabs(kbarray[i][0])+fabs(kbarray[j][0]);
                    p7 = -0.125*dif_angle + 1;

                    dis_center = pow(pow((xn-winWidth/2), 2) + pow((yn-winHeight/2), 2), 0.5);//距中心点距离
                    p8 = (400 - dis_center)/400;

                    div_y_length = ydis/length;
                    div_x_length = xdis/length;

                    f = 0.5*p1 + 4*p2 + 2*p3 + 2*p4 + 2*p5 + p6 + p7 + 0.8*p8;
                    if (test_big_small<1.7&&test_big_small>=0.7) {
                       f *= 1.4;
                    }

                  /*  double point_small_x = kbarray[i][3] <= kbarray[j][3]? kbarray[i][3]: kbarray[j][3];
                    double point_big_x = kbarray[i][3] > kbarray[j][3]? kbarray[i][3]: kbarray[j][3];
                    double point_small_y = kbarray[i][4] <= kbarray[j][4]? kbarray[i][4]: kbarray[j][4];
                    double point_big_y = kbarray[i][4] > kbarray[j][4]? kbarray[i][4]: kbarray[j][4];
                    double diff_x_point = (point_small_x - rect_2d[0].x) / (point_big_x - rect_2d[2].x);
                    double diff_y_point = (point_small_x - rect_2d[0].y) / (point_big_x - rect_2d[2].y);*/

                    double length_max = MAX(kbarray[i][1], kbarray[j][1]);
                    double length_min = MIN(kbarray[i][1], kbarray[j][1]);
                    double xdis_small = xdis - kbarray[i][5]/2 - kbarray[j][5]/2;

                    if (areasimilar<0.2){}//cout<<"formal1"<<endl;}
                    else if(div_x_length > 2.1){}//cout<<"formal2"<<endl;}
                    else if(div_x_length+div_y_length > 2.4){}//cout<<"formal3"<<endl;}
                    else if(div_x_length < 0.1){}//cout<<"formal4"<<endl;}
                    else if(div_y_length > 1.1){}//cout<<"formal5"<<endl;}
                    else if(length < 26){}//cout<<"formal6"<<endl;}
                    //else if(xdis < 15){}
                    //else if(length >=46 && !(fabs(kbarray[i][0]-kbarray[j][0])<=5 && kbarray[j][0] * kbarray[i][0] > -8)){cout<<"formal7"<<endl;}
                    //else if(length <=40 && test_big_small >0.3 && test_big_small <0.7 && !(fabs(kbarray[i][0]-kbarray[j][0])<=4 && kbarray[j][0] * kbarray[i][0] > -6)){cout<<"formal7"<<endl;}
                    else if(length < 40 && !(fabs(kbarray[i][0]-kbarray[j][0])<=5 && kbarray[j][0] * kbarray[i][0] > -8)){}
                    //else if (middle == true){}
                    else if(distance/area > 1){}//cout<<"formal8"<<endl;}
                    else if(length_min/length_max<0.45 && length_max>20){}//cout<<"formal9"<<endl;}
                    else if(length_min/length_max<0.35){}//cout<<"formal10"<<endl;}
                    else if(test_big_small < 0.3 || test_big_small > 1.7){}//cout<<"formal11"<<endl;}

                    else if(test_big_small >= 0.3 && test_big_small <0.87 && div_x_length>1.03 && div_x_length<1.23 && length < 31){}//cout<<"formal12"<<endl;}
                    else if(test_big_small >= 0.3 && test_big_small <0.86 && div_x_length>0.8 && div_x_length<1.35 && length < 42){}//cout<<"formal12"<<endl;}
                    else if(((test_big_small >= 0.3 && test_big_small <0.7) || armorCategrayNum[0] == 2) && length < 45){}//cout<<"formal12"<<endl;}
                    //else if(armor_categray_num[0] == 1 && test_big_small > 0.3 && test_big_small < 0.7){cout<<"demo1"<<endl;}
                    //else if(armor_categray_num[0] == 2 && test_big_small >= 0.7 && test_big_small < 1.7){cout<<"demo2"<<endl;}
                    else if(ydis/xdis > 0.62){}//cout<<"demo4"<<endl;}

                    else if((armorCategrayNum[0] == 1 || (armorCategrayNum[0] == 0 && armorCategrayNum[1] <3)) && div_x_length>1.38 && length >= 80 ){}//cout<<"test1"<<endl;}
                    else if((armorCategrayNum[0] == 1 || (armorCategrayNum[0] == 0 && armorCategrayNum[1] <3)) && div_x_length>1.35 && length < 80 && length >=65){}//cout<<"test2"<<endl;}
                    else if((armorCategrayNum[0] == 1 || (armorCategrayNum[0] == 0 && armorCategrayNum[1] <3)) && div_x_length>1.32 && length >= 50 && length < 65){}//cout<<"test3"<<endl;}
                    else if((armorCategrayNum[0] == 1 || (armorCategrayNum[0] == 0 && armorCategrayNum[1] <3)) && div_x_length>1.28 && length >=46 && length < 50){}//cout<<"test4"<<endl;}//1.13
                    else if((armorCategrayNum[0] == 1 || (armorCategrayNum[0] == 0 && armorCategrayNum[1] <3)) && div_x_length>1.25 && length >=42 && length < 46){}//cout<<"test5"<<endl;}
                    else if((armorCategrayNum[0] == 1 || (armorCategrayNum[0] == 0 && armorCategrayNum[1] <3)) && div_x_length>1.2 && length >=38 && length <42){}//cout<<"test6"<<endl;}
                    //else if(armor_categray_num[0] == 1 && div_x_length>1.07 && length >= 33.5 && length < 35 ){cout<<"test16"<<endl;}
                    else if((armorCategrayNum[0] == 1 || (armorCategrayNum[0] == 0 && armorCategrayNum[1] <3)) && div_x_length>1.25 && length < 38 ){}//cout<<"test7"<<endl;}
                    else if((armorCategrayNum[0] == 1 || (armorCategrayNum[0] == 0 && armorCategrayNum[1] <3)) && div_x_length>1.05 &&length_min/length_max >0.9 && length < 38 ){}//cout<<"test8"<<endl;}
                    //else if(armor_categray_num[0] == 1 && min(kbarray[i][1], kbarray[j][1]) <18.5 && div_x_length >1){}

                    else if((armorCategrayNum[0] == 1 || (armorCategrayNum[0] == 0 && armorCategrayNum[1] <3)) && div_x_length>1.32 && div_y_length < 0.05 && length >= 80 ){}//cout<<"test9"<<endl;}
                    else if((armorCategrayNum[0] == 1 || (armorCategrayNum[0] == 0 && armorCategrayNum[1] <3)) && div_x_length>1.27 && div_y_length < 0.05 && length < 80 && length >=65){}//cout<<"test10"<<endl;}
                    else if((armorCategrayNum[0] == 1 || (armorCategrayNum[0] == 0 && armorCategrayNum[1] <3)) && div_x_length>1.24 && div_y_length < 0.05 && length >= 50 && length < 65){}//cout<<"test11"<<endl;}
                    else if((armorCategrayNum[0] == 1 || (armorCategrayNum[0] == 0 && armorCategrayNum[1] <3)) && div_x_length>1.22 && div_y_length < 0.05 && length >= 45 && length < 50){}//cout<<"test12"<<endl;}
                    else if((armorCategrayNum[0] == 1 || (armorCategrayNum[0] == 0 && armorCategrayNum[1] <3)) && div_x_length>1.2 && div_y_length < 0.05 && length >= 42&& length < 45 ){}//cout<<"test13"<<endl;}

                    else if((armorCategrayNum[0] == 1 || (armorCategrayNum[0] == 0 && armorCategrayNum[1] <3)) && div_x_length + div_y_length > 1.75 && length >= 89){}//cout<<"test14"<<endl;}
                    else if((armorCategrayNum[0] == 1 || (armorCategrayNum[0] == 0 && armorCategrayNum[1] <3)) && div_x_length + div_y_length > 1.67 && length < 89 && length >= 80){}//cout<<"test15"<<endl;}
                    else if((armorCategrayNum[0] == 1 || (armorCategrayNum[0] == 0 && armorCategrayNum[1] <3)) && div_x_length + div_y_length > 1.62 && length < 80 && length >=65){}//cout<<"test16"<<endl;}
                    else if((armorCategrayNum[0] == 1 || (armorCategrayNum[0] == 0 && armorCategrayNum[1] <3)) && div_x_length + div_y_length > 1.57 && length < 65 && length >=50){}//cout<<"test17"<<endl;}
                    else if((armorCategrayNum[0] == 1 || (armorCategrayNum[0] == 0 && armorCategrayNum[1] <3)) && div_x_length + div_y_length > 1.52 && length > 42 && length < 50){}//cout<<"test18"<<endl;}
                    else if((armorCategrayNum[0] == 1 || (armorCategrayNum[0] == 0 && armorCategrayNum[1] <3)) && div_x_length + div_y_length > 1.48 &&length >= 35 && length <= 42){}//cout<<"test19"<<endl;}
                    else if((armorCategrayNum[0] == 1 || (armorCategrayNum[0] == 0 && armorCategrayNum[1] <3)) && div_x_length + div_y_length > 1.4 && length < 35){}//cout<<"test20"<<endl;}


                    else if((y_divide_x_dis>0.8 || xdis==0) && length <= 50){}//cout<<"demo1"<<endl;}
                    else if((y_divide_x_dis>0.35 || xdis==0) && armorCategrayNum[0] == 2 && length <= 50){}//cout<<"demo2"<<endl;}
                    else if(div_x_length<0.2 && div_y_length>0.5){}//cout<<"demo3"<<endl;}

                    else if(armorCategrayNum[0] == 2 && length>60 && xdis+ydis<64){}//cout<<"tttt1"<<endl;}
                    else if(armorCategrayNum[0] == 2 && length>50 && length<=60 && xdis+ydis<60){}//cout<<"tttt1"<<endl;}
                    else if(armorCategrayNum[0] == 2 && length>46 && length<=50 && xdis+ydis<56){}//cout<<"tttt1"<<endl;}
                    else if(armorCategrayNum[0] == 2 && length>43 && length<=46 && xdis+ydis<54){}//cout<<"tttt2"<<endl;}
                    else if(armorCategrayNum[0] == 2 && length>40 && length<=43 && xdis+ydis<52){}//cout<<"tttt20"<<endl;}
                    else if(armorCategrayNum[0] == 2 && length>35 && length <=40 && xdis+ydis<50){}//cout<<"ttttttt3"<<endl;}
                    else if(armorCategrayNum[0] == 2 && length<=35 && xdis+ydis<46){}//cout<<"ttttttt4"<<endl;}
                    else if(armorCategrayNum[0] == 2 && ydis/xdis >0.16){}//cout<<"ttttttt5"<<endl;}

                    //else if(armor_categray_num[0] == 1 && length<60 && length>=50 && xdis+ydis>68){cout<<"ttttttt16"<<endl;}
                    else if(armorCategrayNum[0] == 1 && length<60 && length>=50 && xdis+ydis>63){}//cout<<"ttttttt26"<<endl;}
                    else if(armorCategrayNum[0] == 1 && length<50 && length>=43 && xdis+ydis>56){}//cout<<"ttttttt36"<<endl;}
                    else if(armorCategrayNum[0] == 1 && length<43 && length>=40 && xdis+ydis>49){}//cout<<"ttttttt7"<<endl;}
                    else if(armorCategrayNum[0] == 1 && length<40 && length>=35 && xdis+ydis>45){}//cout<<"ttttttt8"<<endl;}
                    else if(armorCategrayNum[0] == 1 && length<35 && xdis+ydis>43){}//cout<<"ttttttt9"<<endl;}

                    //else if(test_big_small >=0.67 && test_big_small < 0.7){cout<<"ttttttt10"<<endl;}
                    else if(test_big_small >=0.7 && test_big_small < 0.75 ){}//cout<<"ttttttt11"<<endl;}
                    else if(test_big_small >=0.7 && test_big_small < 0.87 && length > 70){}//cout<<"ttttttt11"<<endl;}
                    else if(test_big_small<0.7 && div_x_length<1.2 && div_y_length<0.1){}//cout<<"ttttttt12"<<endl;}
                    //else if(test_big_small<0.7 && length < 47 && div_x_length+div_y_length<1.7){}

                    //else if(test_big_small>0.3 && test_big_small <0.7 && lastsmalldis < 20*pow(length,0.5) && pX!=0){cout<<"ttttttt13"<<endl;}
                   // else if(pointdis<0.8 * distance && pointdis<1.2 * distance ){}
                    else if(xdis/area < 0.07){}
                    else if(xdis < 12){}
                    else if(xdis_small <10){}

                    else if(f < 4){}
                    else
                    {
                        drawContours(rgbPic, hull, i, Scalar(255,255,0), -1,LINE_8,noArray(),INT_MAX);
                        drawContours(rgbPic, hull, j, Scalar(255,255,0), -1,LINE_8,noArray(),INT_MAX);
                        //cout<<"xdis_small"<<xdis_small<<endl;
                        //cout<<"div_y_length/areasimilar  "<<div_y_length/areasimilar<<endl;
                        //cout<<"div_x_length"<<div_x_length <<"  div_ylength  "<< div_y_length<<endl;
                        //cout<<"xdis/area"<<xdis/area<<endl;
                        //cout<<"y_divide_x_dis"<<y_divide_x_dis<<endl;
                        //cout<<"length"<<length<<"   length_diff"<<length_diff<<endl;
                        //cout<<"dis_last_distance"<<dis_last_distance<<endl;

                        //cout<<"angle"<<kbarray[i][0]<<"  "<<kbarray[j][0]<<endl;
                        //cout<<"distance"<<distance<<"  area"<<area<<"  / "<<distance/area<<endl;
                        //cout<<"pointdis"<<pointdis<<endl;
                        //cout<<"ydis"<<ydis<<"  xdis"<<xdis<<"  / "<<ydis/xdis<<endl;
                        //cout<<"test_big_small"<<test_big_small<<"   realdistance "<<real_distance<<" distance"<<distance<<endl;
                        //cout<<"area"<<area<<endl;
                        //cout<<"areasimilar"<<areasimilar<<endl;
                        //cout<<"real_distance"<<real_distance<<"  distance"<<distance<<endl;
                        //cout<<"f"<<f<<"  p1:"<<p1<<"  p2:"<<p2<<"  p3:"<<p3<<"  p4:"<<p4<<"  p5:"<<p5<<"  p6:"<<p6<<"  p8:"<<p8<<endl;
                        point_flag demo;
                        demo.flag = f;
                        demo.test_big_small = test_big_small;
                        demo.point_a = i;
                        demo.point_b = j;
                        demo.distance_current = real_distance;
                        //demo.length_current = length;
                        //demo.x_dis_current = xdis;
                        //demo.area_current = area;
                        //demo.point_a_center = kbarray[i][3];
                        //demo.point_b_center = kbarray[j][3];
                        mylist.push_back(demo);

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
bool sort_list(
                vector<point_flag>& mylist,
                vector<vector<double> >& kbarray,
                Point& picCenter,
                vector<vector<Point> >& hull,
                double& yLengthLast,
                double distanceLast,
                vector<double>distanceLastArray,
                int* armorCategrayNum
              )
{
    if(mylist.empty())
        return true;//需要再追踪

    sort(mylist.begin(), mylist.end(),comp);
    point_flag point_flag_last = mylist[0];
    int px_current = (kbarray[point_flag_last.point_a][3] + kbarray[point_flag_last.point_b][3]) / 2;
    int py_current = (kbarray[point_flag_last.point_a][4] + kbarray[point_flag_last.point_b][4]) / 2;

    //判断大小装甲
    if(armorCategrayNum[1]>150)
    {
        armorCategrayNum[0] = 0;
        armorCategrayNum[1] = 0;
    }

    if(point_flag_last.test_big_small >= 0.7 && point_flag_last.test_big_small <= 1.7 ) //小装甲
    {
      if(armorCategrayNum[0] != 2)
      {
          armorCategrayNum[1] ++;
      }
      else
      {
          armorCategrayNum[1] = 0;
      }
      armorCategrayNum[0] = 1;
    }

    else if(point_flag_last.test_big_small > 0.3 && point_flag_last.test_big_small < 0.7 ) //大装甲
    {
        if(armorCategrayNum[0] != 1)
        {
            armorCategrayNum[1] ++;
        }
        else
        {
            armorCategrayNum[1] = 0;
        }
        armorCategrayNum[0] = 2;
    }

    if(armorCategrayNum[1]>=2)
    {
        if(distanceLastArray.size() == 0 )
        {
            distanceLastArray.push_back(mylist[0].distance_current);
            distanceLast = mylist[0].distance_current;
        }
        if(distanceLastArray.size() == 1 )
        {
            distanceLastArray.push_back(mylist[0].distance_current);
            distanceLast = (distanceLastArray[0] + mylist[0].distance_current)/2;
        }
        if(distanceLastArray.size() == 2)
        {
            distanceLastArray.push_back(mylist[0].distance_current);
            distanceLast = (distanceLastArray[0] + distanceLastArray[1] + mylist[0].distance_current)/3;
        }
        if(distanceLastArray.size() >= 3)
        {
            distanceLastArray.erase(distanceLastArray.begin());
            distanceLastArray.push_back(mylist[0].distance_current);
            distanceLast = (distanceLastArray[0] + distanceLastArray[1] + mylist[0].distance_current)/3;
        }
    }
    else
    {
        distanceLastArray.clear();
        distanceLast = 0;
    }

    //y_length_last = (kbarray[point_flag_last.point_a][1] + kbarray[point_flag_last.point_b][1]) / 2;

    int point_small, point_big;
    point_small = kbarray[mylist[0].point_a][3] <= kbarray[mylist[0].point_b][3]? mylist[0].point_a: mylist[0].point_b;
    point_big = kbarray[mylist[0].point_a][3] > kbarray[mylist[0].point_b][3]? mylist[0].point_a: mylist[0].point_b;

    picCenter.x = px_current;
    picCenter.y = py_current;

    return false;
}

#endif // CONTOURS_H_INCLUDED
