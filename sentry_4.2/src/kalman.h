/******************************************************************
*  Copyright (C), 2018, Lesley
*  File name : fittingPre.h
*  Author : Lesley
*  Description : Prediction using Kalman filter. This program has 
		 some problems.
******************************************************************/

#ifndef KALMAN_H_INCLUDED
#define KALMAN_H_INCLUDED

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
#include "contours.h"
using namespace std;
using namespace cv;

const int stateNum=4;//x,y,detaX,detaY
const int measureNum=2;//x,y
KalmanFilter KF(stateNum, measureNum, 0);

//measurement(x,y)初始测量值
Mat measurement= Mat::zeros(measureNum, 1, CV_32F);;
//Mat processNoise(stateNum, 1, CV_32F);
//Mat state(stateNum, 1, CV_32F);


double anti_range = 0.5;

void Kalman_init(Point& Center)
{
    //!1.kalman filter setup
    //转移矩阵A
    KF.transitionMatrix = (Mat_<float>(4, 4) <<
            1,0,1,0,
            0,1,0,1,
            0,0,1,0,
            0,0,0,1 );
    //!< measurement matrix (H)
    setIdentity(KF.measurementMatrix);

    //!< measurement noise covariance matrix (R)
    setIdentity(KF.measurementNoiseCov, Scalar::all(1e-3)); //quicker regression

    //!< process noise covariance matrix (Q)
    setIdentity(KF.processNoiseCov, Scalar::all(5e-5)); //slower regression
    //!< priori error estimate covariance matrix (P'(k)):P'(k)=A*P(k-1)*At + Q)  A代表F: transitionMatrix
    //预测估计协方差矩阵;
    setIdentity(KF.errorCovPost, Scalar::all(1));
    //!< corrected state (x(k)): x(k)=x'(k)+K(k)*(z(k)-H*x'(k))
    randn(KF.statePost, Scalar::all(0), Scalar::all(0.1));//初始状态值
    //randn(KF.statePre, Scalar::all(0), Scalar::all(0.1));
    //KF.statePost = (Mat_<float>(4, 1) << Center.x, Center.y, 0, 0);//Ensure beginner is default value
}

void Kalman_prediction(Mat& pic, Point& Center)
{
    //!2.kalman prediction
    //由上一刻状态值得到此刻的先验状态估计值， 得到先验误差协方差
    //KF.statePre = KF.predict();
    Point predictPt = Point( (int)KF.statePre.at<float>(0), (int)KF.statePre.at<float>(1));//预测值（x',y'）

    //!3.update measurement
    measurement.at<float>(0)= Center.x;
    measurement.at<float>(1) = Center.y;

    circle(pic, predictPt, 5, CV_RGB(0,255,255),2);//predict point

    //!4.update
    KF.statePost = KF.correct(measurement);
    //Center = Point( (int)KF.statePost.at<float>(0), (int)KF.statePost.at<float>(1));
    Point merPt = Point( (int)KF.statePost.at<float>(0), (int)KF.statePost.at<float>(1));
    //circle(pic, merPt, 5, CV_RGB(0,255,0),2);//predict point
    Point anti_kalmanPoint;
    if((px + anti_range*(px - merPt.x)) <= winWidth || (px + anti_range*(px - merPt.x))>=0)//Prevent Anti-kal out of Mat
    {
        if(fabs(px - merPt.x) > 3)//When points are closed, no Anti-kalman to reduce shaking
            anti_kalmanPoint.x = px + anti_range*(px - merPt.x);
        else
            anti_kalmanPoint.x = px;
    }
    else
    {
        anti_kalmanPoint.x = px;
    }
    if((py + anti_range*(py - merPt.y)) <= winHeight || (py + anti_range*(py - merPt.y))>=0)//Prevent Anti-kal out of Mat
    {
        if(fabs(py - merPt.y) > 3)//When points are closed, no Anti-kalman to reduce shaking
            anti_kalmanPoint.y = py + anti_range*(py - merPt.y);
        else
            anti_kalmanPoint.y = py;
    }
    else
    {
        anti_kalmanPoint.y = py;
    }
    circle(pic, merPt, 5, CV_RGB(0,255,0),2);//predict point
}
/*
1,0,0,1,0,0,
            0,1,0,0,1,0,
            0,0,1,0,0,1,
            0,0,0,1,0,0,
            0,0,0,0,1,0,
            0,0,0,0,0,1);*/

#endif // KALMAN_H_INCLUDED
