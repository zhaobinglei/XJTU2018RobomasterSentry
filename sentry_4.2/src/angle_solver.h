/*****************************************************************
*  Copyright (C), 2018, Lesley
*  File name : angle_solver.h
*  Author : Lesley

*  Description : This programm is used to solve angle and depth in 
		 2D-image through four corresponding points which
		 are in the world coodrdinate system or image 
		 coodrdinate system using solvePnP in OpenCV. 
*  Condition : The cameraMatrix and distCoeff of your camera is 
	       needed to known. 
******************************************************************/

#ifndef ANGLE_SOLVER_H_INCLUDED
#define ANGLE_SOLVER_H_INCLUDED
#include "contours.h"

/**
*  @Function : gamma_correctangle_solver_test(Point2f \
	      image_points_buf[] , Mat& pic, double& depth, \
	      int catgory)
*  @Description : Solve angle and depth of objective according to
		 armor category.
*  @param cameraMatrix Type is Mat, which is the inner parameter 
	  matrix of the camera.
*  @param distCoeffs Type is Mat, which is the radial distortion
	  matrix of the camera.
*/
void angle_solver_test(Point2f image_points_buf[] , Mat& pic, double& depth, int catgory)
{
    // Read points
    vector<Point2f> imagePoints ;
    imagePoints.push_back(image_points_buf[0]);
    imagePoints.push_back(image_points_buf[1]);
    imagePoints.push_back(image_points_buf[2]);
    imagePoints.push_back(image_points_buf[3]);

    vector<Point3f> objectPoints;

    if(catgory == 2)//大装甲
    {
        objectPoints.push_back(Point3f(-12.5, -3, 0.0));
        objectPoints.push_back(Point3f(-12.5, 3, 0.0));
        objectPoints.push_back(Point3f(12.5, 3, 0.0));
        objectPoints.push_back(Point3f(12.5, -3, 0.0));
    }

    else//小装甲 或 没检测出型号
    {
        objectPoints.push_back(Point3f(-7.2, -3, 0.0));
        objectPoints.push_back(Point3f(-7.2, 3, 0.0));
        objectPoints.push_back(Point3f(7.2, 3, 0.0));
        objectPoints.push_back(Point3f(7.2, -3, 0.0));
    }

    //Mat cameraMatrix = (cv::Mat_<double>(3,3) << 799.4882112289697, 0, 335.7201090683797, 0, 798.2745396421427, 244.4640310157802, 0, 0, 1);
    //Mat distCoeffs = (Mat_<double>(1,5) <<-0.4304634395248472, 0.4652451609572892, -0.002170757080254447, -0.001416479077749636, -0.7229891661708902);
    Mat cameraMatrix = (cv::Mat_<double>(3,3) << 890.5354, 0, 310.4096, 0, 892.5205, 259.1173, 0, 0, 1);
    Mat distCoeffs = (Mat_<double>(1,4) <<-0.3353, 0.0222, 0, 0);

    Mat rvec(3,1,cv::DataType<double>::type);
    Mat tvec(3,1,cv::DataType<double>::type);

    solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);

    vector<Point2f> projectedPoints;
    projectPoints(objectPoints, rvec, tvec, cameraMatrix, distCoeffs, projectedPoints);
    bool true_depth = true;
    for(unsigned int i = 0; i < projectedPoints.size(); ++i)
    {
        //cout << "Image point: " << imagePoints[i] << " Projected to " << projectedPoints[i] << endl;
        if(imagePoints[i].x/projectedPoints[i].x>1.2 || imagePoints[i].x/projectedPoints[i].x<0.8 ||imagePoints[i].y/projectedPoints[i].y>1.2 || imagePoints[i].y/projectedPoints[i].y<0.8)
        {
            true_depth = false;
            break;
        }
    }
    if(true_depth)
    {
        Mat_<float> test;
        tvec.convertTo(test, CV_32F);
        depth = test.at<float>(0, 2);
        //cout<<"depth"<<depth<<endl;
     /*   circle(pic,projectedPoints[0],7,Scalar(255,0,0),2);
        circle(pic,projectedPoints[1],7,Scalar(0,255,0),2);
        circle(pic,projectedPoints[2],7,Scalar(0,0,255),2);
        circle(pic,projectedPoints[3],7,Scalar(0,255,255),2);

        circle(pic,imagePoints[0],2,Scalar(255,0,0),-1);
        circle(pic,imagePoints[1],2,Scalar(0,255,0),-1);
        circle(pic,imagePoints[2],2,Scalar(0,0,255),-1);
        circle(pic,imagePoints[3],2,Scalar(0,255,255),-1);*/
    }



}

#endif // ANGLE_SOLVER_H_INCLUDED
