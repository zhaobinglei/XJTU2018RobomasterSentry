/*****************************************************************
*  Copyright (C), 2018, Lesley
*  File name : adjust_color.h
*  Author : Lesley

*  Description : This programm is used to adjust the lightness
	         or contrast of an image. However, the final 
		 project did not use functions in this header file.
******************************************************************/

#ifndef ADJUST_COLOR_H_INCLUDED
#define ADJUST_COLOR_H_INCLUDED

#include<opencv2/opencv.hpp>
using namespace cv;

/**
*  @Function : gamma_correct(Mat& img, Mat& dst, double gamma)
*  @Description : Gamma correct
*/
void gamma_correct(Mat& img, Mat& dst, double gamma) {
	Mat temp;
	CvMat tmp;
	img.convertTo(temp, CV_32FC1, 1.0 / 255.0, 0.0);
	tmp = temp;
	cvPow(&tmp, &tmp, gamma);
	temp.convertTo(dst, CV_8UC1, 255.0, 0.0);
}

/**
*  @Function : define_filter_contrast_ratio(Mat& img)
*  @Description :Use partial large Cauchy distribution function to 
		increase contrast of grey-scale image.
*/
void define_filter_contrast_ratio(Mat& img){偏大型正态分布函数 230->0.9   80->0.2
    for (int y = 0; y < img.rows; y++)
        for (int x = 0; x < img.cols; x++) {
            if (img.at<uchar>(y, x)< 250)
                img.at<uchar>(y, x) = 0;
            else
                img.at<uchar>(y,x) = 255 * float(1 - exp(-pow(((img.at<uchar>(y, x) -27.38) / (-47.84)), 2)));
        }
}

#endif // ADJUST_COLOR_H_INCLUDED
