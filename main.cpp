#include <stdio.h>
#include <iostream>
#include "xiApiPlusOcv.hpp"
#include <opencv2/opencv.hpp>

cv::Point2f getcenter(const cv::Mat& _img, int _threshold){
    cv::Mat img;
    if(_img.channels() == 3)
        cv::cvtColor(_img, img, cv::COLOR_BGR2GRAY);
    else
        img = _img.clone();

    cv::threshold(img, img, _threshold, 255, cv::THRESH_BINARY);

    //find contours
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours( img, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
    //cal moments
    std::vector<cv::Moments> mu(contours.size() );
    for( int i = 0; i < contours.size(); i++ )
    {
        mu[i] = cv::moments( contours[i], false );
    }
    //cal center
    std::vector<cv::Point2f> mc( contours.size() );
    for( int i = 0; i < contours.size(); i++ )
    {
        mc[i] = cv::Point2d( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );
    }

    return mc[0];
}

float cal_distance(cv::Point2f _point1, cv::Point2f _point2, cv::Mat R, cv::Mat shift, float _scale){
    cv::Mat point1 = cv::Mat::zeros(3, 1, CV_32F);
    point1.at<float>(0) = (_point1.x * _scale);
    point1.at<float>(1) = (_point1.y * _scale);
    point1.at<float>(2) = 0;
    cv::Mat point2 = cv::Mat::zeros(3, 1, CV_32F);
    point2.at<float>(0) = (_point2.x * _scale);
    point2.at<float>(1) = (_point2.y * _scale);
    point2.at<float>(2) = 0;

    cv::Mat point1_in_2;
//    cv::transpose()
    point1_in_2 = R * point1  + shift;

    float distance = std::sqrt(std::pow(point1_in_2.at<float>(0) - point2.at<float>(0), 2) +
                               std::pow(point1_in_2.at<float>(1) - point2.at<float>(1), 2) +
                               std::pow(point1_in_2.at<float>(2) - point2.at<float>(2), 2));
    return distance;
}


int main(int argc, char* argv[])
{
    ///////////////////////////////////process
    ///TODO
    cv::Mat R = cv::Mat::eye(cv::Size(3, 3), CV_32F);
    cv::Mat T = cv::Mat::zeros(3, 1, CV_32F);

    R.at<float>(0, 0) = 1; R.at<float>(0, 1) = 1; R.at<float>(0, 2) = 1;
    R.at<float>(1, 0) = 1; R.at<float>(1, 1) = 1; R.at<float>(1, 2) = 1;
    R.at<float>(2, 0) = 1; R.at<float>(2, 1) = 1; R.at<float>(2, 2) = 1;

    T.at<float>(0) = 0; T.at<float>(1) = 0; T.at<float>(2) = 0;

    std::vector<cv::Mat> left_imgs;
    std::vector<cv::Mat> right_imgs;
    try
    {
        xiAPIplusCameraOcv cam1, cam2;

        // Retrieving a handle to the camera device
        printf("Opening first camera...\n");
        std::string str1("21706750");
        std::string str2("21705550");
        cam1.OpenBySN((char*)str1.c_str());
        cam2.OpenBySN((char*)str2.c_str());

        cam1.DisableAutoExposureAutoGain();
        cam2.DisableAutoExposureAutoGain();

        // Create an inverse LUT  (inverts image pixels - min -> max, max -> min)
        int maxIndex = cam1.GetLookUpTableIndex_Maximum();
        int minIndex = cam1.GetLookUpTableIndex_Minimum();
        int actualIndex = cam1.GetLookUpTableIndex();
        int incrementIndex = cam1.GetLookUpTableIndex_Increment();
        int minValue = cam1.GetLookUpTableValue_Minimum();
        int maxValue = cam1.GetLookUpTableValue_Maximum();
        int actualValue = cam1.GetLookUpTableValue();
        int incrementValue = cam1.GetLookUpTableValue_Increment();

        std::cout << "maximal LUT index:" << maxIndex << std::endl;
        std::cout << "minimal LUT index:" << minIndex << std::endl;
        std::cout << "actual LUT index:" << actualIndex << std::endl;
        std::cout << "actual LUT index increment:" << incrementIndex << std::endl;

        std::cout << "maximal LUT value:" << maxValue << std::endl;
        std::cout << "minimal LUT value:" << minValue << std::endl;
        std::cout << "actual LUT value:" << minValue << std::endl;
        std::cout << "actual LUT value increment:" << incrementValue << std::endl;

       //cvWaitKey();

        for (int i = 0; i < maxIndex; i++){
            cam1.SetLookUpTableIndex(i);
            cam1.SetLookUpTableValue(maxValue - i);
        }
        cam1.SetLookUpTableIndex(800);
        cam1.SetLookUpTableValue(4);
        cam1.EnableLookUpTable();
        //cam.DisableLookUpTable();
        //Set exposure
        cam1.SetExposureTime(40000); //10000 us = 10 ms
        // Note: The default parameters of each camera might be different in different API versions

        for (int i = 0; i < maxIndex; i++){
            cam2.SetLookUpTableIndex(i);
            cam2.SetLookUpTableValue(maxValue - i);
        }
        cam2.SetLookUpTableIndex(800);
        cam2.SetLookUpTableValue(4);
        cam2.EnableLookUpTable();
        //cam.DisableLookUpTable();
        //Set exposure
        cam2.SetExposureTime(10000); //10000 us = 10 ms
        // Note: The default parameters of each camera might be different in different API versions

        printf("Starting acquisition...\n");
        cam1.StartAcquisition();
        cam2.StartAcquisition();

        printf("First pixel value \n");
        #define EXPECTED_IMAGES 10000
        for (int images=0;images < EXPECTED_IMAGES;images++)
        {
            cv::Mat cv_mat_image1 = cam1.GetNextImageOcvMat();
            cv::Mat cv_mat_image2 = cam2.GetNextImageOcvMat();
            cv::imshow("Image from camera1",cv_mat_image1);
            cv::imshow("Image from camera2",cv_mat_image2);
            char key = cvWaitKey(0);
            std::cout << (char)key <<std::endl;
            if(key == 's'){
                left_imgs.push_back(cv_mat_image1.clone());
                right_imgs.push_back(cv_mat_image2.clone());
            }
            if(key == 'q')
                break;
        }

        cam1.StopAcquisition();
        cam1.Close();
        cam2.StopAcquisition();
        cam2.Close();
        printf("Done\n");

        cvWaitKey(500);
    }
    catch(xiAPIplus_Exception& exp){
        printf("Error:\n");
        exp.PrintError();
        cvWaitKey(2000);
    }

    std::vector<cv::Point2f> left_points;
    std::vector<cv::Point2f> right_points;
    for(size_t i = 0; i < left_imgs.size(); i++){
        cv::Mat left_temp = left_imgs[i];
        cv::Mat right_temp = right_imgs[i];

        cv::Point2f left_point = getcenter(left_temp, 100);
        cv::Point2f right_point = getcenter(right_temp, 100);

        left_points.push_back(left_point);
        right_points.push_back(right_point);
    }

    cv::Mat result = cv::Mat::zeros(left_points.size(), 1, CV_32F);
    for(size_t i = 0; i < left_points.size(); i++){
        result.at<float>(i) = cal_distance(left_points[i], right_points[i], R, T, 1);
    }
    float distance = cv::mean(result)[0];

    std::cout << "distance is :: " << distance <<std::endl;

    return 0;
}
