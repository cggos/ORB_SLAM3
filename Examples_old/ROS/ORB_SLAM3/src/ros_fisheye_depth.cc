/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM):mpSLAM(pSLAM){}

    void GrabFisheyeD(const sensor_msgs::ImageConstPtr& msgFisheye,const sensor_msgs::ImageConstPtr& msgD);

    ORB_SLAM3::System* mpSLAM;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "FisheyeDepth");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 FisheyeDepth path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::FisheyeD,true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> fisheye_sub(nh, "/T265/fisheye1/image_raw", 1000);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/D435I/depth/image_rect_raw", 1000);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), fisheye_sub, depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabFisheyeD,&igb,_1,_2));

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabFisheyeD(const sensor_msgs::ImageConstPtr& msgFisheye,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrFisheye;
    try
    {
        cv_ptrFisheye = cv_bridge::toCvShare(msgFisheye);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // cv::Mat mat_depth8;
    // {
    //     double min, max;
    //     cv::minMaxLoc(cv_ptrD->image, &min, &max);
    //     cv_ptrD->image.convertTo(mat_depth8, CV_8UC1, 255.0/(max-min), -255.0*min/(max-min));
    // }

    // cv::imshow("fisheye", cv_ptrFisheye->image);
    // cv::imshow("depth", mat_depth8);
    // cv::waitKey(1);

    mpSLAM->TrackRGBD(cv_ptrFisheye->image,cv_ptrD->image,cv_ptrFisheye->header.stamp.toSec());
}


