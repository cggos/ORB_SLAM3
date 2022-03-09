/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
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
#include<thread>
#include<mutex>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include<sensor_msgs/Imu.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

using namespace std;

typedef std::pair<sensor_msgs::ImageConstPtr, sensor_msgs::ImageConstPtr> t_pair_rgbd; // <rgb, depth>

class ImuGrabber
{
public:
    ImuGrabber(){};

    void GrabImu(const sensor_msgs::ImuConstPtr &imu_msg);

    void GrabImuGyr(const sensor_msgs::ImuConstPtr &gyr_msg);
    void GrabImuAcc(const sensor_msgs::ImuConstPtr &acc_msg);

    queue<sensor_msgs::ImuConstPtr> imuBuf;
    std::mutex mBufMutex;

    deque<sensor_msgs::ImuConstPtr> gyrBuf;
    std::mutex mBufMutexGyr;

    const double acc_th_t = 1 / 250.0; // Acc FPS: 250
};

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM, ImuGrabber *pImuGb) : mpSLAM(pSLAM), mpImuGb(pImuGb) {}

    void GetRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD, cv::Mat &imRGB, cv::Mat &imD);

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    void SyncWithImu();

    queue<t_pair_rgbd> imgBuf;
    std::mutex mBufMutex;

    ImuGrabber *mpImuGb;

    ORB_SLAM3::System* mpSLAM;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 RGBD path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::IMU_RGBD,true);

    ImuGrabber imugb;
    ImageGrabber igb(&SLAM, &imugb);

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 100);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "camera/depth_registered/image_raw", 100);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));

    // ros::Subscriber sub_imu = nh.subscribe("/imu", 1000, &ImuGrabber::GrabImu, &imugb);

    ros::Subscriber sub_imu_gyr = nh.subscribe("/imu_gyr", 1000, &ImuGrabber::GrabImuGyr, &imugb);
    ros::Subscriber sub_imu_acc = nh.subscribe("/imu_acc", 1000, &ImuGrabber::GrabImuAcc, &imugb);

    std::thread sync_thread(&ImageGrabber::SyncWithImu,&igb);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GetRGBD(
    const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD, cv::Mat &imRGB, cv::Mat &imD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
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

    imRGB = cv_ptrRGB->image.clone();
    imD = cv_ptrD->image.clone();

    // mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
  mBufMutex.lock();
  if (!imgBuf.empty())
    imgBuf.pop();
  t_pair_rgbd rgbd_pair = std::make_pair(msgRGB, msgD);
  imgBuf.push(rgbd_pair);
  mBufMutex.unlock();
}

void ImageGrabber::SyncWithImu()
{
  while(1)
  {
    cv::Mat imRGB, imD;

    double tIm = 0;
    if (!imgBuf.empty()&&!mpImuGb->imuBuf.empty())
    {
      const t_pair_rgbd &pair_rgbd = imgBuf.front();
      const sensor_msgs::ImageConstPtr& msgRGB = pair_rgbd.first;
      const sensor_msgs::ImageConstPtr& msgD = pair_rgbd.second;

      tIm = msgRGB->header.stamp.toSec();
      if(tIm>mpImuGb->imuBuf.back()->header.stamp.toSec())
          continue;
      {
        this->mBufMutex.lock();
        GetRGBD(msgRGB, msgD, imRGB, imD);
        imgBuf.pop();
        this->mBufMutex.unlock();
      }

      vector<ORB_SLAM3::IMU::Point> vImuMeas;
      mpImuGb->mBufMutex.lock();
      if(!mpImuGb->imuBuf.empty())
      {
        // Load imu measurements from buffer
        vImuMeas.clear();
        while(!mpImuGb->imuBuf.empty() && mpImuGb->imuBuf.front()->header.stamp.toSec()<=tIm)
        {
          double t = mpImuGb->imuBuf.front()->header.stamp.toSec();
          cv::Point3f acc(mpImuGb->imuBuf.front()->linear_acceleration.x, mpImuGb->imuBuf.front()->linear_acceleration.y, mpImuGb->imuBuf.front()->linear_acceleration.z);
          cv::Point3f gyr(mpImuGb->imuBuf.front()->angular_velocity.x, mpImuGb->imuBuf.front()->angular_velocity.y, mpImuGb->imuBuf.front()->angular_velocity.z);
          vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc,gyr,t));
          mpImuGb->imuBuf.pop();
        }
      }
      mpImuGb->mBufMutex.unlock();
    //   if(mbClahe)
    //     mClahe->apply(im,im);

    //   mpSLAM->TrackMonocular(im,tIm,vImuMeas);
      mpSLAM->TrackRGBD(imRGB, imD, tIm, vImuMeas);
    }

    std::chrono::milliseconds tSleep(1);
    std::this_thread::sleep_for(tSleep);
  }
}

void ImuGrabber::GrabImu(const sensor_msgs::ImuConstPtr &imu_msg)
{
  mBufMutex.lock();
  imuBuf.push(imu_msg);
  mBufMutex.unlock();
  std::cout << __FUNCTION__ << std::endl;
}

void ImuGrabber::GrabImuGyr(const sensor_msgs::ImuConstPtr &gyr_msg)
{
  mBufMutexGyr.lock();
  gyrBuf.push_back(gyr_msg);
  mBufMutexGyr.unlock();
}

void ImuGrabber::GrabImuAcc(const sensor_msgs::ImuConstPtr &acc_msg)
{
  sensor_msgs::ImuPtr imu_ptr(new sensor_msgs::Imu());
  imu_ptr->header = acc_msg->header;

  geometry_msgs::Vector3 vec_gyr;
  double acc_t = acc_msg->header.stamp.toSec();
  while(!gyrBuf.empty()) {
    mBufMutexGyr.lock();

    double gyr_t = gyrBuf.front()->header.stamp.toSec();
    if(gyr_t <= acc_t - acc_th_t) {
      gyrBuf.pop_front();
      break;
    }

    bool flag = false;
    auto gyr_last = gyrBuf.front();
    vec_gyr = gyr_last->angular_velocity;
    for(const auto gyr : gyrBuf) {
      gyr_t = gyr->header.stamp.toSec();
      if(gyr_t > acc_t) { 
        vec_gyr.x = 0.5 * (gyr_last->angular_velocity.x + gyr->angular_velocity.x);
        vec_gyr.y = 0.5 * (gyr_last->angular_velocity.y + gyr->angular_velocity.y);
        vec_gyr.z = 0.5 * (gyr_last->angular_velocity.z + gyr->angular_velocity.z);
        flag = true;
        break;
      }
      gyr_last = gyr;
    }
    if(flag) break;

    mBufMutexGyr.unlock();
  }
  imu_ptr->angular_velocity = vec_gyr;

  GrabImu(imu_ptr);
}
