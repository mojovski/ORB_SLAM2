/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>


#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf/transform_broadcaster.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>


#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

using namespace std;
ros::Publisher pub_pose;

//converts cv rot and sv pos to ros pose
geometry_msgs::Pose poseMatrixToMsg(cv::Mat & R, cv::Mat & t);


class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM2::System* mpSLAM;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,&igb);

    //register ouput
    pub_pose = nodeHandler.advertise< geometry_msgs::PoseStamped >("/camera_pose", 5, true);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    mpSLAM->TrackMonocular(cv_ptr->image, "rosimg", cv_ptr->header.stamp.toSec());


    ORB_SLAM2::Tracking* tracker=mpSLAM->getTracker();
    //get last frame
    ORB_SLAM2::Frame frame=tracker->getCurrentFrame();
    cv::Mat cam_center=frame.GetCameraCenter();
    cv::Mat cam_rot=frame.GetRotationInverse(); //TODO: Check what it means: cam in world, or world in cam

    std::cout << "Last Position:" << cam_center << "\n";

    //PUBLISH THE POSE IN ROS
    geometry_msgs::PoseStamped pose_stamped_msg;
    pose_stamped_msg.header = cv_ptr->header; //TODO: Check if header.frame_id = "camera_link"; is required (example in RealSense)
    pose_stamped_msg.pose = poseMatrixToMsg(cam_rot, cam_center);
    pub_pose.publish(pose_stamped_msg);
}




geometry_msgs::Pose poseMatrixToMsg(cv::Mat & R, cv::Mat & t)
{
  tf2::Matrix3x3 rotMat = tf2::Matrix3x3(
                            R.at<float>(0, 0),
                            R.at<float>(0, 1),
                            R.at<float>(0, 2),
                            R.at<float>(1, 0),
                            R.at<float>(1, 1),
                            R.at<float>(1, 2),
                            R.at<float>(2, 0),
                            R.at<float>(2, 1),
                            R.at<float>(2, 2)
                          );
  tf2::Quaternion quat;
  rotMat.getRotation(quat);


  geometry_msgs::Quaternion quat_msg;
  tf2::convert<tf2::Quaternion, geometry_msgs::Quaternion>(quat, quat_msg);

  geometry_msgs::Point point_msg;
  point_msg.x = t.at<float>(0);
  point_msg.y = t.at<float>(1);
  point_msg.z = t.at<float>(2);

  geometry_msgs::Pose pose_msg;
  pose_msg.orientation = quat_msg;
  pose_msg.position = point_msg;


  return pose_msg;
}


