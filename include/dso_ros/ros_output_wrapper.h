/**
* This file is part of DSO.
* 
* Copyright 2016 Technical University of Munich and Intel.
* Developed by Jakob Engel <engelj at in dot tum dot de>,
* for more information see <http://vision.in.tum.de/dso>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* DSO is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* DSO is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with DSO. If not, see <http://www.gnu.org/licenses/>.
*/


#pragma once

#include "boost/thread.hpp"
#include "util/MinimalImage.h"
#include "IOWrapper/Output3DWrapper.h"

#include "FullSystem/HessianBlocks.h"
#include "util/FrameShell.h"
#include "util/settings.h"

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <float.h>
#include <math.h>
#include <cassert>
#include <Eigen/QR>
#include <Eigen/Dense>

namespace dso
{

    class FrameHessian;
    class CalibHessian;
    class FrameShell;

    namespace IOWrap
    {

        class SampleOutputWrapper : public Output3DWrapper
        {

        public:

            inline SampleOutputWrapper(ros::NodeHandle& n, int w, int h, bool use_yaml_start_points)
            {
                printf("OUT: Created SampleOutputWrapper\n");

//                dso_odom_pub_ = n.advertise<nav_msgs::Odometry>("odom", 5, false); //TODO param

                this->w_ = w;
                this->h_ = h;
                start_frame_id_ = 100; //TODO param
                seq_cnt_ = 0;
                seq_length_ = 10; //TODO param
                this->use_yaml_start_points_ = use_yaml_start_points;

                std::vector<cv::Point> center_points;
//                center_points.push_back(cv::Point(150, 150)); //TODO param
//                center_points.push_back(cv::Point( 50, 150));
//                center_points.push_back(cv::Point(150, 50));
//                center_points.push_back(cv::Point( 50, 50));
//                center_points.push_back(cv::Point(174, 244)); //TODO param 15
//                center_points.push_back(cv::Point(128, 204));
//                center_points.push_back(cv::Point(76, 224));
//                center_points.push_back(cv::Point(115, 278));
//                center_points.push_back(cv::Point(365, 350)); //TODO param 100
//                center_points.push_back(cv::Point(290, 328));
//                center_points.push_back(cv::Point(278, 394));
//                center_points.push_back(cv::Point(358, 417));
                center_points.push_back(cv::Point(404, 194)); //TODO param 100
                center_points.push_back(cv::Point(311, 165));
                center_points.push_back(cv::Point(297, 232));
                center_points.push_back(cv::Point(372, 265));
                seq_start_points_.insert(
                        std::pair<int, std::vector<cv::Point>>(start_frame_id_, center_points)
                ); //TODO change data type

                seq_scene_points_.push_back(cv::Point(150,   0)); //TODO param
                seq_scene_points_.push_back(cv::Point(  0,   0));
                seq_scene_points_.push_back(cv::Point(  0, 150));
                seq_scene_points_.push_back(cv::Point(150, 150));
//                seq_scene_size_ = cv::

                wpts_init_ = false;
            }

            virtual ~SampleOutputWrapper()
            {
                printf("OUT: Destroyed SampleOutputWrapper\n");
            }

            virtual void publishGraph(const std::map<uint64_t, Eigen::Vector2i, std::less<uint64_t>, Eigen::aligned_allocator<std::pair<const uint64_t, Eigen::Vector2i>>> &connectivity) override {}

            virtual void publishKeyframes( std::vector<FrameHessian*> &frames, bool final, CalibHessian* HCalib) override {}

            virtual void publishCamPose(FrameShell* frame, CalibHessian* HCalib) override;

            virtual void pushLiveFrame(FrameHessian* image) override {

                MinimalImageB3* internalVideoImg = new MinimalImageB3(w_,h_);
                internalVideoImg->setBlack();

                for(int i=0;i<w_*h_;i++)
                    internalVideoImg->data[i][0] =
                    internalVideoImg->data[i][1] =
                    internalVideoImg->data[i][2] =
                            image->dI[i][0]*0.8 > 255.0f ? 255.0 : image->dI[i][0]*0.8;

                int frameID = image->shell->id;

                cv::Mat tmp = cv::Mat(h_, w_, CV_8UC3, internalVideoImg->data);

                this->addImgToSeq(tmp, frameID);
            }

            virtual void pushDepthImage(MinimalImageB3* image) override {}

            virtual bool needPushDepthImage() override
            {
                return false;
            }

            virtual void pushDepthImageFloat(MinimalImageF* image, FrameHessian* KF ) override {}

            void addImgToSeq(cv_bridge::CvImagePtr, int id);
            void addImgToSeq(cv::Mat, int id);
            cv::Point getPoint(int id);

        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        private:

            geometry_msgs::Pose last_pose_;
            double path_length_;

            ros::Publisher dso_odom_pub_;
            tf::TransformBroadcaster odom_broadcaster_;

            std::map<int, cv::Mat> seq_imgs_;
            std::map<int, std::vector<cv::Point>> seq_start_points_;
            std::map<int, std::vector<cv::Point>> seq_img_points_;
            std::vector<cv::Point> seq_scene_points_;

            std::vector<Eigen::Matrix<double, 4, 1, Eigen::DontAlign>> wpts_;
            boost::mutex wpts_mutex_;
            bool wpts_init_;

            int seq_cnt_;
            int seq_length_;
            int w_, h_;
            int start_frame_id_;
            bool use_yaml_start_points_;

        };
    }
}

