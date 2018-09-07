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

#include <float.h>
#include <math.h>
#include <cassert>
#include <Eigen/QR>
#include <Eigen/Dense>
//#include <boost/accumulators/accumulators.hpp>
//#include <boost/accumulators/statistics.hpp>

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

            inline SampleOutputWrapper(ros::NodeHandle& n, int w, int h)
            {
                printf("OUT: Created SampleOutputWrapper\n");

                dso_odom_pub_ = n.advertise<nav_msgs::Odometry>("odom", 5, false); //TODO param

//                init_wpts_ = false;
                this->w_ = w;
                this->h_ = h;
//                trackingStarted_ = false;
//                tracker_ = cv::TrackerMIL::create(); //TODO param
                start_frame_id_ = 15;//250;//20;

                std::vector<cv::Point> center_points;
                center_points.push_back(cv::Point(100,100));
                center_points.push_back(cv::Point(50,100));
                center_points.push_back(cv::Point(150,100));
                this->addVecToSeq(center_points, start_frame_id_);

                wpts_init_ = false;
            }

            virtual ~SampleOutputWrapper()
            {
                printf("OUT: Destroyed SampleOutputWrapper\n");
            }

            virtual void publishGraph(const std::map<uint64_t, Eigen::Vector2i, std::less<uint64_t>, Eigen::aligned_allocator<std::pair<const uint64_t, Eigen::Vector2i>>> &connectivity) override
            {
//                printf("OUT: got graph with %d edges\n", (int)connectivity.size());
//
//                int maxWrite = 5;
//
//                for(const std::pair<uint64_t,Eigen::Vector2i> &p : connectivity)
//                {
//                    int idHost = p.first>>32;
//                    int idTarget = p.first & ((uint64_t)0xFFFFFFFF);
//                    printf("OUT: Example Edge %d -> %d has %d active and %d marg residuals\n", idHost, idTarget, p.second[0], p.second[1]);
//                    maxWrite--;
//                    if(maxWrite==0) break;
//                }
            }

            virtual void publishKeyframes( std::vector<FrameHessian*> &frames, bool final, CalibHessian* HCalib) override
            {
//                for(FrameHessian* f : frames)
//                {
//                    printf("OUT: KF %d (%s) (id %d, tme %f): %d active, %d marginalized, %d immature points. CameraToWorld:\n",
//                           f->frameID,
//                           final ? "final" : "non-final",
//                           f->shell->incoming_id,
//                           f->shell->timestamp,
//                           (int)f->pointHessians.size(), (int)f->pointHessiansMarginalized.size(), (int)f->immaturePoints.size());
//                    std::cout << f->shell->camToWorld.matrix3x4() << "\n";
//
//
//                    int maxWrite = 5;
//                    for(PointHessian* p : f->pointHessians)
//                    {
//                        printf("OUT: Example Point x=%.1f, y=%.1f, idepth=%f, idepth std.dev. %f, %d inlier-residuals\n",
//                               p->u, p->v, p->idepth_scaled, sqrt(1.0f / p->idepth_hessian), p->numGoodResiduals );
//                        maxWrite--;
//                        if(maxWrite==0) break;
//                    }
//                }
            }

            virtual void publishCamPose(FrameShell* frame, CalibHessian* HCalib) override;

            virtual void pushLiveFrame(FrameHessian* image) override {
//                int w = 640; //1280;//1280;
//                int h = 480; //720; //1024;
                MinimalImageB3* internalVideoImg = new MinimalImageB3(w_,h_);
                internalVideoImg->setBlack();

                for(int i=0;i<w_*h_;i++)
                    internalVideoImg->data[i][0] =
                    internalVideoImg->data[i][1] =
                    internalVideoImg->data[i][2] =
                            image->dI[i][0]*0.8 > 255.0f ? 255.0 : image->dI[i][0]*0.8;

                int frameID = image->shell->id;

                cv::Mat tmp = cv::Mat(h_, w_, CV_8UC3, internalVideoImg->data);

//                if(!trackingStarted_ && frameID == start_frame_id_) {
//                    ROS_INFO("Start Tracking ...");
//                    bool fromCenter = true;
//                    bbox_ = cv::selectROI("Tracking ROI Selection", tmp, fromCenter);
//                    tracker_->init(tmp, bbox_);
//                    trackingStarted_ = true;
//                } else {
//                    bool ok = tracker_->update(tmp, bbox_);
//                }
//
//                cv::Point center_of_rect = (bbox_.br() + bbox_.tl()) *0.5;
//                this->addPointToSeq(center_of_rect, frameID);

//                std::cout << bbox_ << std::endl;
//                std::cout << center_of_rect << std::endl;

                this->addImgToSeq(tmp, frameID);
            }

            virtual void pushDepthImage(MinimalImageB3* image) override;

            virtual bool needPushDepthImage() override
            {
                return false;
            }

            virtual void pushDepthImageFloat(MinimalImageF* image, FrameHessian* KF ) override
            {
//                printf("OUT: Predicted depth for KF %d (id %d, time %f, internal frame-ID %d). CameraToWorld:\n",
//                       KF->frameID,
//                       KF->shell->incoming_id,
//                       KF->shell->timestamp,
//                       KF->shell->id);
//                std::cout << KF->shell->camToWorld.matrix3x4() << "\n\n";
//
//                int maxWrite = 5;
//                for(int y=0;y<image->h;y++)
//                {
//                    for(int x=0;x<image->w;x++)
//                    {
//                        if(image->at(x,y) <= 0) continue;
//
//                        printf("OUT: Example Idepth at pixel (%d,%d): %f.\n", x,y,image->at(x,y));
//                        maxWrite--;
//                        if(maxWrite==0) break;
//                    }
//                    if(maxWrite==0) break;
//                }
//                std::cout << KF->PRE_camToWorld.matrix3x4() << "\n\n";
//                std::cout << KF->PRE_worldToCam.matrix3x4() << "\n\n";
            }

            void addImgToSeq(cv_bridge::CvImagePtr, int id);
            void addImgToSeq(cv::Mat, int id);
//            void addPointToSeq(cv::Point, int id);
            void addVecToSeq(std::vector<cv::Point>, int id);

        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        private:

            geometry_msgs::Pose last_pose_;

            double path_length_;

            ros::Publisher dso_odom_pub_;
            tf::TransformBroadcaster odom_broadcaster_;

            std::map<int, cv::Mat> seq_imgs_;
//            std::map<int, Eigen::Matrix<Sophus::SE3Group<double>::Scalar, 3, 4>> seq_poses_;
            std::map<int, std::vector<cv::Point>> seq_points_;

            std::vector<Eigen::Matrix<double, 4, 1, Eigen::DontAlign>> wpts_;
            boost::mutex wpts_mutex_;
            bool wpts_init_;
            int w_, h_;
            int start_frame_id_;
        };
    }
}

