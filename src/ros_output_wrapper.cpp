
#include <dso_ros/ros_output_wrapper.h>

void dso::IOWrap::SampleOutputWrapper::pushDepthImage(MinimalImageB3* image) {
//    dso::IOWrap::displayImage("test", image, true);
//    cv::waitKey(1);
}

// TODO under construction
//void dso::IOWrap::SampleOutputWrapper::pushDepthImageFloat(dso::MinimalImageF* image,
//                                                           dso::FrameHessian* KF) {
//    cv::Mat image_cv(image->h, image->w, CV_32FC1, image->data);
//    image_cv.convertTo(image_cv, CV_8UC1, 255.0f);
//    cv::Mat inverted_img;
//    cv::bitwise_not(image_cv, inverted_img);
//
//    cv::imshow("Image Window Test [depthimg]", image_cv);
//    cv::waitKey(1);
//}

/*
 * frame->camToWorld.matrix3x4() returns:
 *
 * m00 m01 m02 m03
 * m10 m11 m12 m13
 * m20 m21 m22 m23
 *
 * last column is translation vector
 * 3x3 matrix with diagonal m00 m11 and m22 is a rotation matrix
 *
*/

void dso::IOWrap::SampleOutputWrapper::publishCamPose(dso::FrameShell* frame,
                                                      dso::CalibHessian* HCalib) {
    // TODO under construction
//    ROS_DEBUG_STREAM("publishCamPose called");
//
//    printf("OUT: Current Frame %d (time %f, internal ID %d). CameraToWorld:\n",
//           frame->incoming_id,
//           frame->timestamp,
//           frame->id);
//    std::cout << frame->camToWorld.matrix3x4() << "\n";

    // Init K
    Eigen::Matrix<Sophus::SE3Group<double>::Scalar, 3, 4> K =
            Eigen::Matrix<Sophus::SE3Group<double>::Scalar, 3, 4>::Zero(3, 4);
    K(0,0) = HCalib->fxl();
    K(1,1) = HCalib->fyl();
    K(0,2) = HCalib->cxl();
    K(1,2) = HCalib->cyl();
    K(2,2) = 1;

    // Init m
    Eigen::Matrix<Sophus::SE3Group<double>::Scalar, 3, 4> m =
            frame->camToWorld.matrix3x4();

    // Init m tmp
    Eigen::Matrix<Sophus::SE3Group<double>::Scalar, 4, 4> m_tmp;
    m_tmp << m,
            0, 0, 0, 1;

//    // camera position
//    double camX = m(0, 3);
//    double camY = m(1, 3);
//    double camZ = m(2, 3);
//
//    // camera orientation
//    Eigen::Quaterniond qe(m.block<3,3>(0,0));
//
//    // get tf
//    tf::Transform transform;
//    transform.setOrigin(tf::Vector3(camX, camY, camZ));
//    tf::Quaternion q = tf::Quaternion(qe.x(), qe.y(), qe.z(), qe.w());
//    transform.setRotation(q);
//
//    // publish tf
//    odom_broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
//                                                         "odom", "base_link")); //TODO param
//
//    // odom msg
//    nav_msgs::Odometry odom;
//    odom.header.stamp = ros::Time::now();
//    odom.header.frame_id = "odom"; //TODO param
//    odom.child_frame_id = "base_link"; //TODO param
//
//    geometry_msgs::Pose pose;
//    pose.position.x = camX;
//    pose.position.y = camY;
//    pose.position.z = camZ;
//    pose.orientation.x = qe.x();
//    pose.orientation.y = qe.y();
//    pose.orientation.z = qe.z();
//    pose.orientation.w = qe.w();
//
//    odom.pose.pose = pose;
//
//    odom.twist.twist.linear.x = pose.position.x - last_pose_.position.x;
//    odom.twist.twist.linear.y = pose.position.y - last_pose_.position.y;
//    odom.twist.twist.linear.z = pose.position.z - last_pose_.position.z;
////    odom.twist.twist.linear.x = pose.position.x;
////    odom.twist.twist.linear.y = pose.position.y;
////    odom.twist.twist.linear.z = pose.position.z;
//
//    // publish odom
//    if(!std::isnan(camX)      && !std::isnan(camY)   && !std::isnan(camZ)
//       && !std::isnan(qe.x()) && !std::isnan(qe.y()) && !std::isnan(qe.z()) &&!std::isnan(qe.w())
//       && !std::isinf(camX)   && !std::isinf(camY)   && !std::isinf(camZ)
//       && !std::isinf(qe.x()) && !std::isinf(qe.y()) && !std::isinf(qe.z()) &&!std::isinf(qe.w()))
//    {
//        dso_odom_pub_.publish(odom);
//    } else {
//        ROS_WARN("Odom msg corrupted");
//    }
//
//    // distance traveled
//    path_length_ += hypot ( pose.position.x - last_pose_.position.x,
//                            pose.position.y - last_pose_.position.y);
////    ROS_INFO("Path length [%f]", path_length_);
//
//     TODO add pose
//     TODO causes seg fault
//    seq_poses_.insert(
//            std::pair<int, Eigen::Matrix<Sophus::SE3Group<double>::Scalar, 3, 4>>
//                                                                               (frame->id, m));
//
//    // update pose
//    last_pose_ = pose;

    // TODO compute plane

    if ( frame->id == start_frame_id_ &&
         seq_start_points_.find(start_frame_id_) != seq_start_points_.end() &&
         !wpts_init_
            ) {

        ROS_INFO("Adding world points ...");

        Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> cod_K(K);
        Eigen::Matrix<Sophus::SE3Group<double>::Scalar, Eigen::Dynamic, Eigen::Dynamic> cod_K_tmp = cod_K.pseudoInverse();

        std::vector<cv::Point> v_tmp = seq_start_points_.find(start_frame_id_)->second;

        for(std::vector<cv::Point>::iterator it = v_tmp.begin(); it != v_tmp.end(); ++it) {

            Eigen::Matrix<Sophus::SE3Group<double>::Scalar, 3, 1> ip_tmp; // img point
            ip_tmp << it->x,
                    it->y,
                    1;

            Eigen::Matrix<double, 4, 1, Eigen::DontAlign> w_tmp = m_tmp * cod_K_tmp * ip_tmp; // world point
            w_tmp[3] = 1;

            std::cout << "\n"
                      << K
                      << "\n"
                      << m_tmp
                      << "\n"
                      << cod_K_tmp
                      << "\n"
                      << ip_tmp
                      << "\n"
                      << w_tmp
                      << "\n"
                      << std::endl;

            wpts_mutex_.lock();
            wpts_.push_back(w_tmp);
            wpts_mutex_.unlock();
        }

        wpts_init_=true;

    } else if ( wpts_init_ &&
                seq_imgs_.find(frame->id) != seq_imgs_.end() ) {

//        assert(seq_points_.size() == wpts_.size());
        assert(wpts_.size() == 4); //TODO params

        cv::Mat img_tmp = seq_imgs_.find(frame->id)->second;
        std::vector<cv::Point> ipts_tmp;

        for(int i=0; i < wpts_.size(); i++) {

            Eigen::Matrix<Sophus::SE3Group<double>::Scalar, 3, 1> ip_tmp = K * m_tmp.inverse() * wpts_[i]; // img point

            cv::Point p;
            p.x = ip_tmp(0) / ip_tmp(2);
            p.y = ip_tmp(1) / ip_tmp(2);

            int thickness = 10;
            int lineType = 8;

            cv::circle(img_tmp,
                       p,
                       5,
                       cv::Scalar(0, 0, 255),
                       thickness,
                       lineType);

            ipts_tmp.push_back(p);
        }

        seq_img_points_.insert(
                std::pair<int, std::vector<cv::Point>>(frame->id, ipts_tmp)
        );

        seq_cnt_++;

        cv::imshow("Image Window Test [tracking]", img_tmp);
        cv::waitKey(1);
    }

    if (seq_cnt_ % seq_length_ == 0 && seq_cnt_ >= seq_length_) { // TODO param
//        ROS_INFO("seq full");

        for (int i=seq_length_-1; i>=0; i--) {

            int idx = frame->id - i;
            int start_idx = frame->id - seq_length_ + 1;

            if ( seq_img_points_.find(idx) != seq_img_points_.end() &&
                 seq_imgs_.find(idx) != seq_imgs_.end() ) {
//                ROS_INFO("seq idx %d", frame->id-i);

                if (i <= seq_length_-2) {

//                    std::cout << seq_img_points_.find(start_idx)->second
//                              << seq_img_points_.find(idx)->second
//                              //                              << seq_img_points_.find(start_idx)->second[0].x
//                              << std::endl;

                    cv::Mat h = cv::findHomography(
                            seq_img_points_.find(idx)->second,
                            seq_img_points_.find(start_idx)->second,
                            CV_RANSAC
                    );

                    cv::Mat img_tmp;
                    cv::warpPerspective(seq_imgs_.find(idx)->second, img_tmp, h, seq_imgs_.find(start_idx)->second.size());

//                    try {
//                        img_tmp = img_tmp(cv::Rect(seq_img_points_.find(start_idx)->second[3].x, //TODO param
//                                                   seq_img_points_.find(start_idx)->second[3].y,
//                                                   seq_img_points_.find(start_idx)->second[0].x,
//                                                   seq_img_points_.find(start_idx)->second[0].y
//                        ));
//                    } catch (cv::Exception& e) {
//                        ROS_WARN("Cropping failed.");
//                    }

                    cv::imshow("Image Window Test [homography]", img_tmp);
                    cv::waitKey(300);
                }


            } else {
                ROS_WARN("Image Point or Image is missing in Sequence");
            }
        }
    }
}

// TODO under construction [

void dso::IOWrap::SampleOutputWrapper::addImgToSeq(cv_bridge::CvImagePtr cv_ptr, int id) {
    seq_imgs_.insert(std::pair<int, cv::Mat>(id, cv_ptr->image)); //TODO check efficiency
    // TODO reset requested
}

void dso::IOWrap::SampleOutputWrapper::addImgToSeq(cv::Mat img, int id) {
    seq_imgs_.insert(std::pair<int, cv::Mat>(id, img)); //TODO check efficiency
    // TODO reset requested
}

//void dso::IOWrap::SampleOutputWrapper::addPointToSeq(cv::Point p, int id) {
//    seq_tracking_.insert(
//            std::pair<int, cv::Point>(id, p)
//    );
//}

// TODO ]
