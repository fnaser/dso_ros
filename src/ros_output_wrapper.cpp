
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

    // camera position
    double camX = m(0, 3);
    double camY = m(1, 3);
    double camZ = m(2, 3);

    // camera orientation
    Eigen::Quaterniond qe(m.block<3,3>(0,0));

    // get tf
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(camX, camY, camZ));
    tf::Quaternion q = tf::Quaternion(qe.x(), qe.y(), qe.z(), qe.w());
    transform.setRotation(q);

    // publish tf
    odom_broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                                                         "odom", "base_link")); //TODO param

    // odom msg
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "odom"; //TODO param
    odom.child_frame_id = "base_link"; //TODO param

    geometry_msgs::Pose pose;
    pose.position.x = camX;
    pose.position.y = camY;
    pose.position.z = camZ;
    pose.orientation.x = qe.x();
    pose.orientation.y = qe.y();
    pose.orientation.z = qe.z();
    pose.orientation.w = qe.w();

    odom.pose.pose = pose;

    odom.twist.twist.linear.x = pose.position.x - last_pose_.position.x;
    odom.twist.twist.linear.y = pose.position.y - last_pose_.position.y;
    odom.twist.twist.linear.z = pose.position.z - last_pose_.position.z;
//    odom.twist.twist.linear.x = pose.position.x;
//    odom.twist.twist.linear.y = pose.position.y;
//    odom.twist.twist.linear.z = pose.position.z;

    // publish odom
    if(!std::isnan(camX)      && !std::isnan(camY)   && !std::isnan(camZ)
       && !std::isnan(qe.x()) && !std::isnan(qe.y()) && !std::isnan(qe.z()) &&!std::isnan(qe.w())
       && !std::isinf(camX)   && !std::isinf(camY)   && !std::isinf(camZ)
       && !std::isinf(qe.x()) && !std::isinf(qe.y()) && !std::isinf(qe.z()) &&!std::isinf(qe.w()))
    {
        dso_odom_pub_.publish(odom);
    } else {
        ROS_WARN("Odom msg corrupted");
    }

    // distance traveled
    path_length_ += hypot ( pose.position.x - last_pose_.position.x,
                            pose.position.y - last_pose_.position.y);
//    ROS_INFO("Path length [%f]", path_length_);

//     TODO add pose
//     TODO causes seg fault
//    seq_poses_.insert(
//            std::pair<int, Eigen::Matrix<Sophus::SE3Group<double>::Scalar, 3, 4>>
//                                                                               (frame->id, m));

    // TODO check if seq full

    // TODO compute plane

    // Tracking
    Eigen::Matrix<Sophus::SE3Group<double>::Scalar, 4, 4> m_tmp;
    m_tmp << m,
            0, 0, 0, 1;

    if ( seq_tracking_.find(start_frame_id_) != seq_tracking_.end() &&
         !init_wpts_) {

        cv::Point p_tmp = seq_tracking_.find(start_frame_id_)->second;
        Eigen::Matrix<Sophus::SE3Group<double>::Scalar, 3, 1> ip_tmp; // img point
        ip_tmp << p_tmp.x,
                p_tmp.y,
                1;
        Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> cod_K(K);
        Eigen::Matrix<Sophus::SE3Group<double>::Scalar, Eigen::Dynamic, Eigen::Dynamic> cod_K_tmp = cod_K.pseudoInverse();

        wpt_ = m_tmp * cod_K_tmp * ip_tmp; // world point
        wpt_[3] = 1;

        std::cout << "\n"
                  << K
                  << "\n"
                  << m_tmp
                  << "\n"
                  << cod_K_tmp
                  << "\n"
                  << ip_tmp
                  << "\n"
                  << wpt_
                  << "\n"
                  << std::endl;

        init_wpts_ = true;

        //TODO more pts

//        // TODO test
//        seq_X_.push_back(wp_tmp[0]);
//        seq_Y_.push_back(wp_tmp[1]);
//        seq_Z_.push_back(wp_tmp[2]);
//
//        double averageX = accumulate( seq_X_.begin(), seq_X_.end(), 0.0)/seq_X_.size();
//        double averageY = accumulate( seq_Y_.begin(), seq_Y_.end(), 0.0)/seq_Y_.size();
//        double averageZ = accumulate( seq_Z_.begin(), seq_Z_.end(), 0.0)/seq_Z_.size();
//
//        double var = 0;
//        for (int i = 0; i < seq_X_.size(); i++) {
//            var += (seq_X_[i] - averageX) * (seq_X_[i] - averageX);
//        }
//        var /= seq_X_.size();
//        double sd = sqrt(var);
//        std::cout << sd << "\n" << std::endl;

    } else if (init_wpts_) {
        Eigen::Matrix<Sophus::SE3Group<double>::Scalar, 3, 1> ip_tmp = K * m_tmp.inverse() * wpt_; // img point

        cv::Point p;
        p.x = ip_tmp(0) / ip_tmp(2);
        p.y = ip_tmp(1) / ip_tmp(2);

//        std::cout << p << std::endl;

        int thickness = 10;
        int lineType = 8;

        if (seq_imgs_.find(frame->id) != seq_imgs_.end()) {
            cv::Mat img_tmp = seq_imgs_.find(frame->id)->second;
            cv::circle(img_tmp,
                       p,
                       5,
                       cv::Scalar(0, 0, 255),
                       thickness,
                       lineType);
            cv::imshow("Image Window Test [tracking]", img_tmp);
            cv::waitKey(1);
        }
    }

    // update pose
    last_pose_ = pose;
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

void dso::IOWrap::SampleOutputWrapper::addPointToSeq(cv::Point p, int id) {
    seq_tracking_.insert(
            std::pair<int, cv::Point>(id, p)
    );
}

// TODO ]
