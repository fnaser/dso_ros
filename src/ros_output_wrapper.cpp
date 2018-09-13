
#include <dso_ros/ros_output_wrapper.h>

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

    // TODO compute plane
    // TODO check points and img association
    // TODO check homography

    if ( frame->id == start_frame_id_ &&
         seq_start_points_.find(start_frame_id_) != seq_start_points_.end() &&
         !wpts_init_
            ) {

        ROS_INFO("Adding world points ...");

        Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> cod_K(K);
        Eigen::Matrix<Sophus::SE3Group<double>::Scalar, Eigen::Dynamic, Eigen::Dynamic> cod_K_tmp = cod_K.pseudoInverse();

        std::vector <cv::Point> v_tmp;
        if (use_yaml_start_points_) {
            v_tmp = seq_start_points_.find(start_frame_id_)->second;
        } else {
            v_tmp.push_back(getPoint(frame->id));
            v_tmp.push_back(getPoint(frame->id));
            v_tmp.push_back(getPoint(frame->id));
            v_tmp.push_back(getPoint(frame->id));
        }
        std::cout << "v_tmp: " << v_tmp << std::endl;

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

        cv::imshow("Image Window Test [tracking]", img_tmp*2);
        cv::waitKey(1);
    }

    if (seq_cnt_ % seq_length_ == 0 && seq_cnt_ >= seq_length_) {

        for (int i=seq_length_-1; i>=0; i--) {

            int idx = frame->id - i;
            int start_idx = frame->id - seq_length_ + 1;

            if ( seq_img_points_.find(idx) != seq_img_points_.end() &&
                 seq_imgs_.find(idx) != seq_imgs_.end() ) {

                if (i <= seq_length_-2) {

//                    std::cout << seq_img_points_.find(start_idx)->second
//                              << seq_img_points_.find(idx)->second
//                              //                              << seq_img_points_.find(start_idx)->second[0].x
//                              << std::endl;

                    cv::Mat h = cv::findHomography(
                            seq_img_points_.find(idx)->second,
//                            seq_img_points_.find(start_idx)->second,
                            seq_scene_points_,
                            CV_RANSAC
                    );

                    cv::Mat img_tmp;
//                    cv::warpPerspective(seq_imgs_.find(idx)->second, img_tmp, h, seq_imgs_.find(start_idx)->second.size());
                    cv::warpPerspective(seq_imgs_.find(idx)->second, img_tmp, h, cv::Size(150,150));

                    //TODO cropping
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
                    cv::waitKey(1);
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

// TODO ]

cv::Point dso::IOWrap::SampleOutputWrapper::getPoint(int id) {
    bool fromCenter = true;
    cv::Rect bbox = cv::selectROI("Tracking ROI Selection", seq_imgs_.find(id)->second*2, fromCenter);
    cv::Point center_of_rect = (bbox.br() + bbox.tl()) *0.5;
    return center_of_rect;
};
