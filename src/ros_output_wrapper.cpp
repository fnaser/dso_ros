
#include <dso_ros/ros_output_wrapper.h>

void dso::IOWrap::SampleOutputWrapper::publishCamPose(dso::FrameShell* frame,
                                                      dso::CalibHessian* HCalib) {
    ROS_DEBUG_STREAM("publishCamPose called");

    printf("OUT: Current Frame %d (time %f, internal ID %d). CameraToWorld:\n",
           frame->incoming_id,
           frame->timestamp,
           frame->id);
    std::cout << frame->camToWorld.matrix3x4() << "\n";

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

    const Eigen::Matrix<Sophus::SE3Group<double>::Scalar, 3, 4> m =
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
    if(!std::isnan(camX)&&!std::isnan(camY)&&!std::isnan(camZ)
       && !std::isnan(qe.x()) && !std::isnan(qe.y())&& !std::isnan(qe.z())&&!std::isnan(qe.w())
       &&!std::isinf(camX)&&!std::isinf(camY)&&!std::isinf(camZ)
       && !std::isinf(qe.x()) && !std::isinf(qe.y())&& !std::isinf(qe.z())&&!std::isinf(qe.w()))
    {
        dso_odom_pub_.publish(odom);
    } else {
        ROS_WARN("Odom msg corrupted");
    }

    // distance traveled
    path_length_ += hypot ( pose.position.x - last_pose_.position.x,
                            pose.position.y - last_pose_.position.y);
    ROS_INFO("path length [%f]", path_length_);

    // update pose
    last_pose_ = pose;
}

//TODO under construction [
void dso::IOWrap::SampleOutputWrapper::addImgToSeq(cv_bridge::CvImagePtr cv_ptr, int id) {
//void dso::IOWrap::SampleOutputWrapper::addImgToSeq(cv::Mat img, int id) {
    if (id % 10 == 0 && id > 10) {
//        cv::imshow("Image Window Test [iowrapper]", cv_ptr->image);
//        cv::imshow("Image Window Test [iowrapper]", img);
        cv::imshow("Image Window Test [iowrapper]", seq_imgs_.find(id-10)->second);
        cv::waitKey(1);
    } else {
        seq_imgs_.insert(std::pair<int, cv::Mat>(id, cv_ptr->image));
//        seq_imgs_.push_back(img);
    }
}
//TODO ]