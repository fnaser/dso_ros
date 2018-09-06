#include <iostream>

#include <list>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>


int main ( int argc, char** argv )
{
    if ( argc < 2 )
    {
        std::cout << "no image names given" << std::endl;
        return -1;
    }

    ros::init( argc, argv, "image_publisher" );

    ros::NodeHandle lo_nh;

    try
    {

        std::vector<cv::Mat> lv_images;

        ROS_INFO( "Loading images" );

        for ( int lo_i = 1; lo_i < argc; lo_i++ )
        {
            lv_images.push_back( cv::imread(std::string( argv[ lo_i ] ), CV_LOAD_IMAGE_COLOR ) );
        }

        ros::Publisher lo_pub = lo_nh.advertise<sensor_msgs::Image>("/usb_cam/image_raw", 5 );

        int lo_rate = 20;

        ROS_INFO_STREAM( "Publishing images at " << lo_rate << "hz" );
        ros::Rate lo_rate1( lo_rate );

        std::vector<cv::Mat>::iterator lo_iter_imgs( lv_images.begin() ), lo_iter_imgs_end( lv_images.end() );

        for ( ; lo_iter_imgs != lo_iter_imgs_end ; ++lo_iter_imgs )
        {
            cv_bridge::CvImage lo_img;
            lo_img.encoding = "bgr8";
            lo_img.image = *lo_iter_imgs;

            lo_pub.publish(lo_img.toImageMsg() );

            ros::spinOnce();
            lo_rate1.sleep();
        }
    }
    catch ( const std::exception& lo_e )
    {
        std::cout << "Exception in main: " << lo_e.what() << std::endl;
        return -1;
    }
    catch( ... )
    {
        std::cout << "Unknown exception in main" << std::endl;
        return -1;
    }

    return 0;
}