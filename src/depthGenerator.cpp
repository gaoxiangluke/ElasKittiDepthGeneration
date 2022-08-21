#include <ros/ros.h>
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Image.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "libelas/elas.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace std;
bool showImage;
Elas::parameters elas_init_params() {
    Elas::parameters p;
    p.postprocess_only_left = true;
    return p;
}
static Elas elas_(elas_init_params());
struct Calibration{
    float baseline;
    Eigen::Matrix3f intrinsic;
};
 enum TraceStatus {GOOD=0,
                      OOB,
                      OUTLIER};
TraceStatus pt_depth_from_disparity(int h, int w, int u, int v,
                                    const std::vector<float> & disparity,
                                    const Eigen::Matrix3f & intrinsic,
                                    float baseline,
                                    // output
                                    Eigen::Vector3f result
                                    )  {
    //Vec3f bl;
    //bl << baseline, 0, 0;
    if ( u < 1 || u > w-2 || v < 1 || v > h -2 )
    return TraceStatus::OOB;

    if (disparity[w * v + u] <= 0.05)
    return TraceStatus::OUTLIER;
    
    float depth = std::abs(baseline) * intrinsic(0,0) / disparity[w * v + u];

    result << static_cast<float>(u), static_cast<float>(v), 1.0;
    
    result = (intrinsic.inverse() * result * depth).eval();

    return TraceStatus::GOOD;

}
TraceStatus pt_depth_from_disparity(const cv::Mat & left,
                                //const cv::Mat & right_gray,
                                const std::vector<float> & disparity,
                                const Calibration & calib,
                                const Eigen::Vector2d & input,
                                Eigen::Vector3f result
                                )  {

    int u = input(0);
    int v = input(1);
    int h = left.rows;
    int w = left.cols;

    return pt_depth_from_disparity(h, w, u, v, disparity,calib.intrinsic,
                                    calib.baseline, result);

}


void disparity(const cv::Mat & left_in,
                   const cv::Mat & right_in,
                   std::vector<float> & output_left_disparity)  {

    auto cvt_if_color = [] (const cv::Mat& input) -> cv::Mat {
    cv::Mat ret;
    if (input.channels() == 1)
        ret = input;
    else
        cv::cvtColor(input, ret, cv::COLOR_BGR2GRAY);
    return ret;
    };
    cv::Mat left_gray = cvt_if_color(left_in);
    cv::Mat right_gray = cvt_if_color(right_in);
    
    int32_t width = left_gray.cols;
    int32_t height = left_gray.rows;
    output_left_disparity.resize(width*height);
    std::vector<float> right_disparity(left_gray.total());
    int32_t dims[3] = {width , height, width};
    elas_.process(left_gray.data, right_gray.data,
                output_left_disparity.data(), right_disparity.data(),
                dims);

    bool is_visualize = false;
    if (is_visualize) {
    std::vector<uint8_t> vis(output_left_disparity.size());
    float disp_max = 0;
    auto D1_data = output_left_disparity.data();
    for (int32_t i=0; i<width*height; i++) {
        if (D1_data[i]>disp_max) disp_max = D1_data[i];
    }
    // copy float to uchar
    for (int32_t i=0; i<width*height; i++) 
        vis[i] = (uint8_t)std::max(255.0*D1_data[i]/disp_max,0.0);
    // convet to cv mat
    cv::Mat disp_left(height, width, CV_8UC1, vis.data());
    //cv::namedWindow("Disparity left", )
    cv::imshow("Left disparity", disp_left);
    cv::waitKey();
    }
}

void filterCallback(const sensor_msgs::ImageConstPtr leftImage,const sensor_msgs::ImageConstPtr & rightImage) {
    ROS_INFO("subscribing: right image %s", leftImage->header.frame_id.c_str());  
    ROS_INFO("subscribing: left image %s", rightImage->header.frame_id.c_str());
    cv_bridge::CvImagePtr left_ptr;
    cv_bridge::CvImagePtr right_ptr;
    //cv_bridge::CvImagePtr descriptor_ptr;
    sensor_msgs::ImageConstPtr leftImg( new sensor_msgs::Image( *leftImage ) );
    sensor_msgs::ImageConstPtr rightImg( new sensor_msgs::Image( *rightImage ) );
    //sensor_msgs::ImageConstPtr descriptorImg( new sensor_msgs::Image( msg->descriptor) );
    try {
        left_ptr = cv_bridge::toCvCopy(leftImg, sensor_msgs::image_encodings::BGR8);
        right_ptr = cv_bridge::toCvCopy(rightImg, sensor_msgs::image_encodings::BGR8);
        //descriptor_ptr = cv_bridge::toCvCopy(descriptorImg,sensor_msgs::image_encodings::TYPE_16UC1);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat left = left_ptr->image;
    cv::Mat right = right_ptr->image;
    if (showImage){
        cv::imshow("left", left);
        cv::imshow("tight", right);
        cv::waitKey(1);
    }

    std::vector<float> leftDisparity;

    //generate disparity map
    disparity(left,right,leftDisparity);
    //generate depth

    // publish depth with left timestamp
    //cv::Mat descriptor = descriptor_ptr->image;
     //auto timestap = 
}


int main(int argc, char **argv)
{
    showImage = false;
    ros::init(argc, argv, "depth_genertor");
    ros::NodeHandle nh("~");

    string left_image, right_image;
    nh.param<string>("left_image_topic", left_image, "left");
    nh.param<string>("right_image_topic", right_image, "right");
    
    //message filters
    message_filters::Subscriber<sensor_msgs::Image> subLeftImage(nh, left_image, 5000);
    message_filters::Subscriber<sensor_msgs::Image> subRightImage(nh, right_image, 5000);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
    message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(5000), subLeftImage,  subRightImage);
    sync.registerCallback(boost::bind(&filterCallback, _1, _2));
  

    ros::spin();
}