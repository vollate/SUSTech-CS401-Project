#include "image_transport/publisher.h"
#include "ros/publisher.h"
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vector>
#include <std_msgs/Int16.h>

class Detector{
public:
    Detector(image_transport::ImageTransport& it, ros::NodeHandle& nh)
    {
        this->binary_pub = it.advertise("/detector/binary", 1);
        this->contour_pub = it.advertise("/detector/contours", 1);
        this->signal_pub = it.advertise("/detector/signal_result", 1);
        this->cross_pub = it.advertise("/detector/cross_result", 1);
    
        this->signal_result_pub = nh.advertise<std_msgs::Int16>("/detector/result/signal_result", 1);
    }

    cv::Mat binary(cv::Mat& image, int threshold)
    {
        cv::Mat binary;
        cv::cvtColor(image, binary, cv::COLOR_BGR2GRAY);
        cv::threshold(binary, binary, threshold, 255, cv::THRESH_BINARY);
        return binary;
    }

    cv::Mat br_sub_binary(cv::Mat& image, int threshold)
    {
        std::vector<cv::Mat> channels;
        cv::split(image, channels);
        cv::Mat blueChannel = channels[0];
        cv::Mat greenChannel = channels[1];
        cv::Mat redChannel = channels[2];

        cv::Mat diff;
        cv::subtract(redChannel, blueChannel, diff);

        cv::Mat binary;
        cv::threshold(diff, binary, threshold, 255, cv::THRESH_BINARY);
        return binary;
    }

    int detect_signal(cv::Mat& image)
    {
        int threshold = 30;
        int min_contour_area = 500;
        float min_circle_ratio = 0.8;


        cv::Mat binary = this->br_sub_binary(image, threshold);
        this->binary_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "mono8", binary).toImageMsg());
        
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        
        std::vector<std::vector<cv::Point>> contours_result;
        for (auto& contour : contours) {
            auto contour_area = cv::contourArea(contour);
            if (contour_area < min_contour_area) {
                continue;
            }

            cv::Point2f center;
            float radius;
            cv::minEnclosingCircle(contour, center, radius);
            double contourAreaVal = cv::contourArea(contour);
            double circleArea = CV_PI * radius * radius;
            double ratio = contourAreaVal / circleArea;
            if (ratio < min_circle_ratio) {
                continue;
            }
            contours_result.push_back(contour);
        }

        cv::Mat image_copy = image.clone();
        cv::drawContours(image_copy, contours_result, -1, cv::Scalar(0, 255, 0), 10);
        this->signal_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_copy).toImageMsg());
        
        return contours_result.size() > 0;
    }

    image_transport::Publisher binary_pub;
    image_transport::Publisher contour_pub;
    image_transport::Publisher signal_pub;
    image_transport::Publisher cross_pub;

    ros::Publisher signal_result_pub;
    ros::Publisher cross_result_pub;

};


int imageCallback(const sensor_msgs::ImageConstPtr& msg, Detector& detector)
{
    cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;

    int signal_result = detector.detect_signal(image);

    std_msgs::Int16 signal_result_msg;
    signal_result_msg.data = signal_result;
    detector.signal_result_pub.publish(signal_result_msg);
    
    return 0;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "detector_node");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    
    Detector detector(it, nh);

    image_transport::Subscriber sub = it.subscribe("/camera/rgb/image_raw", 1, boost::bind(imageCallback, _1, detector));

    ros::Rate loop_rate(5);
    while (nh.ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
}