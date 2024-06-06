#include "image_transport/publisher.h"
#include "ros/publisher.h"
#include <cstdlib>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <utility>
#include <vector>
#include <std_msgs/Int16.h>

class Detector{
public:
    Detector(image_transport::ImageTransport& it, ros::NodeHandle& nh){

        this->signal_binary_pub = it.advertise("/detector/signal/binary", 1);
        this->signal_contour_pub = it.advertise("/detector/signal/contours", 1);
        this->signal_result_pub = nh.advertise<std_msgs::Int16>("/detector/signal/result", 1);

        this->cross_binary_pub = it.advertise("/detector/cross/binary", 1);
        this->cross_contour_pub = it.advertise("/detector/cross/contours", 1);
        this->cross_result_pub = nh.advertise<std_msgs::Int16>("/detector/cross/result", 1);

        this->red_light_binary_pub = it.advertise("/detector/light/red/binary", 1);
        this->red_light_contour_pub = it.advertise("/detector/light/red/contours", 1);

        this->yellow_light_binary_pub = it.advertise("/detector/light/yellow/binary", 1);
        this->yellow_light_contour_pub = it.advertise("/detector/light/yellow/contours", 1);
        this->light_result_pub = nh.advertise<std_msgs::Int16>("/detector/light/result", 1);
    }

    cv::Mat compressImg(cv::Mat& image, float ratio){
        cv::Mat compressed;
        cv::resize(image, compressed, cv::Size(image.cols * ratio, image.rows * ratio));
        return compressed;
    }

    cv::Mat binary(cv::Mat& image, int threshold){
        cv::Mat binary;
        cv::cvtColor(image, binary, cv::COLOR_BGR2GRAY);
        cv::threshold(binary, binary, threshold, 255, cv::THRESH_BINARY);
        return binary;
    }

    cv::Mat red_binary(cv::Mat& image, int threshold){
        std::vector<cv::Mat> channels;
        cv::split(image, channels);
        cv::Mat blueChannel = channels[0];
        cv::Mat greenChannel = channels[1];
        cv::Mat redChannel = channels[2];

        cv::Mat diff1, binary_1;
        cv::subtract(redChannel, blueChannel, diff1);
        cv::threshold(diff1, binary_1, threshold, 255, cv::THRESH_BINARY);

        cv::Mat diff2, binary_2;
        cv::subtract(redChannel, greenChannel, diff2);
        cv::threshold(diff2, binary_2, threshold, 255, cv::THRESH_BINARY);
        
        cv::Mat binary;
        cv::bitwise_and(binary_1, binary_2, binary);
        return binary;
    }

    cv::Mat yellow_binary(cv::Mat& image, int threshold){
        std::vector<cv::Mat> channels;
        cv::split(image, channels);
        cv::Mat blueChannel = channels[0];
        cv::Mat greenChannel = channels[1];
        cv::Mat redChannel = channels[2];

        cv::Mat diff1;
        cv::subtract(redChannel, blueChannel, diff1);

        cv::Mat diff2;
        cv::subtract(greenChannel, blueChannel, diff2);

        cv::Mat binary_1;
        cv::threshold(diff1, binary_1, threshold, 255, cv::THRESH_BINARY);

        cv::Mat binary_2;
        cv::threshold(diff2, binary_2, threshold, 255, cv::THRESH_BINARY);

        cv::Mat binary;
        cv::bitwise_and(binary_1, binary_2, binary);
        return binary;
    }


    int detect_signal(cv::Mat& image){

        cv::Mat compressed = image.clone();

        int threshold = 50;
        int min_contour_area = 500;
        float min_circle_ratio = 0.8;


        cv::Mat binary = this->red_binary(compressed, threshold);

        this->signal_binary_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "mono8", binary).toImageMsg());
        
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

        cv::drawContours(compressed, contours_result, -1, cv::Scalar(0, 255, 0), 10);
        this->signal_contour_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", compressed).toImageMsg());
        
        return contours_result.size() > 0 ? 1 : 0;
    }

    void detect_traffic_binary(cv::Mat& binary, cv::Mat& gray, int min_contour_area, float min_circle_ratio, int center_thresh, std::vector<std::vector<cv::Point>>& contours_result){
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for (auto& contour : contours){
            auto contour_area = cv::contourArea(contour);
            if (contour_area < min_contour_area){
                continue;
            }

            cv::Point2f center;
            float radius;
            cv::minEnclosingCircle(contour, center, radius);
            double contourAreaVal = cv::contourArea(contour);
            double circleArea = CV_PI * radius * radius;
            double ratio = contourAreaVal / circleArea;
            if (ratio < min_circle_ratio){
                continue;
            }

            // Check if the center of the circle is too light
            if (gray.at<uchar>(center) < center_thresh) {
                continue;
            }

            contours_result.push_back(contour);
        }
    }

    int detect_traffic_lights(cv::Mat& image){

        int yellow_threshold = 50;
        int red_threshold = 50;
        int min_contour_area = 200;
        float min_circle_ratio = 0.7;
        int center_thresh = 200;


        cv::Mat compressed = this->compressImg(image, 0.5);
        // cv::Mat compressed = image.clone();
        cv::Mat gray;
        cv::cvtColor(compressed, gray, cv::COLOR_BGR2GRAY);

        cv::Mat yellow_binary = this->yellow_binary(compressed, yellow_threshold);
        this->yellow_light_binary_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "mono8", yellow_binary).toImageMsg());
        
        auto yellow_contours = std::vector<std::vector<cv::Point>>();
        this->detect_traffic_binary(yellow_binary, gray, min_contour_area, min_circle_ratio, center_thresh, yellow_contours);
        cv::Mat yellow_contour_image = compressed.clone();
        cv::drawContours(yellow_contour_image, yellow_contours, -1, cv::Scalar(255, 255, 0), 10);
        this->yellow_light_contour_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", yellow_contour_image).toImageMsg());

        cv::Mat red_binary = this->red_binary(compressed, red_threshold);
        this->red_light_binary_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "mono8", red_binary).toImageMsg());
    
        auto red_contours = std::vector<std::vector<cv::Point>>();
        this->detect_traffic_binary(red_binary, gray, min_contour_area, min_circle_ratio, center_thresh, red_contours);
        cv::Mat red_contour_image = compressed.clone();
        cv::drawContours(red_contour_image, red_contours, -1, cv::Scalar(0, 0, 255), 10);
        this->red_light_contour_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", red_contour_image).toImageMsg());

        if (red_contours.size() > 0){
            return 2;
        } else if (yellow_contours.size() > 0){
            return 1;
        } else {
            return 0;
        }
    }

    int detect_cross(cv::Mat& image){
        int threshold = 250;
        int min_contour_area = 300;
        float min_rect_ratio = 5;
        float max_rect_angle = 10;

        cv::Mat compressed = this->compressImg(image, 0.25);
        cv::Mat binary = this->binary(compressed, threshold);

        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
        // cv::dilate(binary, binary, kernel);
        cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, kernel);

        this->cross_binary_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "mono8", binary).toImageMsg());

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        std::vector<std::vector<cv::Point>> contours_result;
        for (auto& contour : contours){
            auto contour_area = cv::contourArea(contour);
            if (contour_area < min_contour_area){
                continue;
            }

            cv::RotatedRect rect = cv::minAreaRect(contour);

            if (rect.center.y < 0.5 * compressed.rows){
                continue;
            }

            if (rect.size.width < rect.size.height){
                std::swap(rect.size.width, rect.size.height);
            }
            float ratio = rect.size.width / rect.size.height;
            if (ratio < min_rect_ratio){
                continue;
            }

            if (std::abs(rect.angle) > max_rect_angle){
                std::cout << "angle: " << rect.angle << std::endl;
                continue;
            }

            contours_result.push_back(contour);
        }

        cv::drawContours(compressed, contours_result, -1, cv::Scalar(0, 255, 0), 10);
        this->cross_contour_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", compressed).toImageMsg());

        return contours_result.size() > 0 ? 1 : 0;
    }
    

    image_transport::Publisher signal_binary_pub;
    image_transport::Publisher signal_contour_pub;

    image_transport::Publisher cross_binary_pub;
    image_transport::Publisher cross_contour_pub;

    image_transport::Publisher red_light_binary_pub;
    image_transport::Publisher red_light_contour_pub;
    image_transport::Publisher yellow_light_binary_pub;
    image_transport::Publisher yellow_light_contour_pub;

    
    ros::Publisher signal_result_pub;
    ros::Publisher cross_result_pub;
    ros::Publisher light_result_pub;
};


int imageCallback(const sensor_msgs::ImageConstPtr& msg, Detector& detector)
{
    cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;

    int signal_result = detector.detect_signal(image);
    std_msgs::Int16 signal_result_msg;
    signal_result_msg.data = signal_result;
    detector.signal_result_pub.publish(signal_result_msg);

    int light_result = detector.detect_traffic_lights(image);
    std_msgs::Int16 light_result_msg;
    light_result_msg.data = light_result;
    detector.light_result_pub.publish(light_result_msg);

    int cross_result = detector.detect_cross(image);
    std_msgs::Int16 cross_result_msg;
    cross_result_msg.data = cross_result;
    detector.cross_result_pub.publish(cross_result_msg);

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