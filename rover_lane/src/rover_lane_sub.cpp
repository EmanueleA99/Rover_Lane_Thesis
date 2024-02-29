// Emanuele Arilli - Master's Thesis //
// emanuelearilli@gmail.com          //
// January 2024                      //

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber.hpp>
#include <opencv2/opencv.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int32.hpp>
#include <iostream>
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/int32.hpp"
#include <std_srvs/srv/set_bool.hpp>

using namespace std::chrono_literals;
using namespace std;
using namespace std_msgs::msg;
using namespace sensor_msgs::msg;
using namespace std_srvs::srv;

class LaneSubscriber : public rclcpp::Node
{
    public:
    LaneSubscriber() : Node("lane_subscriber")
        {
            // parameters
            this->declare_parameter("proportional_kc", 0.10);
            this->declare_parameter("linear_velocity", 0.0);
            this->declare_parameter("filter_parameter", 0.25);
            this->declare_parameter("dead_zone", 2.5);

            enable_server_ = this->create_service<SetBool>("~/enable",std::bind(&LaneSubscriber::enable_callback,this,
            std::placeholders::_1,
            std::placeholders::_2));
            
            // publisher TwistStamped
            ctrl_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/rover/cmd_vel",10);

        }

    void enable_callback(
        SetBool::Request::SharedPtr req,
        SetBool::Response::SharedPtr resp)
        {
        if (req->data) {
            if (!is_on_) {
            lane_sub_ = image_transport::create_subscription(this, "/camera/camera/color/image_raw", std::bind(&LaneSubscriber::callback, this, std::placeholders::_1), "raw");
            is_on_ = true;
            RCLCPP_WARN(this->get_logger(), "Detector ACTIVATED");
            }
            resp->set__success(true);
            resp->set__message("");
        } else {
            if (is_on_) {
            lane_sub_.shutdown();
            is_on_ = false;
            RCLCPP_WARN(this->get_logger(), "Detector DEACTIVATED");
            }
            resp->set__success(true);
            resp->set__message("");
        }
        }

    std::vector<double> display_lines(cv::Mat image, vector<float> lines_avg)
        {
        int height = image.rows;
        int width = image.cols;
        int offset;
        int i;
        float slope, y_int;
        int x1, x2, y1, y2;

        //steering wheel deadzone parameter
        double dead_zone = this->get_parameter("dead_zone").as_double();
        offset = (int) width / 100 * dead_zone;

        for (i=0; i<2; i++)
        {
            slope = lines_avg[0+2*i];
            y_int = lines_avg[1+2*i];

            //std::cout << slope << " " << y_int << endl;

            y1 = height;
            y2 = (int) (y1 * (float)3/5);
            x1 = (int) ((y1 - y_int) / slope);
            x2 = (int) ((y2 - y_int) / slope);

            //std::cout << x1 << " " << x2 << " " << y1 << " " << y2 << endl;

            cv::line(image, cv::Point (x1,y1), cv::Point (x2,y2), cv::Scalar(255,0,0), 10);
        }

        //Deadzone Lines
        cv::line(image, cv::Point ((int)width/2 - offset,0), cv::Point ((int)width/2 - offset,height), cv::Scalar(0,0,255), 3, 8, 0);
        cv::line(image, cv::Point ((int)width/2 + offset,0), cv::Point ((int)width/2 + offset,height), cv::Scalar(0,0,255), 3, 8, 0);

        std::vector<int> result = intersection(lines_avg, image);
        int x = result[0];
        
        //call the control law for lane keeping
        std::vector<double> control_result = control_law(x,offset,width);
        return control_result;
    }

    std::vector<double> control_law(double x, double offset, double width) {
        
        //Proportional Control Law for lane keeping
        double kc = this->get_parameter("proportional_kc").as_double();
        double velocity = this->get_parameter("linear_velocity").as_double();

        if(x == 0){ //Lane not detected case
            std::cout << "Lane not available" << std::endl;
            double steering_angle = 0.0; //no steering correction
            std::vector<double> law = {velocity, steering_angle};
            return law;
        }
        if (x > (width / 2 - offset) && x < (width / 2 + offset)) {
            std::cout << "Dritto" << std::endl;
            double steering_angle = 0.0; //no steering correction
            std::vector<double> law = {velocity, steering_angle};
        return law;
        } else {
            double new_x = x - width / 2; // Set new zero to the center of the screen
            double steering_angle = kc * new_x; //steering correction
            std::cout << steering_angle << std::endl;
            std::vector<double> law = {velocity, steering_angle};
        return law;
        }
    }

    std::vector<int> intersection(vector<float> lines_avg, cv::Mat image)
    {
        int height = image.rows;
        float m1 = lines_avg[0];
        float q1 = lines_avg[1];
        float m2 = lines_avg[2];
        float q2 = lines_avg[3];

        int x;
        int y;
        //Check if lane is detected
        if(!std::isnan(m1) && !std::isnan(q1) && !std::isnan(m2) && !std::isnan(q2)){
            x = (int) (q2 - q1) / (m1 - m2);
            y = (int) (m1*x) + q1;
        }else{ //lane not detected
            x = 0;
            previous_x_ = 0;
        }

        //x filter
        const double alpha = this->get_parameter("filter_parameter").as_double();  // Peso del valore corrente
        if (previous_x_ != 0 && !std::isnan(previous_x_) && x!=0){
        x = (int) (alpha * x + (1 - alpha) * previous_x_);
        }
        previous_x_ = x;

        //Print on image red dot x value
        cv::circle(image, cv::Point(x,height/2), 5, cv::Scalar(0, 0, 255), 10);
        std::vector<int> xy = {x,y};
        return xy;
    }

    std::vector<float> average(vector<cv::Vec4i> lines)
    {
        int x1, y1, x2, y2, i;
        float slope, y_int, slope_sum, y_int_sum;
        
        std::vector<std::vector<float>>  right_lines, left_lines;
        std::vector<float> temp_vect;

        for (i=0; i<(int)lines.size(); i++)
        {
            x1 = lines[i][0]; 
            y1 = lines[i][1];
            x2 = lines[i][2];
            y2 = lines[i][3];
        
            //fit line
            if ((x2 - x1) != 0)
            {
                slope = (float) (y2 - y1) / (x2 - x1);
            } 
            else
            {
                //slope = (float) 1000;
                continue;
            } 
            y_int =  y1 - slope*x1;

            std::vector<float> temp_vect = {slope, y_int};

            //std::cout << slope << " " << y_int << endl;

            if (slope < 0)
            {
                left_lines.push_back(temp_vect);
            }
            else{
                right_lines.push_back(temp_vect);
            }
        }

        slope_sum = 0;
        y_int_sum = 0;

        // average right
        for (i=0; i<(int)right_lines.size(); i++)
        {
            slope_sum = slope_sum + right_lines[i][0];
            y_int_sum = y_int_sum + right_lines[i][1];
        }
        float slope_right_avg = slope_sum / right_lines.size();
        float y_int_right_avg = y_int_sum / right_lines.size();

        slope_sum = 0;
        y_int_sum = 0;
    
        // average left
        for (i=0; i<(int)left_lines.size(); i++)
        {
            slope_sum = slope_sum + left_lines[i][0];
            y_int_sum = y_int_sum + left_lines[i][1];
        }
        float slope_left_avg = slope_sum / left_lines.size();
        float y_int_left_avg = y_int_sum / left_lines.size();

        //Filtering lines
        const double alpha = this->get_parameter("filter_parameter").as_double();  // Peso del valore corrente

        if (previous_slope_left_avg!= 0 && !std::isnan(previous_slope_left_avg)){
        slope_left_avg = (float) (alpha * slope_left_avg + (1 - alpha) * previous_slope_left_avg);
        }
        previous_slope_left_avg = slope_left_avg;

        if (previous_slope_right_avg!= 0 && !std::isnan(previous_slope_right_avg)){
        slope_right_avg = (float) (alpha * slope_right_avg + (1 - alpha) * previous_slope_right_avg);
        }
        previous_slope_right_avg = slope_right_avg;

        if (previous_y_int_left_avg!= 0 && !std::isnan(previous_y_int_left_avg)){
        y_int_left_avg = (float) (alpha * y_int_left_avg + (1 - alpha) * previous_y_int_left_avg);
        }
        previous_y_int_left_avg = y_int_left_avg;

        if (previous_y_int_right_avg!= 0 && !std::isnan(previous_y_int_right_avg)){
        y_int_right_avg = (float) (alpha * y_int_right_avg + (1 - alpha) * previous_y_int_right_avg);
        }
        previous_y_int_right_avg = y_int_right_avg;

        std::vector<float> lines_avg = {slope_left_avg, y_int_left_avg, slope_right_avg, y_int_right_avg};

        return lines_avg;
    }

     private:

        int previous_x_;
        float previous_slope_left_avg;
        float previous_slope_right_avg;
        float previous_y_int_left_avg;
        float previous_y_int_right_avg;
        image_transport::Subscriber lane_sub_;

        void callback(const Image::ConstSharedPtr & msg) 
        {   
            //Image frame from topic
            cv::Mat image(msg->height, msg->width, CV_8UC3, (void *)(msg->data.data()));

            cv::String windowName = "Lane"; //Name of the window

            // apply image blurring
            cv::Mat image_blurred;
            cv::GaussianBlur(image, image_blurred, cv::Size(81, 81), 0);

            // Convert image in HSV format
            cv::Mat hsvImage;
            cv::cvtColor(image_blurred, hsvImage, cv::COLOR_BGR2HSV);

            // Green range for HSV format
            cv::Scalar lowerGreen = cv::Scalar(35, 80, 40); // Green lower bound HSV
            cv::Scalar upperGreen = cv::Scalar(90, 255, 255); // Green upper bound HSV

            // Create a mask for green
            cv::Mat greenMask;
            cv::inRange(hsvImage, lowerGreen, upperGreen, greenMask);
            
            //Set White pixel for all green pixel
            cv::Mat resultImage = image.clone();
            resultImage.setTo(cv::Scalar(255, 255, 255), greenMask);

            // apply image thresholding - Make a black&white image
            cv::Mat image_thresh;
            threshold(resultImage, image_thresh, 150, 255, cv::THRESH_BINARY); 

            // apply image Canny
            cv::Mat image_canny;
            cv::Canny(resultImage, image_canny, 400, 500, 7, false);

            //Create mask for the region of interest for lane keeping
            int height = image.rows;
            int width = image.cols;
            cv::Mat image_mask;
            image_mask = cv::Mat::zeros(cv::Size(width, height),CV_8UC1);
            std::vector<cv::Point> fillContSingle;

            // add all points of the contour to the vector
            vector<cv::Point> vertices{cv::Point(0, height), 
                                    cv::Point(0, (int)height*0.5), 
                                    cv::Point((int)width/2, (int)height*0.35),
                                    cv::Point(width, (int)height*0.5), 
                                    cv::Point(width, height)};

            vector<vector<cv::Point>> pts{vertices};

            cv::fillPoly(image_mask, pts, cv::Scalar(255, 255, 255));
            cv::Mat image_isolated;
            cv::bitwise_and(image_canny, image_mask, image_isolated);

            // extract all the lines

            // Show on windows the result image
            //cv::imshow("Result Image",image_isolated);
            //cv::waitKey(1); 

            vector<cv::Vec4i> lines;
            cv::HoughLinesP(image_isolated, lines, 2, CV_PI/180, 100, 100, 5);

            // average slope, y_int of left and right line
            vector<float> lines_avg;
            lines_avg = average(lines);

            std::vector<double> control_result = display_lines(image, lines_avg);

            auto twist_msg  = geometry_msgs::msg::TwistStamped();
            twist_msg.twist.angular.z = control_result[1];
            twist_msg.header.frame_id = "rover/base_footprint";

            //Publish control message
            ctrl_publisher_->publish(twist_msg);

            //cv::namedWindow(windowName); // Create a window
            //imshow(windowName, image); // Show our image inside the created window.
            //cv::waitKey(1); // Wait for any keystroke in the window
    
        }
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr ctrl_publisher_;
        rclcpp::Service<SetBool>::SharedPtr enable_server_;
        bool is_on_ = false;   

};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    // create a ros2 node
    auto node = std::make_shared<LaneSubscriber>();
    // process ros2 callbacks until receiving a SIGINT (ctrl-c)
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}