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
            lane_sub_ = image_transport::create_subscription(this, "video_frames", std::bind(&LaneSubscriber::callback, this, std::placeholders::_1), "raw");
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

        offset = (int) width / 100 * 2.5;

        for (i=0; i<2; i++)
        {
            slope = lines_avg[0+2*i];
            y_int = lines_avg[1+2*i];

            std::cout << slope << " " << y_int << endl;

            y1 = height;
            y2 = (int) (y1 * (float)3/5);
            x1 = (int) ((y1 - y_int) / slope);
            x2 = (int) ((y2 - y_int) / slope);

            std::cout << x1 << " " << x2 << " " << y1 << " " << y2 << endl;

            cv::line(image, cv::Point (x1,y1), cv::Point (x2,y2), cv::Scalar(255,0,0), 10);
        }

        cv::line(image, cv::Point ((int)width/2 - offset,0), cv::Point ((int)width/2 - offset,480), cv::Scalar(0,0,255), 3, 8, 0);
        cv::line(image, cv::Point ((int)width/2 + offset,0), cv::Point ((int)width/2 + offset,480), cv::Scalar(0,0,255), 3, 8, 0);

        std::vector<int> result = intersection(lines_avg, image);
        int x = result[0];

        std::vector<double> control_result = control_law(x,offset,width); //call the control law for lane keeping
        return control_result;
    }

    std::vector<double> control_law(double x, double offset, double width) {
        // Applica una legge di controllo proporzionale per il mantenimento della corsia
        double kc = this->get_parameter("proportional_kc").as_double();
        double velocity = this->get_parameter("linear_velocity").as_double();

        // Zona morta del volante
        if (x > (width / 2 - offset) && x < (width / 2 + offset)) {
            std::cout << "Dritto" << std::endl;
            double steering_angle = 0.0;
            std::vector<double> law = {velocity, steering_angle};
        return law;
        } else {
            double new_x = x - width / 2; // Sposta il valore zero al centro dello schermo
            double steering_angle = kc * new_x;
            std::cout << steering_angle << std::endl;
            std::vector<double> law = {velocity, steering_angle};
        return law;
        }
    }

    std::vector<int> intersection(vector<float> lines_avg, cv::Mat image)
    {
        float m1 = lines_avg[0];
        float q1 = lines_avg[1];
        float m2 = lines_avg[2];
        float q2 = lines_avg[3];

        int x = (int) (q2 - q1) / (m1 - m2);
        int y = (int) (m1*x) + q1;

        cv::circle(image, cv::Point(x, y), 5, cv::Scalar(0, 0, 255), 10);
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


        std::vector<float> lines_avg = {slope_left_avg, y_int_left_avg, slope_right_avg, y_int_right_avg};

        return lines_avg;
    }

     private:

        image_transport::Subscriber lane_sub_;

        void callback(const Image::ConstSharedPtr & msg) 
        {   
            cv::Mat image(msg->height, msg->width, CV_8UC3, (void *)(msg->data.data()));

            cv::String windowName = "Lane"; //Name of the window

            cv::Mat gray_image;
            cvtColor(image, gray_image, CV_BGR2GRAY );

            // apply image thresholding
            cv::Mat image_thresh;
            threshold(gray_image, image_thresh, 115, 255, cv::THRESH_BINARY); 

            // apply image blurring
            cv::Mat image_blurred;
            cv::GaussianBlur(image_thresh, image_blurred, cv::Size(13, 13), 0);

            // apply image Canny
            cv::Mat image_canny;
            cv::Canny(image_blurred, image_canny, 80, 150, 5, true);


            int height = image.rows;
            int width = image.cols;
            cv::Mat image_mask;
            image_mask = cv::Mat::zeros(cv::Size(width, height),CV_8UC1);
            std::vector<cv::Point> fillContSingle;

            // add all points of the contour to the vector
            vector<cv::Point> vertices{cv::Point(0, height), 
                                    cv::Point(0, (int)height*0.7), 
                                    cv::Point((int)width/2, (int)height*0.55),
                                    cv::Point(width, (int)height*0.7), 
                                    cv::Point(width, height)};

            vector<vector<cv::Point>> pts{vertices};

            cv::fillPoly(image_mask, pts, cv::Scalar(255, 255, 255));
            cv::Mat image_isolated;
            cv::bitwise_and(image_canny, image_mask, image_isolated);

            imshow(windowName, image_isolated);
            // extract all the lines
            vector<cv::Vec4i> lines;
            cv::HoughLinesP(image_isolated, lines, 2, CV_PI/180, 80, 60, 5);

            // average slope, y_int of left and right line
            vector<float> lines_avg;
            lines_avg = average(lines);

            std::vector<double> control_result = display_lines(image, lines_avg);

            auto twist_msg  = geometry_msgs::msg::TwistStamped();
            twist_msg.twist.angular.z = control_result[1];
            twist_msg.header.frame_id = "car1/base_footprint";

            //Publish control message
            ctrl_publisher_->publish(twist_msg);

            cv::namedWindow(windowName); // Create a window
            imshow(windowName, image); // Show our image inside the created window.
            cv::waitKey(1); // Wait for any keystroke in the window
    
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