// Emanuele Arilli - Master's Thesis //
// emanuelearilli@gmail.com          //
// January 2024                      //

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "std_msgs/msg/bool.hpp"

using namespace std::placeholders;

static const std::string OPENCV_WINDOW = "Image window";

class MinimalDepthSubscriber : public rclcpp::Node
{
public:
  MinimalDepthSubscriber() : Node("Obstacle")
  {
    
    // https://github.com/ros2/ros2/wiki/About-Quality-of-Service-Settings

    rclcpp::QoS depth_qos(1);
    depth_qos.keep_last(1);
    depth_qos.best_effort();
    depth_qos.durability_volatile();

    // Create depth map subscriber
    mDepthSub = create_subscription<sensor_msgs::msg::Image>(
      "/camera/aligned_depth_to_color/image_raw", depth_qos, std::bind(&MinimalDepthSubscriber::depthCallback, this, _1));

    //Publisher for topic "rover/obstacle"
    mObstaclePub = create_publisher<std_msgs::msg::Bool>("rover/obstacle", 10);

    this->declare_parameter("Obstacle_threshold", 1500.0);
    this->declare_parameter("Region_of_interest", 500);

  }

protected:
  void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    //Convert msg in cv::mat value
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1); //immagine di profondità

    //eextract cv::mat matrix dimensions
    int x = cv_ptr->image.cols;
    int y = cv_ptr->image.rows;
        
    // Define region of interest
    int region = this->get_parameter("Region_of_interest").as_int();
    int roiSize = region;
    int roiX = x / 2 - roiSize / 2;
    int roiY = y / 2 - roiSize / 2;   

    // Check dimensions
    roiX = std::max(0, std::min(roiX, x - roiSize));
    roiY = std::max(0, std::min(roiY, y - roiSize));

    // Extract region of interest
    cv::Mat roi = cv_ptr->image(cv::Rect(roiX, roiY, roiSize, roiSize));
    
    // Create a mask for non zero value - take only value with confidence
    cv::Mat nonZeroMask = (roi > 0);
    // Take the mean depth value 
    double meanDepth = cv::mean(roi,nonZeroMask)[0];

    // Draw a region of interest rect on the image
    cv::Mat imageWithROI = cv_ptr->image.clone();
    cv::rectangle(imageWithROI, cv::Rect(roiX, roiY, roiSize, roiSize), cv::Scalar(0, 255, 0), 2);

    // Show the image - comment this section for fast code
    cv::namedWindow(OPENCV_WINDOW);
    cv::imshow(OPENCV_WINDOW, imageWithROI);
    cv::waitKey(1);

    // Output mean depth in the region of interest
    std::cout << "Media dei valori di profondità nella zona di interesse: " << meanDepth << std::endl;
    
    // Check if the mean depth is less than obstacle threshold
    double obstacle_threshold = this->get_parameter("Obstacle_threshold").as_double();

    if (meanDepth < obstacle_threshold) {
        std::cout << "OBSTACLE DETECTED" << std::endl;
        //publish bool msg on topic 
        auto obstacle_msg = std_msgs::msg::Bool();
        obstacle_msg.data = true;
        mObstaclePub->publish(obstacle_msg);
    }
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mDepthSub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mObstaclePub;
};

// The main function
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto depth_node = std::make_shared<MinimalDepthSubscriber>();

  rclcpp::spin(depth_node);
  rclcpp::shutdown();
  return 0;
}