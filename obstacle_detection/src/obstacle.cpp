#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

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

    this->declare_parameter("Obstacle_threshold", 1500.0);

  }

protected:
  void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    //converto il messaggio in Valore cv::mat
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1); //immagine di profondità

    //estrapolo grandezza matrice cv::mat
    int x = cv_ptr->image.cols;
    int y = cv_ptr->image.rows;
        
    // Definisco la zona di interesse
    int roiSize = 500;
    int roiX = x / 2 - roiSize / 2;
    int roiY = y / 2 - roiSize / 2;   

    // Assicuro che la zona di interesse sia completamente contenuta nell'immagine
    roiX = std::max(0, std::min(roiX, x - roiSize));
    roiY = std::max(0, std::min(roiY, y - roiSize));

    // Estraggo la zona di interesse
    cv::Mat roi = cv_ptr->image(cv::Rect(roiX, roiY, roiSize, roiSize));
    
    // Crea una maschera per i valori non zero - prendo solo i valori con confidence
    cv::Mat nonZeroMask = (roi > 0);
    // Calcolo la media dei valori di profondità nella zona di interesse
    double meanDepth = cv::mean(roi,nonZeroMask)[0];

    // Grafico il rettangolo di interesse sulla copia dell'immagine
    cv::Mat imageWithROI = cv_ptr->image.clone();
    cv::rectangle(imageWithROI, cv::Rect(roiX, roiY, roiSize, roiSize), cv::Scalar(0, 255, 0), 2);

    // Mostro l'immagine con il rettangolo di interesse
    //cv::namedWindow(OPENCV_WINDOW);
    //cv::imshow(OPENCV_WINDOW, imageWithROI);
    //cv::waitKey(1);

    // Output the measure
    //std::cout << "Valore di profondità al pixel (" << x << ", " << y << "): " << depthValue << std::endl;
    // Output della media dei valori di profondità nella zona di interesse
    std::cout << "Media dei valori di profondità nella zona di interesse: " << meanDepth << std::endl;
    
    // Verifica se la distanza media è inferiore a "param" e stampa il messaggio appropriato
    double obstacle_threshold = this->get_parameter("Obstacle_threshold").as_double();

    if (meanDepth < obstacle_threshold) {
        std::cout << "OBSTACLE DETECTED" << std::endl;
    }
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mDepthSub;
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