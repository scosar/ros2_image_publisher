#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class ImagePublisher : public rclcpp::Node
{
public:
    ImagePublisher()
        : Node("image_publisher"), count_(0)
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image_topic", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),
            std::bind(&ImagePublisher::publish_image, this));
        
        // Initialize OpenCV video capture
        cap_.open(0); // Open default camera
        // Set resolution lower than FullHD to save bandwidth and to obtain high FPS
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640); 
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Could not open camera.");
        }
    }

private:
    void publish_image()
    {
        if (!cap_.isOpened()) {
            return;
        }

        cv::Mat frame;
        cap_ >> frame; // Capture a new frame

        if (frame.empty()) {
            RCLCPP_WARN(this->get_logger(), "Captured empty frame.");
            return;
        }

        // Convert OpenCV image to ROS image message
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        publisher_->publish(*msg);
        // RCLCPP_INFO(this->get_logger(), "Published image %d", count_++);
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;
    size_t count_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImagePublisher>());
    rclcpp::shutdown();
    return 0;
}
