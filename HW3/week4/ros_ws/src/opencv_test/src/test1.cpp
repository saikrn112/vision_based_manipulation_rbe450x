// header for ROS core functionalities
#include "rclcpp/rclcpp.hpp"

// including image message type to be able to receive and publish it
#include "sensor_msgs/msg/image.hpp"

// headers regarding the connection between opencv and ROS
#include <image_transport/image_transport.hpp>
#include "cv_bridge/cv_bridge.h"

// OpenCV core functions and various image processing algorithms
#include <opencv2/opencv.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using std::placeholders::_1;

// Defining a class that will be utilize in the "main" function
class ImageSubscriber : public rclcpp::Node
{

	// Declaring pointer to the publisher and subscriber the publish and receive images.
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;

    const cv::Scalar blue_low = cv::Scalar(200,0,0);
    const cv::Scalar blue_high = cv::Scalar(255,0,0);

    const cv::Scalar green_low = cv::Scalar(0,200,0);
    const cv::Scalar green_high = cv::Scalar(0,255,0);

    const cv::Scalar red_low = cv::Scalar(0,0,0);
    const cv::Scalar red_high = cv::Scalar(0,0,255);

    const cv::Scalar pink_low = cv::Scalar(198,190,200);
    const cv::Scalar pink_high = cv::Scalar(255,255,255);

  public:
	// Constructor of the class. The class is derived from the parent class "Node" of rclcpp and
	// the node is called "image_processor", and each time it receive a data, it will call the callbackImage() function
    	ImageSubscriber() : Node("image_processor")
	{
		// Defining the subscriber: it will receive "Image" type data from the topic /camera1/image_raw 
      		subscription_ = this->create_subscription<sensor_msgs::msg::Image>("/camera1/image_raw", 10, std::bind(&ImageSubscriber::callbackImage, this, _1));

	//defining the publisher: it will publish "Image" type data to the "output_image" topic
            publisher_ = this->create_publisher<sensor_msgs::msg::Image>("output_image", 10);
    }

  private:

	// callback function which will be triggered each time the subscriber_ receives new data.
	// The data it receives is of Image type.
	void callbackImage(const sensor_msgs::msg::Image::SharedPtr msg) const
	{    
		//converting the ROS message type data to opencv type
		cv_bridge::CvImagePtr cv_ptr;
		cv_ptr = cv_bridge::toCvCopy(msg,"bgr8" );
		
		// first checks if the image dimensions are at least 60 (otherwise the program would crash trying to draw a circle), and then draw a circle at the 50,50 location with diameter 10, with color red.
//		if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
//			cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

        cv::Mat blue_mask = cv::Mat::zeros(cv_ptr->image.size(),cv_ptr->image.type());
        cv::Mat red_mask = cv::Mat::zeros(cv_ptr->image.size(),cv_ptr->image.type());
        cv::Mat green_mask = cv::Mat::zeros(cv_ptr->image.size(),cv_ptr->image.type());
        cv::Mat pink_mask = cv::Mat::zeros(cv_ptr->image.size(),cv_ptr->image.type());


        cv::inRange(cv_ptr->image,blue_low,blue_high,blue_mask);
        cv::inRange(cv_ptr->image,red_low,red_high,red_mask);
        cv::inRange(cv_ptr->image,green_low,green_high,green_mask);
        cv::inRange(cv_ptr->image,pink_low,pink_high,pink_mask);

        cv::Mat final_mask = cv::Mat::zeros(cv_ptr->image.size(),cv_ptr->image.type());
        //final_mask = blue_mask + red_mask + green_mask + pink_mask;
        final_mask = red_mask;
        
        cv::Mat gray = cv::Mat::zeros(cv_ptr->image.size(),cv_ptr->image.type());
        cv::cvtColor(cv_ptr->image,gray,cv::COLOR_BGR2GRAY);

        auto cv_ret_ptr = std::make_shared<cv_bridge::CvImage>();
        cv_ret_ptr->image = cv::Mat::zeros(gray.size(),gray.type());
        cv_ret_ptr->header = cv_ptr->header;
        cv_ret_ptr->encoding = sensor_msgs::image_encodings::MONO8;

//        RCLCPP_INFO_STREAM(this->get_logger(), "input:" << cv_ptr->image.size() 
//                        << " input_type:" << cv_ret_ptr->image.type()
//                        << " ret:" << cv_ret_ptr->image.size()
//                        << " ret_type:" << cv_ret_ptr->image.type()
//                        << " mask:" << final_mask.size());
        
        RCLCPP_INFO_STREAM(this->get_logger(), "mat:" << cv_ptr->image);
        exit(0);


        //cv::bitwise_not(gray,final_mask,cv_ret_ptr->image);
        cv_ret_ptr->image = red_mask;


		// converting opencv data type to ROS message type
   		//auto msg_to_send = cv_ptr->toImageMsg();
   		auto msg_to_send = cv_ret_ptr->toImageMsg();

		// publishing the image.
   		publisher_->publish(*msg_to_send);
    }
    
};

int main(int argc, char * argv[])
{

	//initialize ROS
	rclcpp::init(argc, argv);

	//create the 
	rclcpp::spin(std::make_shared<ImageSubscriber>());
	rclcpp::shutdown();
  return 0;
}
