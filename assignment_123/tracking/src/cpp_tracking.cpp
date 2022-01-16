/*--------------- DONT TOUCH ---------------  */
#include <rclcpp/rclcpp.hpp> // ROS2 library
/*------------------------------------------  */

// Standard libraries
#include <functional>
#include <memory>
#include <iostream>
#include <numeric>
#include <stdio.h>
#include <chrono>
#include <string>

// Topic message types
#include <sensor_msgs/msg/image.hpp> // Message type of input  topic; in this file replace "sensor_msgs::msg::Image"  with output topic of interest
#include <rtsd_interfaces/msg/point2.hpp>   // Message type of output topic; in this file replace "rtsd_interfaces::msg::Point2"    with input  topic of interest

/*--------------- DONT TOUCH ---------------  */
using std::placeholders::_1;
using namespace std;
using namespace std::chrono_literals;
/*------------------------------------------  */



/*--------------- USER INPUT ---------------  */
// Node name
string node_name = "tracking"; 

// Input topic name 
string input_topic_name = "webcam_input";

// Output topic name 
string output_topic_name = "setpoint";

// Publish period 
std::chrono::duration<int64_t, std::micro> publish_period = 500ms;

/*------------------------------------------  */

/*--------------- GLOBAL VARIABLES ---------------  */
float x_rad, y_rad;
/*------------------------------------------------  */

// Create class for node
class SubscriberPublisher : public rclcpp::Node
{
public:
  SubscriberPublisher() : Node(node_name)
  {
    /*--------------------- DONT TOUCH ---------------------  */
    auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    /*------------------------------------------------------  */

    /*--------------------- PARAMETERS ---------------------  */
    this->declare_parameter<std::uint8_t>("threshold", 230); // (parameter1)  replace <std::uint8_t> with desired type
    /*------------------------------------------------------  */


    // Subscriber 
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      input_topic_name,
      default_qos,
      std::bind(&SubscriberPublisher::subscriber_callback, this, _1));


    // Publisher 
    publisher_ = this->create_publisher<rtsd_interfaces::msg::Point2>(
      output_topic_name, 
      default_qos);
    timer_ = this->create_wall_timer(
      publish_period, 
      std::bind(&SubscriberPublisher::publisher_callback, 
      this));
  }

private:

  // FUNCTION FOR SUBSCRIBER 
  void subscriber_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
  {
    /*----------------------------- PARAMETERS ---------------------------  */
    // Store parameter in parameter variable (parameter1)
    long unsigned int threshold; // Replace with desired type
    this->get_parameter("threshold", threshold);
    /*---------------------------------------------------------------------  */


    /*--------------------- DATA PROCESSING PART HERE ---------------------  */
    
    // Initialize parameters
    long unsigned int grayscale;
    std::vector<uint> new_data;
    uint SumX = 0;
    uint SumY = 0;
    uint num = 0;

    // Average the RGB values to grayscale 
    for (uint n = 0; n < msg->data.size()/3; n++){
        grayscale = (msg->data.at(n*3) + msg->data.at(n*3 + 1) + msg->data.at(n*3 + 2)) / 3;
        new_data.push_back(grayscale);
    }

    // Find Centre of Gravity 
    for (uint i=0; i<msg->width; i++)
    {
      for (uint j=0; j<msg->height; j++)
      {
        if (new_data[i + j*msg->width] >= threshold)
        {
          SumX = SumX + i;
          SumY = SumY + j;
          num = num + 1;
        }
      }
    }

    // Calculate sums to find coordinate of index
    if (SumX != 0 || SumY != 0)
    {
      SumX = SumX / num;
      SumY = SumY / num;

      // Convert pixel position to radians
      x_rad = ((float)SumX - ( (float)msg->width / 2) ) / (float)msg->width * 3;
      y_rad = (((float)msg->height / 2) - (float)SumY) / (float)msg->height * 1.5;

      // End-stop
      if (y_rad <= -1.0){y_rad = -1.0};

      // Print data to terminal
      RCLCPP_INFO(this->get_logger(),
      "I see Centre Of Gravity (%d, %d), in radians (%f, %f)",   // Type % to print 
      SumX, SumY, x_rad, y_rad);                                 // To-be-printed variables
    }
    else
    {
      // Reset Jiwy to starting position
      x_rad = 0;
      y_rad = 0;

      // Print data to terminal
      RCLCPP_INFO(this->get_logger(),
      "A Centre of Gravity is not observed");
    }

    /*---------------------------------------------------------------------  */

  }


  // FUNCTION FOR PUBLISHER 
  void publisher_callback()
  {
    /*----------------------------- PARAMETERS ---------------------------  */
    /*---------------------------------------------------------------------  */


    /*--------------------- DATA PROCESSING PART HERE ---------------------  */

    // (BEGIN) Message type
    auto message = rtsd_interfaces::msg::Point2();

    // Initialize parameters

    //////// PLACE PROCESSING CODE HERE IN THE MIDDLE

    // (END) Attach information to message
    message.x = x_rad;
    message.y = y_rad;

    /*---------------------------------------------------------------------  */

    // Print data to terminal
    RCLCPP_INFO(this->get_logger(), 
    "Publishing: (%f, %f)",           // Type % to print 
    message.x, message.y);           // To-be-printed variables

    // Publish attached information to topic
    publisher_->publish(message);
  }

  // Subscription side 
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_; 

  // Publisher side 
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<rtsd_interfaces::msg::Point2>::SharedPtr publisher_;
  size_t count_;
};


// Main function to be executed
int main(int argc, char * argv[])
{
  // Initialization
  rclcpp::init(argc, argv);

  // Start the node
  rclcpp::spin(std::make_shared<SubscriberPublisher>());

  // Allow Ctrl + C to be used to shutdown the node
  rclcpp::shutdown();

  // Neglect
  return 0;
}
