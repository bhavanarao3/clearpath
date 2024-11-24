#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include "debris_detection.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

// Test fixture class
class DebrisDetectorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Create node and initialize debris detector
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("debris_detector_test_node");
    debris_detector_ = std::make_shared<DebrisDetector>();
  }

  void TearDown() override {
    rclcpp::shutdown();
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<DebrisDetector> debris_detector_;
};

// // Test for debris detection and robot movement
// TEST_F(DebrisDetectorTest, DetectDebrisAndMove) {
//   // Simulate image and odometry data
//   sensor_msgs::msg::Image::SharedPtr image_msg = std::make_shared<sensor_msgs::msg::Image>();
//   nav_msgs::msg::Odometry::SharedPtr odom_msg = std::make_shared<nav_msgs::msg::Odometry>();

//   // Call the image and odometry processing callbacks
//   debris_detector_->process_image_callback(image_msg);
//   debris_detector_->process_odometry_callback(odom_msg);
  
//   // Simulate debris detection (this part needs the logic to detect debris, which can set the flag)
//   // For example, debris_detected can be set as true after processing the image
//   // Let's assume the detection logic is inside process_image_callback
//   bool debris_handled = debris_detector_->detect_and_handle_debris();
  
//   // Use getter functions to check the internal state
//   EXPECT_EQ(debris_detector_->is_debris_detected(), true);
//   EXPECT_TRUE(debris_handled);

//   // Check that the orientation and movement flags are updated correctly
//   EXPECT_NE(debris_detector_->get_current_orientation(), 0.0);  // Assuming orientation should change
// }

// Test for processing image callback and checking image data
TEST_F(DebrisDetectorTest, ProcessImageCallback) {
  // Mock image data
  sensor_msgs::msg::Image::SharedPtr image_msg = std::make_shared<sensor_msgs::msg::Image>();
  image_msg->height = 640;
  image_msg->width = 480;
  image_msg->encoding = "bgr8";  // Set the encoding to a known type
  image_msg->step = image_msg->width * 3;  // For a "bgr8" image, each pixel has 3 bytes (BGR)
  image_msg->data.resize(image_msg->step * image_msg->height);  // Allocate memory for image data

  // Process the image callback
  debris_detector_->process_image_callback(image_msg);

  // Check that the current image is updated using getter function
  const cv::Mat& current_image = debris_detector_->get_current_image();
  EXPECT_EQ(current_image.cols, 480);
  EXPECT_EQ(current_image.rows, 640);
  EXPECT_EQ(current_image.channels(), 3);  // Check if it's a 3-channel image (for BGR)
}


// Test for processing odometry callback and checking orientation
TEST_F(DebrisDetectorTest, ProcessOdometryCallback) {
  // Mock odometry data with some orientation
  nav_msgs::msg::Odometry::SharedPtr odom_msg = std::make_shared<nav_msgs::msg::Odometry>();
  odom_msg->pose.pose.orientation.z = 1.0;  // Simulate some orientation value

  // Process the odometry callback
  debris_detector_->process_odometry_callback(odom_msg);

  // Check that the current orientation is updated
  EXPECT_EQ(debris_detector_->get_current_orientation(), 1.0);
}

// // Test the navigate_to_debris function (whether it sets the right movement flags)
// TEST_F(DebrisDetectorTest, NavigateToDebris) {
//   // Before calling navigate_to_debris, check initial state
//   EXPECT_FALSE(debris_detector_->is_debris_detected());

//   // Simulate detecting debris
//   debris_detector_->process_image_callback(std::make_shared<sensor_msgs::msg::Image>());
  
//   // Normally, debris detection will happen in process_image_callback.
//   // We simulate it here by calling the method
//   debris_detector_->detect_and_handle_debris();  // This should set `debris_detected_` to true

//   // Now call navigate_to_debris
//   debris_detector_->navigate_to_debris();

//   // After calling navigate_to_debris, check if movement flags are set correctly
//   EXPECT_TRUE(debris_detector_->is_debris_detected());
//   EXPECT_TRUE(debris_detector_->get_current_orientation() != 0.0);  // Check that orientation is updated
// }

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
