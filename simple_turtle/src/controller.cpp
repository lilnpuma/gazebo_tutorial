/**
 * @file controller.cpp
 * @author Manu Pillai (manump@umd.edu)
 * @brief
 * @version 0.1
 * @date 2022-12-05
 *
 * MIT License
 * Copyright (c) 2022 Manu Pillai
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#include "../include/simple_turtle/controller.hpp"

#include <numeric>

testudo::Simpleturt::Simpleturt(std::string name = "SimpleTestudo")
    : Node(name) {
  // Create a publisher object.
  pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  // Create a subscriber object.
  sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&Simpleturt::scanCallback, this, std::placeholders::_1));
  // Create a timer.
  pub_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10), std::bind(&Simpleturt::moveForward, this));
  rotate_timer_ = this->create_wall_timer(std::chrono::milliseconds(10),
                                          std::bind(&Simpleturt::rotate, this));
  // Cancel the rotation timer. Start with only the forward motion.
  rotate_timer_->cancel();
  RCLCPP_INFO(this->get_logger(), "Simpleturt node has been started");
}
testudo::Simpleturt::~Simpleturt() {
  stop();
  RCLCPP_INFO(this->get_logger(), "Simpleturt node has been stopped");
}
void testudo::Simpleturt::scanCallback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  // Get average of the left 90 degree range
  auto front =
      std::vector<float>(msg->ranges.begin(), msg->ranges.begin() + (40));
  // Get average of the right 90 degree range
  front.insert(front.end(), msg->ranges.end() - (40), msg->ranges.end());
  // Get average of the front 180 degree range
  auto min_range = *std::min_element(front.begin(), front.end());
  // Check if the minimum range is less than 0.65 m
  if (min_range < 0.65) {
    // Stop the turtlebot
    if (rotate_timer_->is_canceled()) {
      stop();
      // Start rotating
      pub_timer_->cancel();
      rotate_timer_->reset();
    }
  } else {
    // Start moving forward
    if (pub_timer_->is_canceled()) {
      stop();
      pub_timer_->reset();
      rotate_timer_->cancel();
    }
  }
}
void testudo::Simpleturt::moveForward() {
  // Create a message object.
  setVelocity(0.5, 0.0);
  // Publish the message.
  pub_->publish(vel_);
}
void testudo::Simpleturt::rotate() {
  // Create a message object.
  setVelocity(0.0, 0.5);
  // Publish the message.
  pub_->publish(vel_);
}
void testudo::Simpleturt::setVelocity(float linear, float angular) {
  // Set the Velocity of the robot to the geometry message.
  geometry_msgs::msg::Twist msg;
  msg.linear.x = linear;
  msg.angular.z = angular;
  // Set the message to the class variable.
  vel_ = msg;
}
void testudo::Simpleturt::stop() {
  // Stop the robot.
  setVelocity(0.0, 0.0);
  // Publish the message.
  pub_->publish(vel_);
}
