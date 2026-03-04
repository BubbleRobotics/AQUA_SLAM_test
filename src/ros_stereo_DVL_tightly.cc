#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>

int main(){

    auto main_node = rclcpp::Node::make_shared("aqua_slam_main_node");

    // Idea: pass this node into the SLAM object as it is created (modified System::System constructor)

};