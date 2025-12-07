// Copyright (c) 2025, Błażej Szargut.
// All rights reserved.
//
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

template <typename T>
class TestSubscriber : public rclcpp::Node {
   public:
    TestSubscriber(std::string node_name = "test_subscriber", std::string topic_name = "test_topic");

    void callback(const typename T::SharedPtr msg);

    std::shared_ptr<T> wait_for_message(std::chrono::milliseconds timeout);

    float measurePubRate(std::chrono::seconds duration);

   private:
    rclcpp::Subscription<T>::SharedPtr _sub;
    std::mutex _mutex;
    std::condition_variable _cv;
    std::shared_ptr<T> _last_msg{nullptr};
    std::atomic<size_t> _message_count{0};
};

template <typename T>
class TestPublisher : public rclcpp::Node {
   public:
    TestPublisher(std::string node_name = "test_publisher", std::string topic_name = "test_topic");

    void publish(const typename T::SharedPtr msg);

    void publishEvery(const typename T::SharedPtr msg, std::chrono::milliseconds interval, std::chrono::seconds duration);

   private:
    rclcpp::Publisher<T>::SharedPtr _pub;
    std::mutex _mutex;
    std::condition_variable _cv;
    rclcpp::TimerBase::SharedPtr _timer;
};