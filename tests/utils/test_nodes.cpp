// Copyright (c) 2025, Błażej Szargut.
// All rights reserved.
//
// SPDX-License-Identifier: BSD-3-Clause

#include "test_nodes.hpp"

template <typename T>
TestSubscriber<T>::TestSubscriber(std::string node_name, std::string topic_name) : Node(node_name) {
    _sub = this->create_subscription<T>(
        topic_name, 10,
        std::bind(&TestSubscriber::callback, this, std::placeholders::_1));
}

template <typename T>
void TestSubscriber<T>::callback(const typename T::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(_mutex);
    _last_msg = msg;
    _message_count.fetch_add(1, std::memory_order_acq_rel);
    _cv.notify_all();
}

template <typename T>
std::shared_ptr<T> TestSubscriber<T>::wait_for_message(std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lk(_mutex);
    if (_cv.wait_for(lk, timeout, [this]() { return _last_msg != nullptr; })) {
        return _last_msg;
    } else {
        return nullptr;
    }
}

template <typename T>
float TestSubscriber<T>::measurePubRate(std::chrono::seconds duration) {
    _message_count.store(0, std::memory_order_release);

    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    std::this_thread::sleep_for(duration);
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    size_t final_count = _message_count.load(std::memory_order_acquire);
    std::chrono::duration<float> measured_duration = end - start;
    float duration_s = measured_duration.count();

    if (duration_s > 0.0f) {
        return static_cast<float>(final_count) / duration_s;
    } else {
        return 0.0f;
    }
}

// Explicit template instantiation for used message types
template class TestSubscriber<sensor_msgs::msg::JointState>;
template class TestSubscriber<std_msgs::msg::Float32MultiArray>;

template <typename T>
TestPublisher<T>::TestPublisher(std::string node_name, std::string topic_name) : Node(node_name) {
    _pub = this->create_publisher<T>(topic_name, 10);
}

template <typename T>
void TestPublisher<T>::publish(const typename T::SharedPtr msg) {
    // msg->header.stamp = this->now();
    _pub->publish(*msg);
}

template <typename T>
void TestPublisher<T>::publishEvery(const typename T::SharedPtr msg, std::chrono::milliseconds interval, std::chrono::seconds duration) {
    auto iterations = std::make_shared<size_t>(
        static_cast<size_t>(duration.count() * 1000 / interval.count())
    );

    _timer = this->create_wall_timer(
        interval,
        [this, msg, iterations]() {
            if (*iterations == 0) {
                this->_timer->cancel();
                return;
            }

            this->publish(msg);
            --(*iterations);
        }
    );
}

// Explicit template instantiation for used message types
template class TestPublisher<sensor_msgs::msg::Joy>;
template class TestPublisher<std_msgs::msg::Float32MultiArray>;