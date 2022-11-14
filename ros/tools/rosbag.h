#pragma once

// standard c++
#include <array>
#include <chrono>
#include <string>
#include <thread>
#include <utility>
// ros
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

// return 'true' to continue
// return 'false' to break
template <typename F, size_t... Idx> void static_for_impl(F&& func, std::index_sequence<Idx...>) {
  (func(std::integral_constant<size_t, Idx>()) && ...);
}

template <size_t N, typename F> void static_for(F&& func) {
  static_for_impl(func, std::make_index_sequence<N>());
}

template <size_t N, typename Q, typename K, typename F> void static_switch(const Q& query, const std::array<K, N>& keys, F&& func) {
  static_for<N>([&](auto i) {
    if (query != keys[i.value])
      return true;
    func(i);
    return false;
  });
}

template <size_t I, typename T, typename... Ts> struct index_of {
  static_assert(I <= sizeof...(Ts));
  using type = typename index_of<I - 1, Ts...>::type;
};

template <typename T, typename... Ts> struct index_of<0, T, Ts...> {
  using type = T;
};

template <size_t I, typename... Ts> using index_of_t = typename index_of<I, Ts...>::type;

template <typename... Ts> class bag {
 public:
  constexpr inline static size_t NUM_TOPICS = sizeof...(Ts);

  template <typename... Ss> bag(Ss&&... topics) : topics_{std::forward<Ss>(topics)...} {
    static_assert(sizeof...(Ss) == NUM_TOPICS);
  }

  void open(const std::string& filename, ros::NodeHandle& nh) {
    bag_.open(filename, rosbag::bagmode::Read);
    static_for<NUM_TOPICS>([&](auto i) {
      using msg_type = index_of_t<i.value, Ts...>;
      view_.addQuery(bag_, rosbag::TopicQuery(topics_[i.value]));
      pubs_[i.value] = nh.advertise<msg_type>(topics_[i.value], 1);
      return true;
    });
    view_it_  = view_.begin();
    view_end_ = view_.end();
  }

  bool is_open() const {
    return bag_.isOpen();
  }

  void play_once() {
    if (eof())
      return;
    auto      msg               = *(view_it_++);
    ros::Time curr_msg_bag_time = msg.getTime();
    ros::Time curr_msg_pub_time;
    if (first_play) {
      curr_msg_pub_time = ros::Time::now();
      first_play        = false;
    } else {
      curr_msg_pub_time = prev_msg_pub_time + (curr_msg_bag_time - prev_msg_bag_time);
      ros::Time::sleepUntil(curr_msg_pub_time);
    }
    static_switch(msg.getTopic(), topics_, [&](auto i) {
      using msg_type = index_of_t<i.value, Ts...>;
      auto p_msg     = msg.instantiate<msg_type>();
      pubs_[i.value].publish(p_msg);
    });
    prev_msg_bag_time = curr_msg_bag_time;
    prev_msg_pub_time = curr_msg_pub_time;
  }

  bool eof() {
    return view_it_ == view_end_;
  }

 private:
  std::array<std::string, NUM_TOPICS>    topics_;
  std::array<ros::Publisher, NUM_TOPICS> pubs_;
  rosbag::Bag                            bag_;
  rosbag::View                           view_;
  rosbag::View::const_iterator           view_it_;
  rosbag::View::const_iterator           view_end_;

  ros::Time prev_msg_bag_time;
  ros::Time prev_msg_pub_time;

  bool first_play = true;
};
