#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <memory>
#include <vector>

// Only RViz subscribes to these copies. Flight and teacher topics are untouched.
template<class Message> class DisplayCopy {
  ros::Publisher publisher_;
  ros::Subscriber subscriber_;
  ros::WallTime last_;
  double interval_;
 public:
  DisplayCopy(ros::NodeHandle& nh, const std::string& source, double hz)
      : interval_(1.0 / hz) {
    publisher_ = nh.advertise<Message>("/course_display" + source, 1, true);
    subscriber_ = nh.subscribe<Message>(source, 1,
      [this](const typename Message::ConstPtr& msg) {
        const auto now = ros::WallTime::now();
        if (last_.isZero() || (now-last_).toSec() >= interval_) {
          last_ = now;
          publisher_.publish(msg);
        }
      }, ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "course_display_relay");
  ros::NodeHandle nh;
  DisplayCopy<nav_msgs::OccupancyGrid> grid(nh, "/exploration_grid", 5.0);
  std::vector<std::unique_ptr<DisplayCopy<visualization_msgs::Marker>>> markers;
  for (int i=2; i<=7; ++i) {
    const auto prefix = "/uav" + std::to_string(i);
    markers.emplace_back(new DisplayCopy<visualization_msgs::Marker>(nh, prefix+"/odom_visualization/robot", 60.0));
    markers.emplace_back(new DisplayCopy<visualization_msgs::Marker>(nh, prefix+"/fov_marker", 30.0));
  }
  ros::spin();
}
