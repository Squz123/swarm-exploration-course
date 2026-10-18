#include <rviz/display.h>
#include <rviz/display_context.h>
#include <rviz/view_manager.h>
#include <rviz/render_panel.h>
#include <OgreRenderWindow.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64.h>

namespace course_story1 {
class RenderStats : public rviz::Display {
  ros::Publisher publisher_;
  ros::WallTime last_;
 protected:
  void onInitialize() override {
    publisher_ = update_nh_.advertise<std_msgs::Float64>("/course_display/render_fps", 1);
    last_ = ros::WallTime::now();
  }
  void update(float, float) override {
    auto now = ros::WallTime::now();
    if ((now-last_).toSec() < 1.0) return;
    last_ = now;
    auto panel = context_->getViewManager()->getRenderPanel();
    if (!panel || !panel->getRenderWindow()) return;
    std_msgs::Float64 msg;
    msg.data = panel->getRenderWindow()->getStatistics().lastFPS;
    publisher_.publish(msg);
  }
};
}
PLUGINLIB_EXPORT_CLASS(course_story1::RenderStats, rviz::Display)
