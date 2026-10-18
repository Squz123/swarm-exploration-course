#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <array>
#include <vector>
#include <algorithm>
#include <cmath>

struct Segment {double start,T;std::array<double,3> a,b;};
class Server {
  ros::NodeHandle nh;
  ros::Subscriber sub;
  std::array<ros::Publisher,6> pubs;
  ros::Publisher ready_pub,gap_pub;
  std::array<std::vector<Segment>,6> histories;
  double epoch=0.,max_gap=0.,previous=0.,last_status=0.;
  bool configured=false;
public:
  Server(){
    for(int i=0;i<6;++i)pubs[i]=nh.advertise<quadrotor_msgs::PositionCommand>("/uav"+std::to_string(i+2)+"/position_cmd",1);
    ready_pub=nh.advertise<std_msgs::Bool>("/course_story1/server_ready",1,true);
    gap_pub=nh.advertise<std_msgs::Float64>("/course_story1/max_command_gap",1,true);
    sub=nh.subscribe("/course_story1/trajectories",1,&Server::update,this,ros::TransportHints().tcpNoDelay());
  }
  void update(const std_msgs::Float64MultiArray::ConstPtr& msg){
    // Payload: epoch, then six repetitions of count + count *
    // [start, duration, start xyz, end xyz]. Validate before replacing anything.
    const auto& d=msg->data; if(d.empty())return;
    std::array<std::vector<Segment>,6> candidate;size_t at=1;
    for(int i=0;i<6;++i){
      if(at>=d.size())return;
      int n=static_cast<int>(d[at++]); if(n<1||n>10000||at+8*n>d.size())return;
      double last=-1.;
      for(int k=0;k<n;++k){
        Segment s;s.start=d[at++];s.T=d[at++];
        for(int j=0;j<3;++j)s.a[j]=d[at++];
        for(int j=0;j<3;++j)s.b[j]=d[at++];
        if(!std::isfinite(s.start)||!std::isfinite(s.T)||s.T<=0||s.start<last-1e-7)return;
        for(int j=0;j<3;++j)if(!std::isfinite(s.a[j])||!std::isfinite(s.b[j]))return;
        last=s.start+s.T;candidate[i].push_back(s);
      }
    }
    if(at!=d.size()||!std::isfinite(d[0]))return;
    epoch=d[0];histories=std::move(candidate);configured=true;
  }
  void spin(){
    ros::WallRate rate(100);
    while(ros::ok()){
      ros::spinOnce();const double now=ros::Time::now().toSec();
      if(configured){
        if(previous>0)max_gap=std::max(max_gap,now-previous);previous=now;
        const double t=now-epoch;
        for(int i=0;i<6;++i){
          quadrotor_msgs::PositionCommand cmd;
          cmd.header.stamp=ros::Time::now();cmd.header.frame_id="world";
          cmd.trajectory_id=1;cmd.trajectory_flag=quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
          std::array<double,3> p=histories[i].front().a,v{},a{},j{};
          for(const auto& s:histories[i]){
            if(t<s.start)break;
            double u=std::max(0.,std::min(1.,(t-s.start)/s.T));
            double u2=u*u,u3=u2*u,u4=u3*u,u5=u4*u;
            for(int k=0;k<3;++k){
              double d=s.b[k]-s.a[k];p[k]=s.a[k]+(10*u3-15*u4+6*u5)*d;
              v[k]=u<1?(30*u2-60*u3+30*u4)*d/s.T:0.;
              a[k]=u<1?(60*u-180*u2+120*u3)*d/(s.T*s.T):0.;
              j[k]=u<1?(60-360*u+360*u2)*d/(s.T*s.T*s.T):0.;
            }
          }
          cmd.position.x=p[0];cmd.position.y=p[1];cmd.position.z=p[2];
          cmd.velocity.x=v[0];cmd.velocity.y=v[1];cmd.velocity.z=v[2];
          cmd.acceleration.x=a[0];cmd.acceleration.y=a[1];cmd.acceleration.z=a[2];
          cmd.jerk.x=j[0];cmd.jerk.y=j[1];cmd.jerk.z=j[2];pubs[i].publish(cmd);
        }
      }
      if(now-last_status>.5){
        bool ready=true;for(const auto& p:pubs)ready=ready&&p.getNumSubscribers()>0;
        std_msgs::Bool r;r.data=ready;ready_pub.publish(r);
        std_msgs::Float64 g;g.data=max_gap;gap_pub.publish(g);last_status=now;
      }
      rate.sleep();
    }
  }
};
int main(int argc,char** argv){ros::init(argc,argv,"course_story1_trajectory_server");Server server;server.spin();return 0;}
