#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <array>
#include <vector>
#include <algorithm>
#include <cmath>
#include <limits>

struct FastSegment { double start,T; double c[6][3]; };
class FastServer {
  ros::NodeHandle nh;
  ros::Subscriber plan_sub,map_sub;
  std::array<ros::Subscriber,6> odom_sub,imu_sub;
  std::array<ros::Publisher,6> publishers;
  ros::Publisher ready_pub,gap_pub,audit_pub;
  std::array<std::vector<FastSegment>,6> histories;
  std::array<Eigen::Vector3d,6> positions;
  std::array<bool,6> observed{};
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud{new pcl::PointCloud<pcl::PointXYZ>};
  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
  double epoch=0,previous=0,max_gap=0,last_status=0,max_speed=0,max_acc=0,max_height=0;
  double min_pair=std::numeric_limits<double>::infinity(),min_obs=min_pair;
  unsigned long odom_count=0,imu_count=0;
  bool configured=false,map_ready=false;
  bool auditing() const {return configured && ros::Time::now().toSec()>=epoch;}
 public:
  FastServer() {
    ready_pub=nh.advertise<std_msgs::Bool>("/course_story1/server_ready",1,true);
    gap_pub=nh.advertise<std_msgs::Float64>("/course_story1/max_command_gap",1,true);
    audit_pub=nh.advertise<std_msgs::Float64MultiArray>("/course_fast/audit",1,true);
    plan_sub=nh.subscribe("/course_fast/polynomials",1,&FastServer::update,this,ros::TransportHints().tcpNoDelay());
    map_sub=nh.subscribe<sensor_msgs::PointCloud2>("/mock_map",1,[this](const sensor_msgs::PointCloud2::ConstPtr& msg){
      if(map_ready)return;
      pcl::fromROSMsg(*msg,*cloud); if(cloud->empty())return;
      tree.setInputCloud(cloud);map_ready=true;
    });
    for(int i=0;i<6;++i) {
      const auto prefix="/uav"+std::to_string(i+2);
      publishers[i]=nh.advertise<quadrotor_msgs::PositionCommand>(prefix+"/position_cmd",1);
      odom_sub[i]=nh.subscribe<nav_msgs::Odometry>(prefix+"/sim/odom",1,
        [this,i](const nav_msgs::Odometry::ConstPtr& msg){
          const auto& p=msg->pose.pose.position;const auto& v=msg->twist.twist.linear;
          positions[i]=Eigen::Vector3d(p.x,p.y,p.z);observed[i]=true;
          if(!auditing())return;
          ++odom_count;max_speed=std::max(max_speed,std::sqrt(v.x*v.x+v.y*v.y+v.z*v.z));max_height=std::max(max_height,p.z);
          for(int j=0;j<6;++j)if(i!=j && observed[j])min_pair=std::min(min_pair,(positions[i]-positions[j]).norm());
          if(map_ready) {
            std::vector<int> ids(1);std::vector<float> ds(1);
            if(tree.nearestKSearch(pcl::PointXYZ(p.x,p.y,p.z),1,ids,ds)>0)min_obs=std::min(min_obs,std::sqrt(double(ds[0])));
          }
        },ros::VoidConstPtr(),ros::TransportHints().tcpNoDelay());
      imu_sub[i]=nh.subscribe<sensor_msgs::Imu>(prefix+"/sim/imu",1,
        [this](const sensor_msgs::Imu::ConstPtr& msg){
          if(!auditing())return;
          const auto& a=msg->linear_acceleration;++imu_count;
          max_acc=std::max(max_acc,std::sqrt(a.x*a.x+a.y*a.y+a.z*a.z));
        },ros::VoidConstPtr(),ros::TransportHints().tcpNoDelay());
    }
  }
  void update(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    if(configured)return; // Entire continuous plan is installed once, before flight.
    const auto& d=msg->data;if(d.empty())return;
    std::array<std::vector<FastSegment>,6> candidate;size_t at=1;
    for(int i=0;i<6;++i) {
      if(at>=d.size())return;int n=static_cast<int>(d[at++]);
      if(n<1 || n>10000 || at+20*n>d.size())return;
      double last=-1;
      for(int j=0;j<n;++j) {
        FastSegment s;s.start=d[at++];s.T=d[at++];
        if(!std::isfinite(s.start)||!std::isfinite(s.T)||s.T<=0 || (j>0 && std::abs(s.start-last)>1e-6))return;
        for(int k=0;k<6;++k)for(int axis=0;axis<3;++axis){s.c[k][axis]=d[at++];if(!std::isfinite(s.c[k][axis]))return;}
        last=s.start+s.T;candidate[i].push_back(s);
      }
    }
    if(at!=d.size()||!std::isfinite(d[0]))return;
    epoch=d[0];histories=std::move(candidate);configured=true;
  }
  double evaluate(const FastSegment& s,double u,int axis,int derivative) {
    double result=0;
    for(int k=5;k>=derivative;--k) {
      double factor=1.;for(int j=0;j<derivative;++j)factor*=k-j;
      result=result*u+s.c[k][axis]*factor;
    }
    return result/std::pow(s.T,derivative);
  }
  void spin() {
    ros::WallRate rate(100);
    while(ros::ok()) {
      ros::spinOnce();const double now=ros::Time::now().toSec();
      if(configured) {
        if(previous>0)max_gap=std::max(max_gap,now-previous);previous=now;
        const double t=now-epoch;
        for(int i=0;i<6;++i) {
          const auto& h=histories[i];const FastSegment* s=&h.front();
          for(const auto& part:h){if(t<part.start)break;s=&part;}
          const double u=std::max(0.,std::min(1.,(t-s->start)/s->T));
          const bool moving=t>=s->start && t<s->start+s->T;
          quadrotor_msgs::PositionCommand cmd;cmd.header.stamp=ros::Time::now();cmd.header.frame_id="world";
          cmd.trajectory_id=3;cmd.trajectory_flag=quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
          cmd.position.x=evaluate(*s,u,0,0);cmd.position.y=evaluate(*s,u,1,0);cmd.position.z=evaluate(*s,u,2,0);
          if(moving) {
            cmd.velocity.x=evaluate(*s,u,0,1);cmd.velocity.y=evaluate(*s,u,1,1);cmd.velocity.z=evaluate(*s,u,2,1);
            cmd.acceleration.x=evaluate(*s,u,0,2);cmd.acceleration.y=evaluate(*s,u,1,2);cmd.acceleration.z=evaluate(*s,u,2,2);
            cmd.jerk.x=evaluate(*s,u,0,3);cmd.jerk.y=evaluate(*s,u,1,3);cmd.jerk.z=evaluate(*s,u,2,3);
          }
          publishers[i].publish(cmd);
        }
      }
      if(now-last_status>.2) {
        bool ready=map_ready;for(int i=0;i<6;++i)ready=ready && observed[i] && publishers[i].getNumSubscribers()>0;
        std_msgs::Bool r;r.data=ready;ready_pub.publish(r);
        std_msgs::Float64 g;g.data=max_gap;gap_pub.publish(g);
        std_msgs::Float64MultiArray audit;
        audit.data={max_speed,max_acc,max_height,min_pair,min_obs,double(odom_count),double(imu_count)};audit_pub.publish(audit);
        last_status=now;
      }
      rate.sleep();
    }
  }
};
int main(int argc,char** argv){ros::init(argc,argv,"course_fast_trajectory_server");FastServer server;server.spin();return 0;}
