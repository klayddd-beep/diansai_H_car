// Fire mission state machine. Arena coordinates are dm; TF/control coordinates are m.
#include <algorithm>
#include <cmath>
#include <queue>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

struct P { double x, y; };
class FireMissionManager : public rclcpp::Node {
public:
  FireMissionManager() : Node("fire_mission_manager") {
    declare_parameter<double>("arena_origin_map_x_m", 0.0); declare_parameter<double>("arena_origin_map_y_m", 0.0); declare_parameter<double>("arena_origin_map_yaw_deg", 0.0);
    declare_parameter<double>("home_x_dm", 13.5); declare_parameter<double>("home_y_dm", 2.5); declare_parameter<double>("standoff_dm", 4.0); declare_parameter<double>("arrival_tol_dm", 1.2); declare_parameter<double>("laser_on_s", 2.1); declare_parameter<double>("grid_dm", 1.0); declare_parameter<double>("safety_margin_dm", 1.5);
    declare_parameter<std::vector<double>>("obstacles_dm", std::vector<double>{});
    ox_=get_parameter("arena_origin_map_x_m").as_double(); oy_=get_parameter("arena_origin_map_y_m").as_double(); yaw0_=get_parameter("arena_origin_map_yaw_deg").as_double()*M_PI/180.; home_={get_parameter("home_x_dm").as_double(),get_parameter("home_y_dm").as_double()}; standoff_=get_parameter("standoff_dm").as_double(); tol_=get_parameter("arrival_tol_dm").as_double(); laser_s_=get_parameter("laser_on_s").as_double(); grid_=get_parameter("grid_dm").as_double(); margin_=get_parameter("safety_margin_dm").as_double(); obs_=get_parameter("obstacles_dm").as_double_array();
    tfbuf_=std::make_shared<tf2_ros::Buffer>(get_clock()); tfl_=std::make_shared<tf2_ros::TransformListener>(*tfbuf_);
    target_=create_publisher<std_msgs::msg::Float32MultiArray>("/target_position",10); laser_=create_publisher<std_msgs::msg::String>("/laser_command",10); status_=create_publisher<std_msgs::msg::String>("/fire_mission_status",10);
    fire_=create_subscription<std_msgs::msg::Float32MultiArray>("/fire_event",10,[this](std_msgs::msg::Float32MultiArray::SharedPtr m){ onFire(*m); });
    laser_status_=create_subscription<std_msgs::msg::String>("/laser_status",10,[this](std_msgs::msg::String::SharedPtr m){ laser_state_=m->data; });
    timer_=create_wall_timer(std::chrono::milliseconds(100),[this](){ tick(); }); report("ready");
  }
private:
  enum class S {IDLE, DRIVE_FIRE, LASER_ON, DRIVE_HOME, SAFE_STOP}; S state_{S::IDLE};
  bool pose(P &p, double &yaw) { try { auto t=tfbuf_->lookupTransform("map","laser_link",tf2::TimePointZero); tf2::Quaternion q; tf2::fromMsg(t.transform.rotation,q); double r,pi; tf2::Matrix3x3(q).getRPY(r,pi,yaw); const double dx=t.transform.translation.x-ox_,dy=t.transform.translation.y-oy_, c=std::cos(yaw0_),s=std::sin(yaw0_); p.x=(c*dx+s*dy)*10.; p.y=(-s*dx+c*dy)*10.; return true;} catch(const tf2::TransformException &e){RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"TF unavailable: %s",e.what());return false;} }
  bool blocked(double x,double y) const { if(x<0||x>48||y<0||y>40) return true; for(size_t i=0;i+3<obs_.size();i+=4) if(x>=obs_[i]-margin_&&x<=obs_[i+2]+margin_&&y>=obs_[i+1]-margin_&&y<=obs_[i+3]+margin_) return true; return false; }
  std::vector<P> plan(P a,P b) const { const int nx=int(48/grid_)+1, ny=int(40/grid_)+1; auto idx=[nx](int x,int y){return y*nx+x;}; auto cell=[this](P p){return std::pair<int,int>{int(std::round(p.x/grid_)),int(std::round(p.y/grid_))};}; const auto start=cell(a), goal=cell(b); const int sx=start.first,sy=start.second,gx=goal.first,gy=goal.second; if(sx<0||sx>=nx||sy<0||sy>=ny||gx<0||gx>=nx||gy<0||gy>=ny||blocked(b.x,b.y)) return {}; std::vector<int> prev(nx*ny,-1); std::queue<int> q; q.push(idx(sx,sy)); prev[idx(sx,sy)]=idx(sx,sy); const int dx[4]={1,-1,0,0},dy[4]={0,0,1,-1}; while(!q.empty()){int u=q.front();q.pop();int x=u%nx,y=u/nx;if(x==gx&&y==gy)break;for(int k=0;k<4;k++){int X=x+dx[k],Y=y+dy[k];if(X>=0&&X<nx&&Y>=0&&Y<ny&&prev[idx(X,Y)]<0&&!blocked(X*grid_,Y*grid_)){prev[idx(X,Y)]=u;q.push(idx(X,Y));}}} if(prev[idx(gx,gy)]<0)return{}; std::vector<P> out; for(int u=idx(gx,gy);u!=idx(sx,sy);u=prev[u])out.push_back({(u%nx)*grid_,(u/nx)*grid_});out.push_back(a);std::reverse(out.begin(),out.end());out.back()=b; return out; }
  void onFire(const std_msgs::msg::Float32MultiArray&m) { if(m.data.size()<2)return; if(state_!=S::IDLE){report(current_status_);return;} firep_={m.data[0],m.data[1]}; P cur;double y;if(!pose(cur,y)){fail("tf_lost");return;} std::vector<P> candidates={{firep_.x-standoff_,firep_.y},{firep_.x+standoff_,firep_.y},{firep_.x,firep_.y-standoff_},{firep_.x,firep_.y+standoff_}}; for(auto c:candidates){auto p=plan(cur,c);if(!p.empty()){path_=p;wp_=0;state_=S::DRIVE_FIRE;report("enroute");return;}} fail("unreachable"); }
  void send(P p, double yaw) { const double c=std::cos(yaw0_),s=std::sin(yaw0_); double xm=ox_+(c*p.x-s*p.y)/10.,ym=oy_+(s*p.x+c*p.y)/10.; std_msgs::msg::Float32MultiArray m;m.data={float(xm*100),float(ym*100),0.f,float(yaw*180/M_PI)};target_->publish(m); }
  void tick() { if(state_==S::IDLE||state_==S::SAFE_STOP)return; P cur;double yaw;if(!pose(cur,yaw)){fail("tf_lost");return;} if(state_==S::LASER_ON){if(laser_state_=="timeout_off"){fail("laser_timeout");return;} if((now()-laser_start_).seconds()>=laser_s_){std_msgs::msg::String m;m.data="OFF";laser_->publish(m);path_=plan(cur,home_);if(path_.empty()){fail("home_unreachable");return;}wp_=0;state_=S::DRIVE_HOME;report("returning");}return;} if(wp_>=path_.size()){if(state_==S::DRIVE_FIRE){laser_state_.clear();std_msgs::msg::String m;m.data="ON";laser_->publish(m);laser_start_=now();state_=S::LASER_ON;report("extinguishing");}else{state_=S::IDLE;report("done");}return;} P target=path_[wp_]; if(std::hypot(cur.x-target.x,cur.y-target.y)<tol_){wp_++;return;} double desired=std::atan2(target.y-cur.y,target.x-cur.x)+yaw0_;send(target,desired); }
  void fail(const std::string&s){state_=S::SAFE_STOP; P p;double y;if(pose(p,y))send(p,y);report("failed:"+s);} void report(const std::string&s){current_status_=s;std_msgs::msg::String m;m.data=s;status_->publish(m);}
  double ox_,oy_,yaw0_,standoff_,tol_,laser_s_,grid_,margin_;P home_,firep_;std::vector<double>obs_;std::vector<P>path_;size_t wp_{0};std::string laser_state_,current_status_{"ready"};rclcpp::Time laser_start_;std::shared_ptr<tf2_ros::Buffer>tfbuf_;std::shared_ptr<tf2_ros::TransformListener>tfl_;rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr target_;rclcpp::Publisher<std_msgs::msg::String>::SharedPtr laser_,status_;rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr fire_;rclcpp::Subscription<std_msgs::msg::String>::SharedPtr laser_status_;rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char**argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<FireMissionManager>());rclcpp::shutdown();}
