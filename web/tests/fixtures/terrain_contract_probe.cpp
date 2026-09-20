#include "rubi_control_core/gazebo_terrain_policy_adapter.hpp"
#include "rubi_gazebo_plugins/velocity_input_arbitration.hpp"
#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>

namespace rc=rubi_control_core;
namespace rg=rubi_gazebo_plugins;

struct FakePolicy : rc::GazeboTerrainPolicyInterface {
  rc::GazeboTerrainLatent latent{};
  rc::Action raw{};
  FakePolicy() { for(size_t i=0;i<32;i++) latent[i]=100.0f+float(i); }
  rc::GazeboTerrainLatent encode(const rc::GazeboTerrainHistory&) override {return latent;}
  rc::Action act(const rc::GazeboTerrainPolicyInput&) override {return raw;}
};

template<class T> void arr(const T& a) {
  std::cout<<'['; bool first=true;
  for(auto v:a){if(!first)std::cout<<',';std::cout<<v;first=false;}std::cout<<']';
}
template<class T> void field(const char* key,const T& a) {
  std::cout<<'"'<<key<<"\":";arr(a);std::cout<<',';
}
bool first_record=true;
void dump(const std::string& name, const std::string& operation,
          const rc::GazeboTerrainPolicyAdapter& c, const rc::RobotState& s,
          const std::array<double,3>& raw_command,
          const rc::UserCommand& cmd, const FakePolicy& p,
          const rc::ControllerOutput& o) {
  if(!first_record)std::cout<<",\n";first_record=false;
  std::cout<<"{\"name\":\""<<name<<"\",\"operation\":\""<<operation<<"\",\"state\":{";
  field("q",s.joint_position);field("qd",s.joint_velocity);
  field("quat",s.base_orientation_xyzw);
  std::cout<<"\"omega\":";arr(s.base_angular_velocity);std::cout<<"},";
  field("rawCommand",raw_command);
  field("command",std::array<double,3>{cmd.linear_x,cmd.linear_y,cmd.angular_z});
  field("mockLatent",p.latent);field("mockAction",p.raw);
  std::cout<<"\"expected\":{";
  field("observation",c.last_observation());field("actor",c.last_actor_observation());
  field("history",c.history());field("input",c.last_policy_input());
  field("rawAction",c.last_network_output());field("action",c.held_action());
  field("target",o.target_position);field("effort",o.effort);
  std::cout<<"\"phase\":"<<c.gait_phase()<<",\"tick\":"<<c.physics_tick_count()
    <<",\"inferenceCount\":"<<c.inference_count()<<",\"ready\":"
    <<(c.walk_ready_complete()?"true":"false")<<",\"mode\":"<<int(c.mode())
    <<",\"inferenceRan\":"<<(o.inference_ran?"true":"false")<<"}}";
}

int main() {
  std::cout<<std::setprecision(17);
  std::cout<<"{\"sourceCommit\":\"8991543654f2d608b9eb722906dd7f2740611ed6\","
    <<"\"generator\":\"Original C++ GazeboTerrainPolicyAdapter and terrain_navigation_command; fake networks\",\"records\":[\n";
  auto fake=std::make_shared<FakePolicy>();
  rc::GazeboTerrainPolicyAdapter controller(fake);
  rc::RobotState state;
  state.joint_position=rc::GazeboTerrainContract{}.default_pose;
  rc::UserCommand cmd;
  std::array<double,3> raw_command{0,0,0};
  auto command=[&](std::array<double,3> v){
    raw_command=v;
    auto mapped=rg::terrain_navigation_command({v[0],v[1],v[2]});
    cmd.linear_x=mapped.linear_x;cmd.linear_y=mapped.linear_y;
    cmd.angular_z=mapped.angular_z;++cmd.command_sequence;
  };
  auto step=[&](){return controller.update(state,cmd);};
  cmd.requested_mode=rc::ControllerMode::kWalkReady;cmd.mode_sequence=1;
  for(int i=0;i<252;i++){
    auto out=step();
    if(i==0||i==125||i==250||i==251)
      dump("ready_"+std::to_string(i+1),"ready_tick",controller,state,raw_command,cmd,*fake,out);
  }
  cmd.requested_mode=rc::ControllerMode::kPolicyOn;cmd.mode_sequence=2;
  auto infer=[&](const std::string& name){
    rc::ControllerOutput out;
    for(int i=0;i<5;i++){out=step();if(out.inference_ran)break;}
    if(!out.inference_ran){std::cerr<<"No inference within five ticks";std::exit(2);}
    dump(name,"advance_to_inference",controller,state,raw_command,cmd,*fake,out);
  };
  state.base_angular_velocity={4,8,12};
  command({.25,-.1,.4});fake->raw.fill(100);
  infer("nominal_positive_clip");

  for(size_t i=0;i<6;i++){
    state.joint_position[i]+=((i%2)?-1:1)*.1*double(i+1);
    state.joint_velocity[i]=((i%2)?-1:1)*double(i+1);
    fake->raw[i]=(i%2)?-100:100;
  }
  state.base_orientation_xyzw={0,std::sin(.3),0,std::cos(.3)};
  state.base_angular_velocity={4,-8,12};command({2,-1,-3});
  infer("pitched_dynamic_signed_clip");

  state.joint_position=rc::GazeboTerrainContract{}.default_pose;
  state.joint_velocity.fill(0);
  state.base_orientation_xyzw={std::sqrt(.5),0,0,std::sqrt(.5)};
  state.base_angular_velocity={0,0,0};command({0,0,0});
  fake->raw={.25f,-.25f,.5f,-.5f,.75f,-.75f};
  infer("roll90_history_feedback");

  state.base_orientation_xyzw={0,0,std::sqrt(2.),std::sqrt(2.)};
  fake->raw.fill(0);command({-.25,.2,-.8});
  infer("nonunit_yaw90");

  state.base_orientation_xyzw={0,0,0,1};
  // Advance past history fill and one complete gait cycle.
  for(int i=0;i<48;i++) {
    command({double(i%5)*.1,0,0});
    rc::ControllerOutput out;
    for(int j=0;j<5;j++){out=step();if(out.inference_ran)break;}
    dump("long_sequence_"+std::to_string(i),"advance_to_inference",controller,state,raw_command,cmd,*fake,out);
  }

  cmd.requested_mode=rc::ControllerMode::kTorqueOff;++cmd.mode_sequence;
  auto off=step();dump("off_preserves_history","off_tick",controller,state,raw_command,cmd,*fake,off);
  cmd.requested_mode=rc::ControllerMode::kPolicyOn;++cmd.mode_sequence;
  infer("policy_resume_preserves_phase");

  cmd.reset=true;cmd.reset_sequence=1;
  auto reset=step();dump("full_reset","reset",controller,state,raw_command,cmd,*fake,reset);
  std::cout<<"\n]}\n";
}
