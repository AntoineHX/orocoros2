#ifndef OROCOROS2_PONGCOMP_HPP
#define OROCOROS2_PONGCOMP_HPP

#include <rtt/Component.hpp>
#include <rtt/RTT.hpp>
// #include <rtt_ros2_services/rosservice.hpp>
#include <rtt_ros2_topics/rostopic.hpp>
#include <rtt_ros2_params/rosparam.hpp>
#include <std_msgs/typekit/Types.hpp>
#include <stdlib.h>//Random gen

namespace orocoros2
{
class pongComp : public RTT::TaskContext
{
  public:
    explicit pongComp(const std::string& name);
    virtual ~pongComp();

    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

    // bool connect_to_topics_; //ROS Topic connection guard

    float hit_chance_;

    RTT::InputPort<bool> incoming_ball_port_;
    RTT::OutputPort<bool> return_ball_port_;
    RTT::OutputPort<bool> miss_ball_port_;
    RTT::OutputPort<std_msgs::msg::Int32> player_action_port_;
};
}  // namespace orocoros2

#endif
