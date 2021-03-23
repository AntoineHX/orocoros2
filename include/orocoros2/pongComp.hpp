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
    // int input_port_value_;
    // int32_t event_port_value_;
    // int32_t event_port_cb_value_;
    // int32_t op_callback_value_;

    // These ports are `int` ports, and will automatically map to `std_msgs/Int32` when connected to a ROS topic
    // RTT::InputPort<std_msgs::msg::Int32> input_port_;
    RTT::InputPort<bool> incoming_ball_port_;
    RTT::OutputPort<bool> return_ball_port_;
    RTT::OutputPort<bool> miss_ball_port_;
    RTT::OutputPort<std_msgs::msg::Int32> player_action_port_;

    // These ports are `std_msgs::Int32` to show that you can use ROS messages, too.
    // RTT::InputPort<std_msgs::msg::Int32> event_port_;
    // RTT::InputPort<std_msgs::msg::Int32> event_port_cb_;
    // void eventPortCallback();

    // Operations and OperationCallers that are tied to ROS services must share the same callback function definition as ROS services, though
    // (but with non-const reference arguments, no shared pointers).
    // RTT::OperationCaller<void(oe_msgs::srv::Operation::Request&, oe_msgs::srv::Operation::Response&)> op_caller_;
    // void op_callback(oe_msgs::srv::Operation::Request&, oe_msgs::srv::Operation::Response&);
};
}  // namespace orocoros2

#endif
