#ifndef OROCOROS2_REFEREECOMP_HPP
#define OROCOROS2_REFEREECOMP_HPP

#include <rtt/Component.hpp>
#include <rtt/RTT.hpp>
#include <rtt_ros2_services/rosservice.hpp>
#include <rtt_ros2_topics/rostopic.hpp>
#include <rtt_ros2_params/rosparam.hpp>
#include <std_msgs/typekit/Types.hpp>
#include <oe_msgs/srv/operation.hpp>

namespace orocoros2
{
class refereeComp : public RTT::TaskContext
{
  public:
    explicit refereeComp(const std::string& name);
    virtual ~refereeComp();

    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

    //  ROS Service
    // Operations and OperationCallers that are tied to ROS services must share the same callback function definition as ROS services, though
    // (but with non-const reference arguments, no shared pointers).
    // RTT::OperationCaller<void(oe_msgs::srv::Operation::Request&, oe_msgs::srv::Operation::Response&)> op_caller_;
    void op_callback(oe_msgs::srv::Operation::Request&, oe_msgs::srv::Operation::Response&);


    bool connect_to_topics_; //ROS Topic connection guard

    int current_score_;

    // These ports are `int` ports, and will automatically map to `std_msgs/Int32` when connected to a ROS topic
    RTT::InputPort<bool> player1_miss_port_;
    RTT::InputPort<bool> player2_miss_port_;
    RTT::OutputPort<bool> player1_start_port_;
    RTT::OutputPort<bool> player2_start_port_;
    RTT::OutputPort<std_msgs::msg::Int32> score_port_;

};
}  // namespace orocoros2

#endif
