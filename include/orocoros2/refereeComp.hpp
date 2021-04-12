#ifndef OROCOROS2_REFEREECOMP_HPP
#define OROCOROS2_REFEREECOMP_HPP

#include <rtt/Component.hpp>
#include <rtt/RTT.hpp>
#include <rtt_ros2_services/rosservice.hpp>
#include <rtt_ros2_topics/rostopic.hpp>
#include <rtt_ros2_params/rosparam.hpp>
#include <std_msgs/typekit/Types.hpp>
#include <orocoros2_msgs/srv/player_service.hpp>
#include <orocoros2_msgs/msg/score.hpp>
#include <array> 

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
    void op_callback(orocoros2_msgs::srv::PlayerService::Request&, orocoros2_msgs::srv::PlayerService::Response&);


    // bool connect_to_topics_; //ROS Topic connection guard

    std::array<int, 2> current_score_;

    //Orocos ports
    RTT::InputPort<bool> player1_miss_port_;
    RTT::InputPort<bool> player2_miss_port_;
    RTT::OutputPort<bool> player1_start_port_;
    RTT::OutputPort<bool> player2_start_port_;
    //ROS ports
    RTT::InputPort<std_msgs::msg::Int32> player_watch_port_;
    RTT::OutputPort<orocoros2_msgs::msg::Score> score_port_;

};
}  // namespace orocoros2

#endif
