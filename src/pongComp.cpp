#include "orocoros2/pongComp.hpp"

namespace orocoros2
{
pongComp::pongComp(const std::string& name)
  : TaskContext(name)
  , connect_to_topics_(false)
  , hit_chance_(0.5)
  // , input_port_value_(0)
{

  this->addProperty("connect_to_topics", connect_to_topics_).doc("ROS Topic connection guard");
  this->addProperty("hit_chance", hit_chance_).doc("Hit Chance");
  // this->addProperty("input_port_value", input_port_value_).doc("Latest value processed on pongComp Input Port");

  //Orocos ports
  this->ports()->addPort("incoming_ball_port", incoming_ball_port_).doc("pongComp Input Port");
  this->ports()->addPort("return_ball_port", return_ball_port_).doc("pongComp Output Port");
  this->ports()->addPort("miss_ball_port", miss_ball_port_).doc("pongComp Output Port");
  //ROS ports
  this->ports()->addPort("player_action_port", player_action_port_).doc("pongComp Output Port with ROS");

  std::cout << "pongComp [" << getName() << "]: constructed" << std::endl;
}

pongComp::~pongComp()
{
  std::cout << "pongComp [" << getName() << "]: destruced" << std::endl;
}

bool pongComp::configureHook()
{
  bool success = true;

  //ROS Params
  boost::shared_ptr<rtt_ros2_params::RosParam> rosparam = this->getProvider<rtt_ros2_params::RosParam>("rosparam");
  if (rosparam and rosparam->ready())
  {
    if (not rosparam->loadProperty("connect_to_topics", "connect_to_topics"))
    {
        std::cout << "pongComp [" << getName() << "]: failed to update rosparam [connect_to_topics]" << std::endl;
    }
    if (not rosparam->storeProperty("hit_chance", "hit_chance"))  // set() here to initialize rosparam value
    {
        std::cout << "pongComp [" << getName() << "]: failed to update rosparam [hit_chance]" << std::endl;
    }
  }
  //ROS Topics
  if (connect_to_topics_)
  {
    if (not player_action_port_.createStream(rtt_ros2_topics::topic("/" + getName() + "/player_action_port")))
    {
        std::cout << "pongComp [" << getName() << "]: failed to connect to ROS topic [~/player_action_port]" << std::endl;
        success = false;
    }
  }
  std::cout << "pongComp [" << getName() << "]: configured" << std::endl;
  return success;
}

bool pongComp::startHook()
{
  bool success = true;

  std::cout << "pongComp [" << getName() << "]: started" << std::endl;
  return success;
}

void pongComp::updateHook()
{
  //Get latest value from rosparam
  // boost::shared_ptr<rtt_ros2_params::RosParam> rosparam = this->getProvider<rtt_ros2_params::RosParam>("rosparam");
  // if (rosparam and rosparam->ready())
  // {
  //   if (not rosparam->loadProperty("hit_chance", "hit_chance"))
  //   {
  //       std::cout << "pongComp [" << getName() << "]: failed to update rosparam [hit_chance]" << std::endl;
  //   }
  // }

  // Process Inputs
  //Orocos ports
  bool incoming_ball=false, new_data=false;
  if (incoming_ball_port_.read(incoming_ball) == RTT::FlowStatus::NewData)
    new_data = true;

  // Do Something
  std_msgs::msg::Int32 action;
  action.data = 0; //Wait
  bool miss=false;
  if(new_data && incoming_ball)
  {
    action.data = 1; //Hit

    miss = true;
    action.data = 2; //Miss
  }

  // Process Outputs
  //Orocos ports
  if(new_data)
  {
    if (incoming_ball && not return_ball_port_.write(!miss) == RTT::WriteStatus::WriteSuccess)
    {
        std::cerr << "pongComp [" << getName() << "]: return_ball_port failed to write" << std::endl;
    }
    if (not miss_ball_port_.write(miss) == RTT::WriteStatus::WriteSuccess)
    {
        std::cerr << "pongComp [" << getName() << "]: return_ball_port failed to write" << std::endl;
    }
  }
  //ROS ports
  if (not player_action_port_.write(action) == RTT::WriteStatus::WriteSuccess)
  {
      std::cerr << "pongComp [" << getName() << "]: player_action_port failed to write" << std::endl;
  }

  // std::cout << "pongComp [" << getName() << "]: updated" << std::endl;
}

void pongComp::stopHook()
{
  std::cout << "pongComp [" << getName() << "]: stopped" << std::endl;
}

void pongComp::cleanupHook()
{
  std::cout << "pongComp [" << getName() << "]: cleaned" << std::endl;
}


}  // namespace orocoros2

ORO_CREATE_COMPONENT(orocoros2::pongComp)
