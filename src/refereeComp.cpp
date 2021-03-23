#include "orocoros2/refereeComp.hpp"

namespace orocoros2
{
refereeComp::refereeComp(const std::string& name)
  : TaskContext(name)
  // , connect_to_topics_(false)
  , current_score_ {0,0}
{

  // this->addProperty("connect_to_topics", connect_to_topics_).doc("ROS Topic connection guard");
  this->addProperty("current_score", current_score_).doc("Current score");

  //Orocos ports
  this->ports()->addPort("player1_miss_port", player1_miss_port_).doc("refereeComp Input Port");
  this->ports()->addPort("player2_miss_port", player2_miss_port_).doc("refereeComp Input Port");
  this->ports()->addPort("player1_start_port", player1_start_port_).doc("refereeComp Output Port");
  this->ports()->addPort("player2_start_port", player2_start_port_).doc("refereeComp Output Port");

  //ROS ports
  this->ports()->addPort("score_port", score_port_).doc("refereeComp ROS Output Port");

  //ROS Service
  this->provides("referee_service")
      ->addOperation("start_match", &refereeComp::op_callback, this, RTT::ClientThread)
      .arg("request", "An integer input.")
      .arg("response", "An bool output.")
      .doc("Launch a new ball to a player with ROS Messages");

  std::cout << "refereeComp [" << getName() << "]: constructed" << std::endl;
}

refereeComp::~refereeComp()
{
  std::cout << "refereeComp [" << getName() << "]: destruced" << std::endl;
}

bool refereeComp::configureHook()
{
  bool success = true;

  //ROS Params
  boost::shared_ptr<rtt_ros2_params::RosParam> rosparam = this->getProvider<rtt_ros2_params::RosParam>("rosparam");
  if (rosparam and rosparam->ready())
  {
      // if (not rosparam->loadProperty("connect_to_topics", "connect_to_topics"))
      // {
      //     std::cout << "refereeComp [" << getName() << "]: failed to update rosparam [connect_to_topics]" << std::endl;
      // }
      if (not rosparam->storeProperty("current_score", "current_score"))  // set() here to initialize rosparam value
      {
          std::cout << "refereeComp [" << getName() << "]: failed to update rosparam [current_score]" << std::endl;
      }
  }   
  //ROS Topics
  // if (connect_to_topics_)
  // {
  //     if (not score_port_.createStream(rtt_ros2_topics::topic("/" + getName() + "/score_port")))
  //     {
  //         std::cout << "refereeComp [" << getName() << "]: failed to connect to ROS topic [~/score_port]" << std::endl;
  //         success = false;
  //     }
  // }
  //ROS Services
  // boost::shared_ptr<rtt_ros2_services::RosService> rosservice = this->getProvider<rtt_ros2_services::RosService>("rosservice");
  // if (rosservice and rosservice->ready())
  // {
  //     if (not rosservice->connect("referee_service.start_match", "/" + getName() + "/start_match", "oe_msgs/srv/Operation"))
  //     {
  //         std::cout << "refereeComp [" << getName() << "]: failed to connect to ROS service [~/start_match]" << std::endl;
  //         success = false;
  //     }
  // }

  std::cout << "refereeComp [" << getName() << "]: configured" << std::endl;
  return success;
}

bool refereeComp::startHook()
{
  bool success = true;

  std::cout << "refereeComp [" << getName() << "]: started" << std::endl;
  return success;
}

void refereeComp::updateHook()
{
  // Process Inputs
  //Orocos ports
  bool input_player1, input_player2, new_data =false;
  if (player1_miss_port_.read(input_player1) == RTT::FlowStatus::NewData)
    new_data = true;
  if (player2_miss_port_.read(input_player2) == RTT::FlowStatus::NewData)
    new_data = true;

  // Do Something
  if(new_data)
  {
    if(input_player1)
      current_score_[0]+=1;
    if(input_player2)
      current_score_[1]+=1;
  }

  // Process Outputs
  //ROS ports
  orocoros2_msgs::msg::Score score_msg;
  score_msg.score = current_score_;
  if (not score_port_.write(score_msg) == RTT::WriteStatus::WriteSuccess)
  {
      std::cerr << "refereeComp [" << getName() << "]: score_port failed to write" << std::endl;
  }

  // std::cout << "refereeComp [" << getName() << "]: updated" << std::endl;
}

void refereeComp::stopHook()
{
  std::cout << "refereeComp [" << getName() << "]: stopped" << std::endl;
}

void refereeComp::cleanupHook()
{
  std::cout << "refereeComp [" << getName() << "]: cleaned" << std::endl;
}

//Better to write on port in Update loop ?
void refereeComp::op_callback(orocoros2_msgs::srv::PlayerService::Request& request, orocoros2_msgs::srv::PlayerService::Response& response)
{
  bool success=true;
  if (request.player == request.PLAYER1 && not player1_start_port_.write(true) == RTT::WriteStatus::WriteSuccess)
  {
      std::cerr << "refereeComp [" << getName() << "]: player1_start_port failed to write" << std::endl;
      success=false;
  } 
  if (request.player== request.PLAYER2 && not player2_start_port_.write(true) == RTT::WriteStatus::WriteSuccess)
  {
      std::cerr << "refereeComp [" << getName() << "]: player2_start_port failed to write" << std::endl;
      success=false;
  } 
  
  response.success = success;
  std::cout << "refereeComp [" << getName() << "]: start_match operation executed (success:"<< success <<") with argument = " << request.player << std::endl;
}

}  // namespace orocoros2

ORO_CREATE_COMPONENT(orocoros2::refereeComp)
