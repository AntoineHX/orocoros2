require "rttlib"
-- require "rttros"
require "os"

-- ------------------------------------------ --
-- Get the deployer and the necessary imports --
-- ------------------------------------------ --

-- get the deployer
tc=rtt.getTC()
if tc:getName() == "lua" then
  dep=tc:getPeer("Deployer")     
elseif tc:getName() == "Deployer" then
  dep=tc
end

-- do the necessary imports
dep:import("ocl")
dep:import("rtt_ros2_std_msgs")
dep:import("rtt_ros2")
dep:import("rtt_ros2_node")
rtt.provides("ros"):import("orocoros2")

-- ------------------------- --
-- Important global settings --
-- ------------------------- --
ros_application_name = "PingPong"
app_period = 1.0
connect_to_topics = true
rtt.provides("ros"):create_named_node_with_namespace("Main_node",ros_application_name)
-- connect_to_topics = rtt.Variable("rclcpp.ParameterValue",true)
-- print(connect_to_topics)
-- rtt.provides("ros"):provides("rosparam"):setParameter("connect_to_topics", connect_to_topics)

-- ----------------- --
-- Create components --
-- ----------------- --
-- Referee
comp_name="Referee"
dep:loadComponent(comp_name,"orocoros2::refereeComp")
pong_referee = dep:getPeer(comp_name)
pong_referee:addPeer(dep)
dep:setActivity(comp_name,app_period,1,1)
pong_referee:loadService("rosparam")
pong_referee:loadService("rosservice")

-- ctt_prop=pong_referee:getProperty("connect_to_topics")
-- ctt_prop:set(connect_to_topics)

-- Player1
comp_name="Player1"
dep:loadComponent(comp_name,"orocoros2::pongComp")
pong_player1 = dep:getPeer(comp_name)
pong_player1:addPeer(dep)
dep:setActivity(comp_name,app_period,1,1)
pong_player1:loadService("rosparam")

-- ctt_prop=pong_player1:getProperty("connect_to_topics")
-- ctt_prop:set(connect_to_topics)

-- Player2
comp_name="Player2"
dep:loadComponent(comp_name,"orocoros2::pongComp")
pong_player2 = dep:getPeer(comp_name)
pong_player2:addPeer(dep)
dep:setActivity(comp_name,app_period,1,1)
pong_player2:loadService("rosparam")

-- ctt_prop=pong_player2:getProperty("connect_to_topics")
-- ctt_prop:set(connect_to_topics)

-- -------------------- --
-- Configure components --
-- -------------------- --
pong_referee:configure()
pong_player1:configure()
pong_player2:configure()

-- -- ------------------ --
-- -- Create connections --
-- -- ------------------ --

cp = rtt.Variable('ConnPolicy')
cp.type = 0 -- type DATA
-- cp_ros = rtt.Variable('ConnPolicy')
-- cp_ros.transport = 3   -- transport layer: ROS

-- set the OROCOS port connections ################ replace by the name
dep:connect( pong_referee:getName()..".player1_start_port", pong_player1:getName()..".incoming_ball_port", cp )
dep:connect( pong_referee:getName()..".player2_start_port", pong_player2:getName()..".incoming_ball_port", cp )
dep:connect( pong_player1:getName()..".miss_ball_port", pong_referee:getName()..".player1_miss_port", cp )
dep:connect( pong_player2:getName()..".miss_ball_port", pong_referee:getName()..".player2_miss_port", cp )
dep:connect( pong_player1:getName()..".return_ball_port", pong_player2:getName()..".incoming_ball_port", cp )
dep:connect( pong_player2:getName()..".return_ball_port", pong_player1:getName()..".incoming_ball_port", cp )
-- set the ROS port connections --
ros_namespace ="/"..ros_application_name --Concatenate strings
dep:stream( pong_referee:getName()..".score_port", rtt.provides("ros"):topic(ros_namespace.."/score_port", false))
dep:stream( pong_player1:getName()..".player_action_port", rtt.provides("ros"):topic(ros_namespace.."/"..pong_player1:getName().."_action", false))
dep:stream( pong_player2:getName()..".player_action_port", rtt.provides("ros"):topic(ros_namespace.."/"..pong_player2:getName().."_action", false))
-- Connect ROS Services --
pong_referee:provides("rosservice"):connect("referee_service.start_match", ros_namespace.."/start_match", "orocoros2_msgs/srv/PlayerService")
-- ---------------- --
-- Start components --
-- ---------------- --
pong_referee:start()
pong_player1:start()
pong_player2:start()

print("-----------------------------------------------------------------")