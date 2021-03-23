# OrocoRos2

## Install

Orocos + ROS2 install :

```
sudo apt-get install liblua5.1-0-dev
sudo apt install -y build-essential cmake libboost-all-dev libxml-xpath-perl libboost-all-dev pkg-config libxml2-dev ruby-dev
sudo apt install omniorb omniidl omniorb-idl omniorb-nameserver libomniorb4-dev
(sudo apt-get install doxygen)

mkdir -p ~/orocos_ws/src
cd ~/orocos_ws/src

git clone https://github.com/orocos/rtt_ros2_integration.git
git clone https://github.com/orocos/rtt_ros2_common_interfaces.git
git clone https://github.com/orocos-toolchain/orocos_toolchain.git
cd orocos_toolchain/
git checkout ros2

cd ~/orocos_ws/
colcon build \
--packages-select orocos_toolchain \
--parallel-workers 6 \
--event-handlers console_direct+ \
--install-base ~/orocos/${ROS_DISTRO} \
--merge-install \
--cmake-args \
    \ -DBUILD_TESTING=OFF \
    \ -DCMAKE_BUILD_TYPE=Release \
    \ -DENABLE_CORBA=ON \
    \ -DCORBA_IMPLEMENTATION=OMNIORB \
    \ -DOROCOS_INSTALL_INTO_PREFIX_ROOT=ON

colcon build \
--event-handlers console_direct+ \
--install-base ~/orocos/${ROS_DISTRO} \
--merge-install \
--cmake-args \
    \ -DBUILD_TESTING=OFF \
    \ -DCMAKE_BUILD_TYPE=Release \
    \ -DENABLE_CORBA=ON \
    \ -DCORBA_IMPLEMENTATION=OMNIORB \
    \ -DOROCOS_INSTALL_INTO_PREFIX_ROOT=ON


source ~/orocos/foxy/local_setup.bash
```

OrocoRos2 install :

```
mkdir -p ~/dev_ws/src
cd ~/dev_ws/src
git clone https://github.com/AntoineHX/orocoros2_msgs.git
git clone https://github.com/AntoineHX/rtt_ros2_orocoros2_msgs
git clone https://github.com/AntoineHX/OrocoRos2.git

cd ~/dev_ws
colcon build
```
## Usage

- Start pong components :
```
source ~/dev_ws/install/local_setup.bash
cd ~/dev_ws/src/orocoros2/launch
deployer PingPong.ops
```
- Start match (Replace <Player> by 1 or 2, to choose the first player to hit the ball) :

```
ros2 service call /PingPong/start_match orocoros2_msgs/srv/PlayerService "{player: <Player>}"
```
