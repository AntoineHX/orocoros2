# OrocoRos2

## Table of Contents
- [Environment setup](#environment-setup)
   1. [Real-Time kernel setup](#real-time-kernel-setup)
   2. [ROS2-Foxy (Ubuntu Focal) setup](#ros-setup)
   3. [OROCOS 2.9 + ROS2 integration setup](#orocos-ros-setup)
- [OrocoRos2](#orocoros2-repo)
- [Other Orocos+ROS2 ressources](#other-ressources)

---
## Environment setup
### [Real-Time kernel setup](https://frankaemika.github.io/docs/installation_linux.html#setting-up-the-real-time-kernel)

This step is optional for this repository.

**WARNING**: You need to have at least 32GB free on your disk.

```
mkdir ~/rt_kernel
cd ~/rt_kernel
```

#### Pre-requirements
Dependencies :
```
apt-get install build-essential bc curl ca-certificates fakeroot gnupg2 libssl-dev lsb-release libelf-dev bison flex
```
Choose the closest kernel version from [https://www.kernel.org/pub/linux/kernel/projects/rt/](https://www.kernel.org/pub/linux/kernel/projects/rt/) by running : ``` uname -r ```. For example if your kernel version is *"5.8.0-48-generic"*, you should choose *"5.6.19-rt12"*.

```
curl -SLO https://www.kernel.org/pub/linux/kernel/v5.x/linux-5.6.19.tar.xz
curl -SLO https://www.kernel.org/pub/linux/kernel/v5.x/linux-5.6.19.tar.sign
curl -SLO https://www.kernel.org/pub/linux/kernel/projects/rt/5.6/older/patch-5.6.19-rt12.patch.xz
curl -SLO https://www.kernel.org/pub/linux/kernel/projects/rt/5.6/older/patch-5.6.19-rt12.patch.sign

xz -d linux-5.6.19.tar.xz
xz -d patch-5.6.19-rt12.patch.xz
```
#### (Optional) Check file integrity
Find RSA key IDs :
```
gpg2 --verify linux-5.6.19.tar.sign
gpg2 --verify patch-5.6.19-rt12.patch.sign
```
Download public keys (Replace <...> by the corresponding key IDs):
```
gpg2  --keyserver hkp://keys.gnupg.net --recv-keys <linux-5.6.19.tar.sign RSA key ID>
gpg2  --keyserver hkp://keys.gnupg.net --recv-keys <patch-5.6.19-rt12.patch.sign>
```
Verify files:
``` 
gpg2 --verify linux-5.6.19.tar.sign #Should display something like : Good signature from "Greg Kroah-Hartman <gregkh@linuxfoundation.org>"
gpg2 --verify patch-5.6.19-rt12.patch.sign #Should display something like : Good signature from "Sebastian Andrzej Siewior"
``` 
 

#### Kernel compilation
##### Apply patch 
```
tar xf linux-5.6.19.tar
cd linux-5.6.19
patch -p1 < ../patch-5.6.19-rt12.patch
```
##### Configure kernel
```
make oldconfig
```
When asked for the Preemption Model, choose the Fully Preemptible Kernel. You can pick the other options at their default values (Just press enter).

##### Compile kernel
```
fakeroot sudo make -j<parallel-workers> deb-pkg
```
###### (Optional) Compilation errors
If you get the following errors while running :
- With parallel-workers : ```dpkg-buildpackage: error: debian/rules build subprocess returned exit status 2```
- Without parallel-workers : ```make[4]: *** No rule to make target 'debian/canonical-certs.pem', needed by 'certs/x509_certificate_list'.  Stop.```

Edit the config file (```gedit ~/rt_kernel/linux-5.6.19/.config```) and comment (with ```#```) the following lines :
- CONFIG_MODULE_SIG_KEY
- CONFIG_SYSTEM_TRUSTED_KEYS

##### Install new kernel
```
sudo dpkg -i ../linux-headers-5.6.19-rt12_*.deb ../linux-image-5.6.19-rt12_*.deb
```

##### Verify new kernel
Restart your system and look for the newly installed kernel in the Grub boot menu. You might need to go into the *Ubuntu advanced boot option* to find the RT kernel (*"Ubuntu, with Linux 5.6.19-rt12"*).

After a successful boot, you can check that you're using the new RT kernel with : ```uname -a```. It should contain **"PREEMPT_RT"**.

###### (Optional) Booting errors
If you get the following error : ```error: boot/vmlinuz-5.6.19-rt12 has invalid signature```. 

You can either disable Secure Boot in your UEFI menu or, if you want to keep Secure Boot enabled, sign your kernel with this procedure : [https://askubuntu.com/a/1182830](https://askubuntu.com/a/1182830). If you're trying to sign the kernel, you might want to only use **2** integer for ```countryName = <YOURcountrycode>``` in Step *1* and replace ```[KERNEL-VERSION]-surface-linux-surface``` in the tutorial by ```5.6.19-rt12```.

##### Set RT permissions
```
sudo addgroup realtime
sudo usermod -a -G realtime $(whoami)
```
Add the following to *"/etc/security/limits.conf"* (```sudo gedit /etc/security/limits.conf```):
```
@realtime soft rtprio 99
@realtime soft priority 99
@realtime soft memlock 102400
@realtime hard rtprio 99
@realtime hard priority 99
@realtime hard memlock 102400
```
The limits will be applied after you log out and in again.

##### (CHECK IF OKAY) (Optional) Customize boot options
```
sudo apt install grub-customizer
grub-customizer
```
Create a new boot sequence, copy the boot sequence from the RT kernel and add the following to the "linux" line after "quiet splash" : ``` nosmt noefi intel_idle.max_cstate=1 pstate_driver=no_hwp clocksource=tsc tsc=reliable rcu_nocb_poll rcu_nocbs=3 nohz_full=3 nmi_watchdog=0 nosoftlockup nosmap audit=0 irqaffinity=0-2 isolcpus=5 ```
It'll disable hyperthreading and isolate a cpu (isolcpus=<cpuID>) for RT processes.
    
---
### [ROS2-Foxy (Ubuntu Focal) setup](https://docs.ros.org/en/foxy/Installation/Linux-Install-Debians.html#) <a name="ros-setup"></a>

#### Set locale
```
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```
#### Setup sources
```
sudo apt update && sudo apt install curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
```
#### Install ROS2
```
sudo apt update
sudo apt install ros-foxy-desktop
```
##### Complete colcon installation
```
sudo apt install python3-colcon-common-extensions
```
##### (Optional) Install argcomplete
```
sudo apt install -y python3-argcomplete
```
##### (Optional) Edit your shell startup script
Source automatically ROS2 : 
```
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
```
Source automatically Colcon_cd :
```
echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
echo "export _colcon_cd_root=~/ros2_install" >> ~/.bashrc
```
Use a ROS_DOMAIN_ID. Replace <your_domain_id> by an integer between 0-232.
```
echo "export ROS_DOMAIN_ID=<your_domain_id>" >> ~/.bashrc
```
---
### OROCOS 2.9 + ROS2 integration setup <a name="orocos-ros-setup"></a>
#### Pre-reuirements
```
sudo apt-get install liblua5.1-0-dev
sudo apt install -y git ros-foxy-ament-cmake-clang-format omniorb omniidl omniorb-idl omniorb-nameserver libomniorb4-dev

mkdir -p ~/orocos_ws/src  # any directory you prefer, but change below acoordingly
cd ~/orocos_ws/src

git clone https://github.com/orocos/rtt_ros2_integration.git
git clone https://github.com/orocos/rtt_ros2_common_interfaces.git
git clone --recurse-submodules --branch=ros2 https://github.com/orocos-toolchain/orocos_toolchain.git 
```
#### Install

Start building the orocos_toolchain package, you can ignore the "stderr output":
```
cd ~/orocos_ws

colcon build \
--packages-select orocos_toolchain \
--parallel-workers <parallel-workers> \
--install-base ~/orocos/${ROS_DISTRO} \
--merge-install \
--cmake-args \
    \ -DBUILD_TESTING=OFF \
    \ -DCMAKE_BUILD_TYPE=Release \
    \ -DENABLE_CORBA=ON \
    \ -DCORBA_IMPLEMENTATION=OMNIORB \
    \ -DOROCOS_INSTALL_INTO_PREFIX_ROOT=ON
```

Build the Orocos-toolchain with ROS2 integration, you might want to run the following twice. You should only get an "stderr output" for the orocos_toolchain package :
```
colcon build \
--parallel-workers <parallel-workers> \
--install-base ~/orocos/${ROS_DISTRO} \
--merge-install \
--cmake-args \
    \ -DBUILD_TESTING=OFF \
    \ -DCMAKE_BUILD_TYPE=Release \
    \ -DENABLE_CORBA=ON \
    \ -DCORBA_IMPLEMENTATION=OMNIORB \
    \ -DOROCOS_INSTALL_INTO_PREFIX_ROOT=ON 
   
```
##### (Optional) Install Orocos-toolchain with tests
If you also want to install the tests packages :
```
sudo apt install ros-foxy-test-msgs
```
And run the previous *colcon build* command with ```-DBUILD_TESTING=ON``` option.
#### Verfify installation
```
source ~/orocos/foxy/local_setup.bash
deployer
import("rtt_ros2")
```
The import command should return ```true```.

#### (Optional) Edit your shell startup script
Source automatically Orocos :
```
echo "source ~/orocos/foxy/local_setup.bash" >> ~/.bashrc
```
---
---
## OrocoRos2 <a name="orocoros2-repo"></a>
### Install

```
mkdir -p ~/dev_ws/src
cd ~/dev_ws/src
git clone https://github.com/AntoineHX/orocoros2_msgs.git
git clone https://github.com/AntoineHX/rtt_ros2_orocoros2_msgs
git clone https://github.com/AntoineHX/OrocoRos2.git

cd ~/dev_ws
colcon build
```
### Usage

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
## Other Orocos+ROS2 ressources <a name="other-ressources"></a>
- Antoher simple example : https://gitlab.com/AntoineHX/orocos_examples
- OROCOS+ROS courses : https://atlas-itn.eu/training/network-training-activities/nta-3-best-integration-practices-and-robotic-middleware/

---
---
## Contributions
Contributions from Antoine Harlé (<harle@isir.upmc.fr>), Jimmy Da Silva (<jimmy.dasilva@isir.upmc.fr>), Kenan Niu (<kenan.niu@kuleuven.be>), Sergio Portoles Diez (<sergio.portoles.diez@intermodalics.eu>).
