set -e
set -v

while true; do echo "INSTALL IS RUNNING" && sleep 60; done&

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update -qq > /dev/null 2>&1
sudo apt-get install -qq -y python-rosdep python-wstool > /dev/null 2>&1
sudo apt-get install -y ros-$CI_ROS_DISTRO-ros-base # > /dev/null 2>&1
sudo rosdep init
rosdep update

source /opt/ros/$CI_ROS_DISTRO/setup.bash # > /dev/null 2>&1 # source release
# create empty ATF workspace
mkdir -p $ATF_WS_SRC
catkin_init_workspace $ATF_WS_SRC
cd $ATF_WS
catkin_make -DCMAKE_BUILD_TYPE=Release # build empty ATF devel space
catkin_make -DCMAKE_BUILD_TYPE=Release install # build empty ATF install space
cd $ATF_WS_SRC
# clone ATF to src/
git clone git@github.com:ipa-flg-ma/atf.git
cd $ATF_WS
# populate ATF
if [ -f src/.travis.rosinstall ]; then wstool init -j10 $ATF_WS_SRC/.travis.rosinstall; fi
# install dependencies from ATF
rosdep install -q --from-paths $ATF_WS_SRC -i -y --rosdistro $CI_ROS_DISTRO > /dev/null #2>&1
# build devel space of underlay
source $ATF_WS/devel/setup.bash # > /dev/null 2>&1 # source devel space of underlay
catkin_make -DCMAKE_BUILD_TYPE=Release
# build install space of underlay
catkin_make -DCMAKE_BUILD_TYPE=Release install # > /dev/null #2>&1


# create empty underlay workspace
mkdir -p $CATKIN_WS_UNDERLAY_SRC
catkin_init_workspace $CATKIN_WS_UNDERLAY_SRC
cd $CATKIN_WS_UNDERLAY
catkin_make -DCMAKE_BUILD_TYPE=Release # build empty underlay devel space
catkin_make -DCMAKE_BUILD_TYPE=Release install # build empty underlay install space
# populate underlay
if [ -f $TRAVIS_BUILD_DIR/.travis.rosinstall ]; then wstool init -j10 src $TRAVIS_BUILD_DIR/.travis.rosinstall; fi
if [ ! -f $TRAVIS_BUILD_DIR/.travis.rosinstall ]; then wstool init -j10 src $DEFAULT_ROSINSTALL; fi
# install dependencies from underlay
rosdep install -q --from-paths $CATKIN_WS_UNDERLAY_SRC -i -y --rosdistro $CI_ROS_DISTRO # > /dev/null #2>&1
# build devel space of underlay
source $CATKIN_WS_UNDERLAY/devel/setup.bash # > /dev/null 2>&1 # source devel space of underlay
catkin_make -DCMAKE_BUILD_TYPE=Release
# build install space of underlay
catkin_make -DCMAKE_BUILD_TYPE=Release install # > /dev/null #2>&1
ret=$?
kill %%
exit $ret