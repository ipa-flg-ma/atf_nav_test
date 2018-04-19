set -e
set -v

while true; do echo "SCRIPT IS RUNNING" && sleep 60; done&

# create empty overlay workspace
mkdir -p $CATKIN_WS_SRC
# mkdir -p $CATKIN_WS_UNDERLAY_SRC
# catkin_init_workspace $CATKIN_WS_SRC
cd $CATKIN_WS
catkin_make -DCMAKE_BUILD_TYPE=Release # build empty overlay
catkin_make -DCMAKE_BUILD_TYPE=Release install # build empty underlay install space
# populate overlay
ln -s $TRAVIS_BUILD_DIR $CATKIN_WS_SRC
# install dependencies from overlay
rosdep install -q --from-paths $CATKIN_WS_SRC -i -y --rosdistro $CI_ROS_DISTRO
# build overlay
source $CATKIN_WS/devel/setup.bash # source devel space of overlay
catkin_make -DCMAKE_BUILD_TYPE=Release
catkin_make -DCMAKE_BUILD_TYPE=Release install # build empty underlay install space
# if [ "$CATKIN_ENABLE_TESTING" == "OFF" ]; then
#   echo "Testing disabled"
# else
#   mkdir -p $CATKIN_WS/build/test_results # create test_results directory to prevent concurrent tests to fail create it
#   catkin_make run_tests $CATKIN_TEST_ARGUMENTS # test overlay
# fi
# catkin_test_results --verbose
ret=$?
kill %%
exit $ret