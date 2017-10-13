#!/usr/bin/python
import unittest
import rospy
import rostest
import sys

from atf_core import ATF

from rviz_publisher import RvizPublisher

from simple_script_server import *

sss = simple_script_server()


class Application:
    def __init__(self):
        rp = RvizPublisher()
        filepath = '/home/flg-ma/git/catkin_ws/src/msh/msh_bringup/launch/t_passage.launch'
        # 'initialpose' publish
        rp.main(filepath, True, False)
        rospy.sleep(1)
        self.atf = ATF()

    def execute(self):
        self.atf.start('testblock_nav')

        # TODO send navigation goal and wait for goal reached

        # ipa-apartment:
        # sss.move("base", [3,0,0])
        # sss.move("base", [3,-3,-3.14159265358979/2])
        # sss.move("base", [0,-3,3.14159265358979])
        # sss.move("base", [0,0,0])


        # saturn-ingolstadt:
        # sss.move("base", [-13.577, 12.629, -1.544])
        # sss.move("base", [-0.1, 6.3, -0.0])

        # necessary to catch goal published on topic /move_base/goal
        rospy.sleep(3)
        sss.move("base", [5.0, -5.0, -3.14159265358979/2])
#        sss.move("base", [5.0, -3.0, 0.0])
        # rp.main(filepath, False, True, 2.0, 0.0, 0, 0, 0)

        # rospy.loginfo('\033[92m' + 'End' + '\033[0m')
        self.atf.stop('testblock_nav')
        self.atf.shutdown()


class Test(unittest.TestCase):
    def setUp(self):
        self.app = Application()

    def tearDown(self):
        pass

    def test_Recording(self):
        self.app.execute()


if __name__ == '__main__':
    rospy.init_node('test_name')
    if "standalone" in sys.argv:
        app = Application()
        app.execute()
    else:
        rostest.rosrun('application', 'recording', Test, sysargs=None)
