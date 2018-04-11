#!/usr/bin/python

"""
Created on April 3, 2018

@author: flg-ma
@attention: start auto testcases using ATF
@contact: albus.marcel@gmail.com (Marcel Albus)
@version: 1.0.0

#############################################################################################

History:
- v1.0.0: first init
"""

import os
import shutil
import rospkg
import argparse
from distutils import dir_util
import time
import glob
import getpass
from bcolors import TerminalColors as tc


class StartTestcases:
    def __init__(self):
        self.testcases = ['line_passage',  # 0
                          'line_passage_obstacle',  # 1
                          'line_passage_person_moving',  # 2
                          'line_passage_spawn_obstacle',  # 3
                          'narrow_passage_2_cone',  # 4
                          't_passage',  # 5
                          't_passage_obstacle']  # 6
        self.rospack = rospkg.RosPack()  # get path for ROS package
        # get username go include in path
        self.atf_yaml_generated_pth = '/home/' + getpass.getuser() + '/git/atf_nav_test_config/scripts/yaml_files'
        self.args = self.build_parser().parse_args()
        self.timeformat = "%Y_%m_%d"
        self.move_base_eband_param_path = self.rospack.get_path(
            'ipa_navigation_config') + '/robots/raw3-3/nav/move_base_eband_params.yaml'

    def build_parser(self):
        parser = argparse.ArgumentParser(description='Start testcases using ATF')
        parser.add_argument('-c', '--count', help='How many repetitions should be made for the test cases?', type=int,
                            default=1)
        return parser

    def save_files(self, filepath, testcase):
        '''
        save the output files of the metrics and the parameter files in the desired filepath
        :filepath: path where the generated '.yaml' files should be saved
        :return: --
        '''
        metric_yaml_output_directory = '/tmp/atf_nav_test/'
        # copy the generated output yaml files from the metrics in the data folder
        dir_util.copy_tree(src=metric_yaml_output_directory, dst=filepath)
        # copy the 'move_base_eband_params.yaml' file in the data folder
        shutil.copy2(src=self.move_base_eband_param_path, dst=filepath)
        print '=' * 80
        print tc.OKGREEN + 'Copying output \'yaml\'-files of ' + testcase + tc.ENDC
        print '=' * 80

    def cpy_manual_yaml_to_conf_foldr(self, yaml_name):
        # copy the 'yaml' files from the atf directory into the config folder for ATF
        shutil.copy2(src=yaml_name, dst=self.move_base_eband_param_path)

    def main(self):
        '''
        main function of the program
        :return: --
        '''

        print '=' * 80
        print '=' * 80
        print tc.OKGREEN + 'Automated Test Simulation'
        print 'including the following testcases: '
        for case in self.testcases:
            print '- ', case
        print tc.ENDC
        print '=' * 80
        print '=' * 80

        starttime = time.time()

        # start ATF repetition cycle
        for repetition in xrange(1, self.args.count + 1):
            for case in self.testcases:
                atf_pkg_path = self.rospack.get_path(case)
                # path where the data is saved
                data_save_pth = 'Data/' + time.strftime(self.timeformat) + '/' + case + '_' + str(repetition)
                print '=' * 80
                print tc.OKGREEN + 'The specified testcase is: ' + case + tc.ENDC
                print '=' * 80

                os.chdir(atf_pkg_path + '/../../../')
                # console command to start ATF
                startstring = 'catkin build -v ' + case + ' --catkin-make-args run_tests'
                # start ATF
                os.system(startstring)
                # kill all gazebo tasks
                os.system('kill $(ps aux | grep \'[g]azebo\' | awk \'{print $2}\')')
                # save the generated 'yaml' files from the metrics and the 'move_base_eband_params.yaml' file
                print '=' * 80
                print tc.OKGREEN + 'Save the generated \'yaml\' files at: ' + data_save_pth + tc.ENDC
                self.save_files(filepath=data_save_pth, testcase=case)
                print '=' * 80

                # purge 'log' files for 'catkin build ...'
                os.chdir('/home/' + getpass.getuser() + '/.ros')
                os.system('rosclean purge -y')
                print tc.WARNING + '=' * 80 + tc.ENDC
                print '=' * 80

        endtime = time.time()
        timestring = tc.WARNING + 'Time needed: {:10.3} [min]' + tc.ENDC
        print timestring.format((endtime - starttime) / 60)
        print '=' * 80


if __name__ == '__main__':
    st = StartTestcases()
    st.main()
pass
