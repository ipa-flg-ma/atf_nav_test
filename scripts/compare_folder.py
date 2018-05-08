#!/usr/bin/python

"""
Created on April 3, 2018

@author: flg-ma
@attention: compare the output results of a complete ATF test folder
@contact: albus.marcel@gmail.com (Marcel Albus)
@version: 1.1.1


#############################################################################################

History:
- v1.1.1: rename from 'compare' to 'compare_folder'
- v1.1.0: write "average values" and "standard deviation" to "average.txt" file
- v1.0.0: first push
"""

import yaml
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os
from bcolors import TerminalColors as tc
import getpass


class CompareResults:
    def __init__(self, filepath=None):
        print tc.OKBLUE + '=' * 100 + tc.ENDC
        print tc.OKBLUE + '=' * 42 + ' Compare Average ' + '=' * 41 + tc.ENDC
        print tc.OKBLUE + '=' * 100 + tc.ENDC
        try:
            if filepath is None:
                self.pth = raw_input(
                    'Please enter Path to generated testcase output (e.g: \'/home/' + getpass.getuser() + '/Test/\'): ')
            else:
                self.pth = filepath
            if os.path.exists(self.pth + 'Dataframe.csv'):
                pass
            else:
                print 'Collecting directories in given path...'
                self.directories = os.walk(self.pth).next()[1]
        except StopIteration:  # catch error when there is no valid directory given
            exit('The directory path does not exist')

        print '=' * 100

        self.yaml_directory = 'results_yaml'  # output 'yaml'-directory
        self.yaml_name = 'ts0_c0_r0_e0_0.yaml'  # output 'yaml'-name
        # testcases for ATF
        self.testcases = ['line_passage',  # 0
                          'line_passage_obstacle',  # 1
                          'line_passage_person_moving',  # 2
                          'line_passage_spawn_obstacle',  # 3
                          'narrow_passage_2_cone',  # 4
                          't_passage',  # 5
                          't_passage_obstacle']  # 6
        # metrics for ATF
        self.metrics = ['path_length',
                        'goal',
                        'jerk',
                        'time']

    def read_yaml(self):
        '''
        reads the output yaml-files of the ATF and saves them as a pandas dataframe
        :return: pandas dataframe with all metrics values
        '''
        if os.path.exists(self.pth + 'Dataframe.csv'):
            df = pd.read_csv(self.pth + 'Dataframe.csv')
        else:
            data_dict = {}
            columns = ['testcase', 'test_number']  # columns of the pandas dataframe...
            columns.extend(self.metrics)  # ... extended with the metrics
            df = pd.DataFrame([], columns=columns)  # setup dataframe with the testcases as columns
            for items in self.testcases:
                data_dict[items] = {}  # create a dict inside a dict for all testcases
            counter = 0
            for folder in self.directories:
                try:  # if the dataname includes no number...
                    except_flag = False
                    testcase_number = int(filter(str.isdigit, folder))  # number of testcase is saved
                except ValueError as e:
                    except_flag = True
                    print 'No number in testcase found, assuming only one test was made...'
                if ('narrow' in folder) and (not except_flag) and (testcase_number != 2):
                    # save testcase name from folder name without number
                    testcase_name = folder[: -(len(str(testcase_number)))]
                    # narrow_passage_2_cone would save the '2' from the name as number --> not needed
                    testcase_number = int(str(testcase_number)[1:])
                elif not except_flag and not ('narrow' in folder):
                    # save testcase name from folder name without number when it's not 'narrow_passage_2_cone'
                    testcase_name = folder[: -(len(str(testcase_number)) + 1)]
                else:
                    testcase_number = 0  # ... zero is saved as number
                    testcase_name = folder

                filepath = self.pth + folder + '/' + self.yaml_directory + '/' + self.yaml_name
                if os.path.exists(filepath):  # save the data from the 'yaml' if there is an output file
                    stream = file(filepath, 'r')  # open filestream for yaml
                    data_dict[testcase_name][testcase_number] = yaml.load(stream)  # save yaml in dict

                    # create a data dict to append at the dataframe
                    data = {'testcase': testcase_name,
                            'test_number': testcase_number,
                            'path_length': data_dict[testcase_name][testcase_number]['testblock_nav']['path_length'][0][
                                'data'],
                            'goal': data_dict[testcase_name][testcase_number]['testblock_nav']['goal'][0]['data'],
                            'jerk': data_dict[testcase_name][testcase_number]['testblock_nav']['jerk'][0]['data'],
                            'time': data_dict[testcase_name][testcase_number]['testblock_nav']['time'][0]['data']}
                    df = df.append(data, ignore_index=True)  # append data to dataframe
                    stream.close()  # close filestream
                else:  # if there is no generated output 'yaml'-file, save only the testcase name and number
                    data = {'testcase': testcase_name,
                            'test_number': testcase_number,
                            'path_length': np.nan,  # set values to 'np.nan'
                            'goal': np.nan,
                            'jerk': np.nan,
                            'time': np.nan}
                    df = df.append(data, ignore_index=True)  # append data to dataframe
                    data_dict[testcase_name][testcase_number] = None
                # increase counter
                counter += 1
                #
                if counter % 20 == 0:
                    print '\r' + 'Directories saved: ' + str(counter) + ' / ' + str(self.directories.__len__()),

        self.dataframe = df.copy()  # save dataframe globally
        self.dataframe.to_csv(self.pth + 'Dataframe.csv', index=False)
        print 'save \'dataframe\' as \'csv\''
        print '=' * 100

        formatted_testcases = ['Line Passage',
                               'Line Passage Obstacle',
                               'Line Passage Person Moving',
                               'Line Passage Spawn Obstacle',
                               'Narrow Passage Two Cone',
                               'T Passage',
                               'T Passage Obstacle']
        print df.head(10)  # print the first 'n' numbers of the table
        print '=' * 100
        return df  # returns the dataframe

    def plot_error_bar(self):

        df = self.dataframe.copy()
        df = df.drop(['test_number'], axis=1)
        print df.head(5)  # print first 'n' rows
        print '=' * 100
        gp = df.groupby(['testcase'])

        rename_dict = {'line_passage': 'Line Passage',
                       'line_passage_obstacle': 'Line Passage\nObstacle',
                       'line_passage_person_moving': 'Line Passage\nPerson Moving',
                       'line_passage_spawn_obstacle': 'Line Passage\nSpawn Obstacle',
                       'narrow_passage_2_cone': 'Narrow Passage\nTwo Cone',
                       't_passage': 'T Passage',
                       't_passage_obstacle': 'T Passage\nObstacle'}
        means = gp.mean().rename(index=rename_dict)  # first mean, then rename, otherwise no errorbars are shown
        error = gp.std().rename(index=rename_dict)  # rename standard deviation
        print tc.OKBLUE + 'average values' + tc.ENDC
        print means
        print '=' * 100
        print tc.OKBLUE + 'standard deviation' + tc.ENDC
        print error

        # write average and standard deviation to file
        with open(self.pth + 'Average.txt', 'w') as file:
            file.write('average values\n')
            file.write(means.__str__() + '\n')
            file.write('=' * 100 + '\n')  # add lines for readability
            file.write('=' * 100 + '\n')  # add lines for readability
            file.write('standard deviation\n')
            file.write(error.__str__() + '\n')
            file.close()

        scale_factor = 1.2
        fig = plt.figure(222, figsize=(13.8 * scale_factor, 12.6 * 2.2))
        ax1 = fig.add_subplot(411)
        ax = means.plot.bar(yerr=error, ax=ax1, error_kw={'elinewidth': 2})
        plt.xticks(rotation=90, fontsize=20)
        ax.xaxis.label.set_size(20)
        plt.legend(loc=2, fontsize=12)
        plt.grid(True)
        plt.savefig(self.pth + 'Errorbar.pdf', bbox_inches='tight')
        fig.clf()
        # plt.show()

    def main(self):
        self.read_yaml()
        self.plot_error_bar()
        print tc.OKGREEN + '=' * 41 + ' Generated Errorbarplot ' + '=' * 41 + tc.ENDC


if __name__ == '__main__':
    cr = CompareResults()
    cr.main()
