#!/usr/bin/python

'''
Created on Apr 11, 2018

@author: flg-ma
@attention: beautiful outputs for travis CI
@contact: albus.marcel@gmail.com (Marcel Albus)
@version: 1.0.0


#############################################################################################

History:
- v1.0.0: first push
'''

import yaml
import pandas as pd
import numpy as np
import os
import sys


class CIOutput:
    def __init__(self):
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
        pass

    def color_string(self, string_text, color='normal'):
        '''
        function to print colorful text to terminal commands surrounded by lines of hashtags
        string_text:
        color:
        return: string
        '''
        color = color.lower()
        terminal_colors = {'normal': '',
                           'header': '\033[95m',
                           'blue': '\033[94m',
                           'green': '\033[92m',
                           'warning': '\033[93m',
                           'fail': '\033[91m',
                           'end': '\033[0m',
                           'bold': '\033[1m',
                           'underline': '\033[4m'}
        if color in terminal_colors.keys():
            return terminal_colors[color] + string_text + terminal_colors['end']
        else:
            return string_text

    def read_yaml(self):
        '''
        reads the output yaml-files of the ATF and saves them as a pandas dataframe
        return: pandas dataframe with bool values for successful tests
        '''
        data_dict = {}
        columns = ['testcase', 'bool']  # columns of the pandas dataframe...
        columns.extend(self.metrics)  # ... extended with the metrics
        df = pd.DataFrame([], columns=columns)  # setup dataframe with the columns above
        for items in self.testcases:
            data_dict[items] = {}  # create a dict inside a dict for all testcases
        counter = 0

        for case in self.testcases:  # save the metrics output for all testcases
            # metrics output directory includes test name
            filepath = '/tmp/atf_' + case + '/' + self.yaml_directory + '/' + self.yaml_name
            if os.path.exists(filepath):  # save the data from the 'yaml' if there is an output file
                stream = file(filepath, 'r')  # open filestream for yaml
                data_dict[case] = yaml.load(stream)  # save yaml in dict

                # create one bool with logic && from the 'groundtruth_results'
                testcase_bool = data_dict[case]['testblock_nav']['path_length'][0][
                                    'groundtruth_result'] and \
                                data_dict[case]['testblock_nav']['goal'][0][
                                    'groundtruth_result'] and \
                                data_dict[case]['testblock_nav']['jerk'][0][
                                    'groundtruth_result'] and \
                                data_dict[case]['testblock_nav']['time'][0][
                                    'groundtruth_result']
                # save the bool as float for the heatmap colour output
                testcase_bool = float(testcase_bool)
                # create a data dict to append at the dataframe
                data = {'testcase': case,
                        'bool': testcase_bool,
                        'path_length': data_dict[case]['testblock_nav']['path_length'][0][
                            'data'],
                        'path_length_err':
                            data_dict[case]['testblock_nav']['path_length'][0][
                                'groundtruth'] -
                            data_dict[case]['testblock_nav']['path_length'][0]['data'],
                        'path_length_gte':
                            data_dict[case]['testblock_nav']['path_length'][0][
                                'groundtruth_epsilon'],
                        'goal': data_dict[case]['testblock_nav']['goal'][0]['data'],
                        'goal_err':
                            data_dict[case]['testblock_nav']['goal'][0]['groundtruth'] -
                            data_dict[case]['testblock_nav']['goal'][0]['data'],
                        'goal_gte': data_dict[case]['testblock_nav']['goal'][0][
                            'groundtruth_epsilon'],
                        'jerk': data_dict[case]['testblock_nav']['jerk'][0]['data'],
                        'jerk_err':
                            data_dict[case]['testblock_nav']['jerk'][0]['groundtruth'] -
                            data_dict[case]['testblock_nav']['jerk'][0]['data'],
                        'jerk_gte': data_dict[case]['testblock_nav']['jerk'][0][
                            'groundtruth_epsilon'],
                        'time': data_dict[case]['testblock_nav']['time'][0]['data'],
                        'time_err':
                            data_dict[case]['testblock_nav']['time'][0]['groundtruth'] -
                            data_dict[case]['testblock_nav']['time'][0]['data'],
                        'time_gte': data_dict[case]['testblock_nav']['time'][0][
                            'groundtruth_epsilon']}
                df = df.append(data, ignore_index=True)  # append data to dataframe
                stream.close()  # close filestream
            else:  # if there is no generated output 'yaml'-file, save only the testcase name and number
                data = {'testcase': case,
                        'bool': 0.0,
                        'path_length': np.nan,  # set values to 'np.nan'
                        'path_length_err': np.nan,
                        'path_length_gte': np.nan,
                        'goal': np.nan,
                        'goal_err': np.nan,
                        'goal_gte': np.nan,
                        'jerk': np.nan,
                        'jerk_err': np.nan,
                        'jerk_gte': np.nan,
                        'time': np.nan,
                        'time_err': np.nan,
                        'time_gte': np.nan}
                df = df.append(data, ignore_index=True)  # append data to dataframe
                data_dict[case] = None
            # increase counter
            counter += 1

        self.dataframe = df.copy()  # save dataframe globally
        formatted_testcases = ['Line Passage',
                               'Line Passage Obstacle',
                               'Line Passage Person Moving',
                               'Line Passage Spawn Obstacle',
                               'Narrow Passage Two Cone',
                               'T Passage',
                               'T Passage Obstacle']

        i = 0
        for name in self.testcases:  # rename the row indices to nice, formatted names
            df = df.rename(index={name: formatted_testcases[i]})
            i += 1
        return df  # returns the dataframe

    def terminal_output(self):
        '''
        function for terminal output with color identification of bad/good values of ATF metrics
        :rtype: bool
        :return: error_flag
        '''
        error_flag = False

        # print introduction
        print self.color_string('#' * 118, 'bold')
        print self.color_string('#' * 46 + ' Travis CI Test using ATF ' + '#' * 46, 'bold')  # Travis CI Test using ATF
        print self.color_string('#' * 118, 'bold')

        df = self.read_yaml().rename(
            index={num: self.testcases[num] for num in
                   xrange(0, len(self.testcases))})  # rename index to testcase names

        # for every testcase name in index...
        for index in df.index.tolist():
            # ... and for every metric we tested...
            for metric in self.metrics:
                # ... format the string to a warning if the value is outside tolerated range
                if abs(df.loc[index, metric + '_err']) > df.loc[index, metric + '_gte']:
                    df.loc[index, metric] = self.color_string(
                        '{:3.4f} / {:3.4f}'.format(df.loc[index, metric],
                                                   # add the gte and err
                                                   df.loc[index, metric + '_gte'] + df.loc[index, metric + '_err']
                                                   # if gte is bigger than err
                                                   if df.loc[index, metric + '_gte'] > df.loc[index, metric + '_err']
                                                   # else subtract err from gte
                                                   else df.loc[index, metric + '_err'] - df.loc[
                                                       index, metric + '_gte']),
                        'warning')
                    error_flag = True

                # ... format the string to blue color if the value is inside tolerated range and OK
                else:
                    df.loc[index, metric] = self.color_string(
                        '{:3.4f} / {:3.4f}'.format(df.loc[index, metric],
                                                   # add the gte and err
                                                   df.loc[index, metric + '_gte'] + df.loc[index, metric + '_err']
                                                   # if gte is bigger than err
                                                   if df.loc[index, metric + '_gte'] > df.loc[index, metric + '_err']
                                                   # else subtract err from gte
                                                   else df.loc[index, metric + '_err'] - df.loc[
                                                       index, metric + '_gte']),
                        'blue')
        # formatted output of all metrics with fixed with of 30px, left aligned
        print '{:.<30} {:.<30} {:.<30} {:.<30} {:.<30}'.format('Metrics',
                                                               self.color_string('path_length', 'blue'),
                                                               self.color_string('goal', 'blue'),
                                                               self.color_string('time', 'blue'),
                                                               self.color_string('jerk', 'blue'))
        # formatted output for all values with fixed width of 30px, left aligned
        for index in df.index.tolist():
            print '{:.<30} {:.<30} {:.<30} {:.<30} {:.<30}'.format(index, df.loc[index]['path_length'],
                                                                   df.loc[index]['goal'],
                                                                   df.loc[index]['time'],
                                                                   df.loc[index]['jerk'])
        # print explanation
        print self.color_string('#' * 118, 'bold')
        print self.color_string('#' * 118, 'bold')
        end_string = '#' * 52 + ' Explanation ' \
                     + '#' * 53 + '\n' \
                     + 'first value: \t data value \n\n' \
                     + 'second value: \t difference to groundtruth \n' \
                     + 'plus if data is below groundtruth \n' \
                     + 'minus if data is above groundtruth \n\n' \
                     + 'color: \n' \
                     + 'blue if data is in range: groundtruth - groundtruth_epsilon <= data <= groundtruth + groundtruth_epsilon \n' \
                     + 'yellow if data is outside of defined range: groundtruth + groundtruth_epsilon < data || data < groundtruth - groundtruth_epsilon '
        print self.color_string(end_string, 'normal')

        return error_flag

    def main(self):
        '''
        main entrance point
        :rtype: bool
        :return: error_flag
        '''
        return self.terminal_output()


if __name__ == '__main__':
    cr = CIOutput()
    if not cr.main():
        sys.exit(0)
    else:
        sys.exit(1)
