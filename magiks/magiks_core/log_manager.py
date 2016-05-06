# HEADER

## @file        	log_manager.py
#  @brief           This module provides classes containing data structure for IK test log data
#  @author      	Nima Ramezani Taghiabadi
#
#               	PhD Researcher 
#               	Faculty of Engineering and Information Technology 
#               	University of Technology Sydney (UTS) 
#               	Broadway, Ultimo, NSW 2007, Australia 
#               	Phone No. :   04 5027 4611 
#               	Email(1)  : nima.ramezani@gmail.com 
#               	Email(2)  : Nima.RamezaniTaghiabadi@uts.edu.au 
#  @version     	3.0
#
#  Last Revision:  	03 January 2015

# BODY
import math, pickle

from math_tools.algebra import vectors_and_matrices as vecmat
from math_tools.statistics import statistics

# \cond
key_dic = {
    # Forward:
    # For Test_Log_Data_Single_Run():
    
    'TPN'   : 'Target Pose Number',
    'S'     : 'Success',
    'CIR'   : 'Configuration in Range',
    'NSPT'  : 'Number of Starting Point Trials',
    'NI'    : 'Number of Iterations',
    'NS'    : 'Number of Success', 
    'RT'    : 'Running Time (ms)',
    'TRT'   : 'Total Running Time (ms)',
    'ART'   : 'Average Running Time (ms)', 
    'TNI'   : 'Total Number of Iterations', 
    'ANI'   : 'Average Number of Iterations',
    'PS'    : 'Percentage of Success',  # percentage of success until now (This field exists in Test_Log_Data_Statistics() and contains per.Suc. for all runs
    'IC'    : 'Initial Configuration',
    'FC'    : 'Final Configuration',
    'IP'    : 'Initial Pose',
    'FP'    : 'Final Pose',

    # For Test_Log_Data_Statistics():
    'PSIR'  : 'Percentage of Success in Range',  # percentage of successful runs in range 
    'NRun'  : 'Number of Runs',
    'NSRun' : 'Number of Successful Runs',
    'NIR'   : 'Number of Runs in Range',
    'NSIR'  : 'Number of Successful Runs in Range',
    'NI-tot': 'Number of Iterations-Total',
    'NI-max': 'Number of Iterations-Maximum',
    'NI-min': 'Number of Iterations-Minimum',
    'NI-avr': 'Number of Iterations-Average',
    'NI-med': 'Number of Iterations-Median',
    'NI-sdv': 'Number of Iterations-Standard Deviation',
    'NI-mse': 'Number of Iterations-Mean Standard Error',
    'NT-max': 'Number of Trials-Maximum',
    'NT-min': 'Number of Trials-Minimum',
    'NT-avr': 'Number of Trials-Average',
    'RT-tot': 'Running Time-Total (ms)',
    'RT-max': 'Running Time-Maximum (ms)',
    'RT-min': 'Running Time-Minimum (ms)',
    'RT-avr': 'Running Time-Average (ms)',
    'RT-med': 'Running Time-Median (ms)',
    'RT-sdv': 'Running Time-Standard Deviation (ms)', 
    'RT-mse': 'Running Time-Mean Standard Error',

    #Inverse:

    # For Test_Log_Data_Single_Run():
    'Target Pose Number'                        : 'TPN',
    'Success'                                   : 'SUC',
    'Number of Starting Point Trials'           : 'NSPT',
    'Number of Iterations'                      : 'NI',
    'Number of Success'                         : 'NS', 
    'Running Time (ms)'                         : 'RT',
    'Total Running Time (ms)'                   : 'TRT',
    'Average Running Time (ms)'                 : 'ART', 
    'Total Number of Iterations'                : 'TNI', 
    'Average Number of Iterations'              : 'ANI',
    'Percentage of Success'                     : 'PS',
    'Initial Configuration'                     : 'IC',
    'Final Configuration'                       : 'FC',
    # For Test_Log_Data_Statistics():

    'Number of Iterations-Total'                : 'NI-tot',
    'Number of Iterations-Maximum'              : 'NI-max',
    'Number of Iterations-Minimum'              : 'NI-min',
    'Number of Iterations-Average'              : 'NI-avr',
    'Number of Iterations-Standard Deviation'   : 'NI-sdv',
    'Number of Iterations-Mean Standard Error'  : 'NI-mse',
    'Number of Trials-Maximum'                  : 'NT-max',
    'Number of Trials-Minimum'                  : 'NT-min',
    'Number of Trials-Average'                  : 'NT-avr',
    'Running Time-Total (ms)'                   : 'RT-tot',
    'Running Time-Maximum (ms)'                 : 'RT-max',
    'Running Time-Minimum (ms)'                 : 'RT-min',
    'Running Time-Average (ms)'                 : 'RT-avr',
    'Running Time-Standard Deviation (ms)'      : 'RT-sdv', 
    'Running Time-Mean Standard Error'          : 'RT-mse'
    }

class Single_Run_Log():
    '''
    Contains data structure for evaluation test results for a single run associated with a target pose in PPS
    Including:
        Initial Status (Configuration, Endeffector Pose, Values of error functions)
        Final   Status (Configuration, Endeffector Pose, Values of error functions)

    '''    
    
    def __init__(self, pose_number, success = False, config_in_range = False):

        # Index of target pose in the target pose workspace 
        self.target_pose_num     = pose_number
        # Number of starting point trials for this target pose
        self.num_trial           = 0
        # Number of successful attempts until now   
        self.num_suc_til_now     = 0
        # Boolean property indicating if the implementation was successful or not
        self.success             = success
        self.config_in_range     = config_in_range
    
        # Number of iterations for this target pose
        self.num_iter            = 0
        # Total number of iterations until now
        self.num_iter_til_now    = 0
    
        # Running time for this configuration
        self.run_time            = 0.0
        # Total running time until now
        self.run_time_til_now    = 0.0
        # Average running time for one iteration
        self.mean_stp_time       = 0.0

        self.start_config_str    = ''
        self.final_config_str    = ''
        self.start_pose_str      = ''
        self.final_pose_str      = ''
        self.str_parameter_set   = ['TPN', 'S', 'CIR', 'NSPT', 'NI', 'RT','NS', 'TRT', 'ART', 'TNI', 'ANI','PS', 'IC', 'FC', 'IP', 'FP']
        self.csv_parameter_set   = ['TPN', 'S', 'CIR', 'NSPT', 'NI', 'RT','NS', 'TRT', 'ART', 'TNI', 'ANI', 'PS']

    def parameter_value(self, parameter):
        if parameter == 'TPN':
            return str(self.target_pose_num)
        elif parameter == 'S':
            return str(self.success)
        elif parameter == 'CIR':
            return str(self.config_in_range)
        elif parameter == 'NSPT':
            return str(self.num_trial)
        elif parameter == 'NI':
            return str(self.num_iter)
        elif parameter == 'RT':
            return str(1000*self.run_time)
        elif parameter == 'NS':
            return str(self.num_suc_til_now)
        elif parameter == 'TRT':
            return str(1000*self.run_time_til_now)
        elif parameter == 'ART':
            return str(1000*self.run_time_til_now/(self.target_pose_num + 1))
        elif parameter == 'TNI':
            return str(self.num_iter_til_now)
        elif parameter == 'ANI':
            return str(self.num_iter_til_now/(self.target_pose_num + 1))
        elif parameter == 'PS':
            return str(100.00*self.num_suc_til_now/(self.target_pose_num + 1))
        elif parameter == 'IC':
            return self.start_config_str
        elif parameter == 'FC':
            return self.final_config_str
        elif parameter == 'IP':
            return '\n' + '------------' + '\n' + self.start_pose_str
        elif parameter == 'FP':
            return '\n' + '------------' + '\n' + self.final_pose_str

    def __str__(self, parameter_set = None):
        if parameter_set == None:
            parameter_set = self.str_parameter_set
        s =   '\n'
        s  += 'Test Result:' + '\n\n'
        for p in parameter_set:
            value = self.parameter_value(p)
            param = key_dic[p]
            s +=  param + " "*(32-len(param)) +': ' + value + '\n'

        s += '\n' + '______________________________________________________________________________________________________________________________'
        
        return s

    def csv(self, header = True, parameter_set = None):
        if parameter_set == None:
            parameter_set = self.csv_parameter_set

        if header:
            s  = 'Parameter' + ',' + 'Value' +'\n'
        else:
            s = ''
        for p in parameter_set:
            value = self.parameter_value(p)
            s += key_dic[p] + "," + value + '\n'
        return s

    def csv_horizontal_header(self, parameter_set = None):
        if parameter_set == None:
            parameter_set = self.csv_parameter_set
        s = ''
        for p in parameter_set:
            s += key_dic[p] + ","
        s  = s[0:len(s) - 1]
        return s

    def csv_horizontal(self, parameter_set = None):
        if parameter_set == None:
            parameter_set = self.csv_parameter_set
        s = ''
        for p in parameter_set:
            s += self.parameter_value(p) + ','
        s  = s[0:len(s) - 1]

        return s

    def write_csv(self, filename):
        print 'Test_Log_Data_Single_Run(): Writing csv file started ...'
        CSV_FILE_HANDLE = open(filename, "w")
        CSV_FILE_HANDLE.write(self.csv())
        print 'Test_Log_Data_Single_Run(): Writing csv file ended.'
        
class Test_Statistics():
    '''
    Contains structure for statistical data of evaluation test results for a set of target poses
    Including:
        Initial Status (Configuration, Endeffector Pose, Values of error functions)
        Final   Status (Configuration, Endeffector Pose, Values of error functions)
            
    '''    
    def __init__(self):
        
        self.num_success                = 0
        self.num_inrange                = 0
        self.num_suc_inrange            = 0
        self.num_run                    = 0

        self.sum_num_iter               = 0              # Total number of iterations for all target poses
        self.max_num_iter               = 0              # Maximum number of iterations
        self.max_num_iter_pose_number   = 0              # Pose number corresponding to maximum number of iterations
        self.min_num_iter               = 1000000        # Minimum number of iterations
        self.min_num_iter_pose_number   = 0              # Pose number corresponding to minimum number of iterations
        
        self.sum_num_trial              = 0              # Total number of trials for all target poses
        self.max_num_trial              = 0              # Maximum number of trials
        self.max_num_trial_pose_number  = 0              # Pose number corresponding to maximum number of trials
        self.min_num_trial              = 1000000        # Minimum number of trials
        self.min_num_trial_pose_number  = 0              # Pose number corresponding to minimum number of trials
        
        self.max_run_time               = 0.0
        self.max_run_time_pose_number   = 0              # Pose number corresponding to maximum running time
        self.min_run_time               = 1000000.00
        self.min_run_time_pose_number   = 0              # Pose number corresponding to minimum running time
        self.sum_run_time               = 0.0
        
        self.mean_num_iter              = 0.0
        self.mean_num_trial             = 0.0
        self.mean_run_time              = 0.0
        self.mean_stp_time              = 0.0
        self.sd_num_iter                = 0.0
        self.sd_run_time                = 0.0
        self.mse_num_iter               = 0.0
        self.mse_run_time               = 0.0

        self.median_num_iter            = 0.0
        self.median_run_time            = 0.0
        
        self.str_parameter_set = ['NRun','NSRun','NSIR','PS','NI-tot', 'NI-max', 'NI-min', 'NI-avr','NI-sdv','NI-mse', 
                                  'NT-max', 'NT-min','NT-avr', 'RT-tot', 'RT-max', 'RT-min', 'RT-avr', 'RT-mse']
        self.csv_parameter_set = ['PS', 'NI-avr','NI-mse', 'RT-max', 'RT-min', 'RT-avr', 'RT-mse']

    def parameter_value(self, parameter):
        if parameter == 'NRun':
            return str(self.num_run)
        elif parameter == 'NSRun':
            return str(self.num_success)
        elif parameter == 'NSIR':
            return str(self.num_suc_inrange)
        elif parameter == 'NIR':
            return str(self.num_inrange)
        elif parameter == 'PS':
            return vecmat.value_to_str(100*self.num_success/float(self.num_run))
        elif parameter == 'PSIR':
            return vecmat.value_to_str(100*self.num_suc_inrange/float(self.num_run))
        elif parameter == 'NI-tot':
            return str(self.sum_num_iter)
        elif parameter == 'NI-max':
            # return str(self.max_num_iter) + " for target pose number: " + str(self.max_num_iter_pose_number)
            return str(self.max_num_iter)
        elif parameter == 'NI-min':
            # return str(self.min_num_iter) + " for target pose number: " + str(self.min_num_iter_pose_number)
            return str(self.min_num_iter)
        elif parameter == 'NI-avr':
            return vecmat.value_to_str(self.mean_num_iter)
        elif parameter == 'NI-med':
            return vecmat.value_to_str(self.median_num_iter)
        elif parameter == 'NI-sdv':
            return vecmat.value_to_str(self.sd_num_iter)
        elif parameter == 'NI-mse':
            return vecmat.value_to_str(self.mse_num_iter)
        elif parameter == 'NT-max':
            # return str(self.max_num_trial) + ' for target pose number: ' + str(self.max_num_trial_pose_number)
            return vecmat.value_to_str(self.max_num_trial)
        elif parameter == 'NT-min':
            # return str(self.min_num_trial) + ' for target pose number: ' + str(self.min_num_trial_pose_number)
            return vecmat.value_to_str(self.min_num_trial)
        elif parameter == 'NT-avr':
            return vecmat.value_to_str(self.mean_num_trial)
        elif parameter == 'RT-tot':
            return vecmat.value_to_str(1000*self.sum_run_time)
        elif parameter == 'RT-max':
            return vecmat.value_to_str(1000*self.max_run_time)
        elif parameter == 'RT-min':
            return vecmat.value_to_str(1000*self.min_run_time)
        elif parameter == 'RT-avr':
            return vecmat.value_to_str(1000*self.mean_run_time)
        elif parameter == 'RT-med':
            return vecmat.value_to_str(1000*self.median_run_time)
        elif parameter == 'RT-sdv':
            return vecmat.value_to_str(1000*self.sd_run_time)
        elif parameter == 'RT-mse':
            return vecmat.value_to_str(1000*self.mse_run_time)
        else:
            assert False, genpy.err_str(__name__, __class__.__name__, 'parameter_value', parameter + ' is an invalid value for parameter')

    def __str__(self, parameter_set = None):
        if parameter_set == None:
            parameter_set = self.str_parameter_set
        s =   '\n'
        s  += 'Test Statistics:' + '\n\n'
        for p in parameter_set:
            value = self.parameter_value(p)
            param = key_dic[p]
            s +=  param + " "*(40-len(param)) +': ' + value + '\n'

        s += '\n' + '______________________________________________________________________________________________________________________________'
        
        return s

    def csv(self, header = True, parameter_set = None):
        if parameter_set == None:
            parameter_set = self.csv_parameter_set
        if header:
            s  = 'Parameter' + ',' + 'Value' +'\n'
        else:
            s = ''
        for p in parameter_set:
            value = self.parameter_value(p)
            s += key_dic[p] + "," + value + '\n'
        return s

    def csv_horizontal_header(self, parameter_set = None, use_key = False):
        if parameter_set == None:
            parameter_set = self.csv_parameter_set
        s = ''
        for p in parameter_set:
            if use_key:
                s += p + ","
            else:
                s += key_dic[p] + ","
        s  = s[0:len(s) - 1]
        return s

    def csv_horizontal(self, parameter_set = None):
        if parameter_set == None:
            parameter_set = self.csv_parameter_set
        s = ''
        for p in parameter_set:
            s += self.parameter_value(p) + ','
        s  = s[0:len(s) - 1]
        return s

    def write_csv(self, filename):
        print 'Eval_Log_Data_Statistics(): Writing csv file started ...'
        CSV_FILE_HANDLE = open(filename, "w")
        CSV_FILE_HANDLE.write(self.csv())
        print 'Eval_Log_Data_Statistics(): Writing csv file ended.'
        
    def calculate_statistics(self, body):   

        self.__init__()
        sum_stp_time = 0.0
        
        self.num_run = len(body)
        num_iter_list  = []
        num_trial_list = []
        run_time_list  = []

        for i in range(0, self.num_run):
            num_iter_list.append(body[i].num_iter)
            num_trial_list.append(body[i].num_trial)
            run_time_list.append(body[i].run_time)
            
            self.sum_num_iter   += body[i].num_iter
            self.sum_num_trial  += body[i].num_trial
            self.sum_run_time   += body[i].run_time
            sum_stp_time        += body[i].mean_stp_time
            
            if body[i].num_iter > self.max_num_iter:
                self.max_num_iter = body[i].num_iter
                self.max_num_iter_pose_number = i
            if body[i].num_iter < self.min_num_iter:
                self.min_num_iter = body[i].num_iter
                self.min_num_iter_pose_number = i

            if body[i].num_trial > self.max_num_trial:
                self.max_num_trial = body[i].num_trial
                self.max_num_trial_pose_number = i
            if body[i].num_trial < self.min_num_trial:
                self.min_num_trial = body[i].num_trial
                self.min_num_trial_pose_number = i

            if body[i].run_time > self.max_run_time:
                self.max_run_time = body[i].run_time
                self.max_run_time_pose_number = i
            if body[i].run_time < self.min_run_time:
                self.min_run_time = body[i].run_time
                self.min_run_time_pose_number = i

        assert self.sum_run_time == body[self.num_run - 1].run_time_til_now
        assert self.sum_num_iter == body[self.num_run - 1].num_iter_til_now
    
        self.mean_num_iter  = float(self.sum_num_iter) / self.num_run
        self.mean_num_trial = float(self.sum_num_trial) / self.num_run
        self.mean_run_time  = self.sum_run_time / self.num_run
        self.mean_stp_time  = sum_stp_time / self.num_run

        self.median_num_iter  = statistics.median(num_iter_list)
        self.median_run_time  = statistics.median(run_time_list)

        sum_var_num_iter  = 0
        sum_var_run_time  = 0
    
        self.num_inrange      = 0
        self.num_success      = 0
        self.num_suc_inrange  = 0
        for i in range(0, self.num_run):
            
            sum_var_num_iter += (float(body[i].num_iter) - self.mean_num_iter) ** 2
            sum_var_run_time += (      body[i].run_time  - self.mean_run_time) ** 2

            self.num_inrange     += body[i].config_in_range
            self.num_success     += body[i].success
            self.num_suc_inrange += (body[i].config_in_range and body[i].success)
        
        if self.num_run > 1:
            den = self.num_run - 1
        else:
            den = 1

        var_num_iter = sum_var_num_iter / den
        var_run_time = sum_var_run_time / den
        
        self.sd_num_iter   = math.sqrt(var_num_iter)
        self.sd_run_time   = math.sqrt(var_run_time)
        self.mse_num_iter  = self.sd_num_iter / math.sqrt(den)
        self.mse_run_time  = self.sd_run_time / math.sqrt(den)

        assert self.num_success == body[self.num_run - 1].num_suc_til_now
        
class Test_Log():
    '''
    Contains data structure for evaluation test results for a set of runs associated with multiple target poses 
    Including:
        Header:
            Test Settings (An instance of "Kinematic_Manager_Settings")
        Body:
            Log data for each configuration (A list of instances of "Run_Log")
        Footer
            Statistic Data (An instance of "Run_Log_Statistics")
            
    '''    
    def __init__(self, km_settings ):
        self.header = km_settings
        self.body   = [] # should be an array of instances of Run_Log()
        self.footer = Test_Statistics()

    def write_log(self, filename):
        print 'Test_Log(): Writing log file started ...'
        LOG_FILE_HANDLE = open(filename, "w")
        LOG_FILE_HANDLE.write(str(self.header))
        LOG_FILE_HANDLE.write("\n" + "--------------------------------------------------------------------------------" + "\n")
        for body_log in self.body:
            LOG_FILE_HANDLE.write(str(body_log))
        LOG_FILE_HANDLE.write("\n" + "--------------------------------------------------------------------------------" + "\n")
        LOG_FILE_HANDLE.write(str(self.footer))
        print 'Test_Log: Writing log file ended.'

    def write_self(self, filename):
        print 'Test_Log: Writing self file started ...'
        SELF_FILE_HANDLE = open(filename, "w")
        pickle.dump(self, SELF_FILE_HANDLE)
        print 'Test_Log: Writing self file ended.'

    def write_csv(self, filename):
        print 'Test_Log: Writing csv file started ...'
        CSV_FILE_HANDLE = open(filename, "w")
        x = self.body[0]
        CSV_FILE_HANDLE.write(x.csv_horizontal_header() + '\n')
        for x in self.body:
            CSV_FILE_HANDLE.write(x.csv_horizontal() + '\n')
        print 'Test_Log: Writing csv file ended.'

# \endcond
