#general
import yaml
import rospy
import os
import re
import threading
from signal import signal
from signal import SIGINT
import logging
import sys
from datetime import datetime
import time
from inspect import getsourcefile
from os.path import abspath
from statemachine import StateMachine, State
import numpy

#ros
import dynamic_reconfigure.client
import sensor_msgs.msg
import std_msgs.msg
import hr_msgs.msg
import grace_attn_msgs.msg
import grace_attn_msgs.srv
import hr_msgs.msg
import hr_msgs.cfg
import hr_msgs.srv
import std_msgs


#Load configs
def loadConfig(path):
    #Load configs
    with open(path, "r") as config_file:
        config_data = yaml.load(config_file, Loader=yaml.FullLoader)
        # print("Config file loaded")
    return config_data

#Create Logger
def setupLogger(file_log_level, terminal_log_level, logger_name, log_file_name):
    log_formatter = logging.Formatter('%(asctime)s %(msecs)03d %(name)s %(levelname)s %(funcName)s(%(lineno)d) %(message)s', 
                                  datefmt='%d/%m/%Y %H:%M:%S')

    f = open(log_file_name, "a")
    f.close()
    file_handler = logging.FileHandler(log_file_name)
    file_handler.setFormatter(log_formatter)
    file_handler.setLevel(file_log_level)

    stream_handler = logging.StreamHandler()
    stream_handler.setFormatter(log_formatter)
    stream_handler.setLevel(terminal_log_level)

    logger = logging.getLogger(logger_name)
    logger.addHandler(file_handler)
    logger.addHandler(stream_handler)
    logger.setLevel( min(file_log_level,terminal_log_level) )#set to lowest

    return logger

#Respond to exit signal
def handle_sigint(signalnum, frame):
    # terminate
    print('Main interrupted! Exiting.')
    sys.exit()



class InstantaneousPolicy(StateMachine):

    def __init__(self):
        #miscellaneous
        signal(SIGINT, handle_sigint)
        self.__logger = setupLogger(
                    logging.DEBUG, 
                    logging.INFO, 
                    self.__class__.__name__,
                    "./logs/log_" + datetime.now().strftime("%a_%d_%b_%Y_%I_%M_%S_%p"))

        path = os.path.dirname(os.path.realpath(getsourcefile(lambda:0))) + "/config/config.yaml"
        self.__config_data = loadConfig(path)



    def applyPolicy(self, state_inst, state_prog):
        gaze_action = self.__gazePolicy(state_inst, state_prog)
        bc_action = self.__bcPolicy(state_inst, state_prog)
        return {'gaze_action': gaze_action, 'bc_action': bc_action}

    def __gazePolicy(self, state_inst, state_prog):
        gaze_action = ''
        if( state_inst['robot_speaking']['val'] == self.__config_data['StateCode']['r_speaking'] ):
            #When robot is speaking (be it bc or genuine speech), gaze should be following
            gaze_action = 'following'
        else:
            #When robot is not speaking, alternate between following and aversion
            gaze_action = self.__alternatingGazeAction(state_inst)
        return gaze_action

    def __alternatingGazeAction(self,state_inst):
        #Refer to the timestamp of the aversion / following state to decide whether 
        #the robot should alternate gaze behavior at this iteration
        gaze_action = None

        #Sample critical time stamps
        if( state_inst['robot_speaking']['transition'] #If robot just finished speaking
            or (
                state_inst['robot_gaze']['val'] == state_inst['StateCode']['r_following'] #Or if robot just starts following
                and state_inst['robot_gaze']['transition']
                )
            ):
            #Sample a time interval after which we will start aversion
            self.__next_aversion_interval = numpy.random.exponential(
                        self.__config_data['GazeBehavSpec']['aversion_mean_interval'])
            #Sample the duration of performing aversion
            self.__aversion_dur = numpy.random.uniform(
                        self.__config_data['GazeBehavSpec']['aversion_dur_range'][0],
                        self.__config_data['GazeBehavSpec']['aversion_dur_range'][1])

        #Check against time stamps
        dur_gaze_state = time.time() - state_inst['robot_gaze']['stamp']
        if( state_inst['robot_gaze']['val'] == state_inst['StateCode']['r_following'] ):
            if( dur_gaze_state >= self.__next_aversion_interval):
                gaze_action = 'aversion' #Timeup, start aversion
        else:
            if( dur_gaze_state >= self.__aversion_dur):
                gaze_action = 'following' #Timeup, back to following


    def __bcPolicy(self, state_inst, state_prog):
        bc_action = {}
        bc_action.setdefault('nodding', False)
        bc_action.setdefault('hum', None)

        if( state_inst['turn_ownership']['val'] == self.__config_data['StateCode']['turn_h'] ):
            #In human's turn, (don't further specify for now)
            bc_action = self.__humanTurnBC(state_inst)
        else:
            #In indefinite / robot's turn, (don't further specify for now)
            bc_action = self.__robotTurnBC(state_inst)

        return bc_action
        

    def __humanTurnBC(self, state_inst):
        #Refer to the timestamp of the robot not-speaking state & robot not-nodding state
        #to decide whether the robot should output some bc at this instant
        bc_action = None

        #For now we use a standard routine  with different specs
        nodding_trigger = __standardBCTrigger(
                                'robot_nodding',
                                self.__config_data['StateCode']['r_n_nodding'],
                                self.__config_data['NoddingSpec']['mean_interval'])
        bc_action['nodding'] = nodding_trigger


        hum_trigger = __standardBCTrigger(
                                'robot_speaking',
                                self.__config_data['StateCode']['r_n_speaking'],
                                self.__config_data['HUMSpec']['human_turn']['mean_interval'])
        if(hum_trigger):
            bc_action['hum'] = 'human turn hum'
        else:
            bc_action['hum'] = None
            
    def __robotTurnBC(self, state_inst):
        #Refer to the timestamp of the robot not-speaking state & robot not-nodding state
        #to decide whether the robot should output some bc at this instant
        bc_action = None

        if( state_inst['turn_ownership']['transition'] ):
            #Just transisted from human turn to robot turn
            #force a special bc immediately, including nodding and utterance
            bc_action['hum'] = 'special confirm bc'

            return bc_action 

        else:
            #For now we use a standard routine  with different specs
            hum_trigger = __standardBCTrigger(
                                    'robot_speaking',
                                    self.__config_data['StateCode']['r_n_speaking'],
                                    self.__config_data['HUMSpec']['robot_turn']['mean_interval'])
            if(hum_trigger):
                bc_action['hum'] = 'human turn hum'
            else:
                bc_action['hum'] = None

    def __standardBCTrigger(self,
                    state_inst,
                    monitored_state,
                    not_acting_state_val,
                    mean_interval):
        perform_BC = False
        #If we are currently in the not-acting state
        not_acting = (state_inst[monitored_state]['val'] == not_acting_state_val)
        #If we just transited into this not-acting state
        sample_trigger = not_acting and state_inst[monitored_state]['transition']

        if( sample_trigger ):
            #Sample for the interval till next BC
            self.__bc_stamp[monitored_state] = numpy.random.exponential(mean_interval)

        #Compar against the sampled interval
        if( not_acting ):
            not_acting_dur = time.time() - state_inst[monitored_state]['stamp']
            if(not_acting_dur >= self.__bc_stamp[monitored_state]):
                perform_BC = True

        return perform_BC