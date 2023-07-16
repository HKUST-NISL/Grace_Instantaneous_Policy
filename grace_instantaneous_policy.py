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

#Misc
file_path = os.path.dirname(os.path.realpath(getsourcefile(lambda:0)))
sys.path.append(os.path.join(file_path, '..'))
from CommonConfigs.grace_cfg_loader import *
from CommonConfigs.logging import setupLogger

#Respond to exit signal
def handle_sigint(signalnum, frame):
    # terminate
    print('Main interrupted! Exiting.')
    sys.exit()

def sample_rectified_exponential(mean, min_val, max_val):
    return min(max_val,
               max(
                    min_val,
                    numpy.random.exponential(mean)
               )
              )

class InstantaneousPolicy(StateMachine):

    def __init__(self, config_data, logger = None):
        #miscellaneous
        signal(SIGINT, handle_sigint)

        #Config
        self.__config_data = config_data

        #Policy uses its own logger (output to its own dir) or the input logger
        if(logger == None):
            self.__logger = setupLogger(
                        logging.DEBUG, 
                        logging.INFO, 
                        self.__class__.__name__,
                        os.path.join(file_path,"./logs/log_") + datetime.now().strftime(self.__config_data['Custom']['Logging']['time_format']))
        else:
            self.__logger = logger.getChild(self.__class__.__name__)


        #policy related
        self.__next_aversion_interval = self.__config_data['InstPolicy']['Misc']['no_stamp_val']
        self.__aversion_dur = self.__config_data['InstPolicy']['Misc']['no_stamp_val']

        self.__bc_ref_stamp = {
                            'robot_nodding': self.__config_data['InstPolicy']['Misc']['no_stamp_val'], 
                            'robot_humming': self.__config_data['InstPolicy']['Misc']['no_stamp_val']
                            }
        self.__bc_interval = {
                            'robot_nodding': self.__config_data['InstPolicy']['Misc']['no_stamp_val'], 
                            'robot_humming': self.__config_data['InstPolicy']['Misc']['no_stamp_val']
                            }


    def applyPolicy(self, state_inst):
        gaze_action = self.__gazePolicy(state_inst)
        bc_action = self.__bcPolicy(state_inst)
        return {'gaze_action': gaze_action, 'bc_action': bc_action}

    def __gazePolicy(self, state_inst):
        gaze_action = None
        if(self.__macro_robot_uttering(state_inst)):
            #When robot starts speaking or humming , gaze should be following
            if(self.__macro_robot_uttering_transition(state_inst)):
                gaze_action = self.__config_data['BehavExec']['General']['head_gaze_follow']
            else:
                #No need to repeated publish following command
                pass
        else:
            #When robot is not uttering anything, alternate between following and aversion
            gaze_action = self.__alternatingGazeAction(state_inst)
        return gaze_action

    def __macro_robot_uttering(self, state_inst):
        #robot is speaking or humming
        return (state_inst['robot_speaking']['val'] == self.__config_data['InstState']['StateCode']['r_speaking'] 
                or
                state_inst['robot_humming']['val'] == self.__config_data['InstState']['StateCode']['r_humming'])

    def __macro_robot_uttering_transition(self, state_inst):
        #robot uttering state transition just occurred
        return (state_inst['robot_speaking']['transition']
                or
                state_inst['robot_humming']['transition'])

    def __macro_robot_just_stopped_averting(self, state_inst):
        #robot gaze just starts following
        return (state_inst['robot_gaze']['val'] == self.__config_data['InstState']['StateCode']['r_following'] 
                and state_inst['robot_gaze']['transition'])

    def __alternatingGazeAction(self,state_inst):
        #Refer to the timestamp of the aversion / following state to decide whether 
        #the robot should alternate gaze behavior at this iteration
        gaze_action = None

        #Sample critical time stamps
        if( 
            #If robot just finished speaking or humming (enteres here means not uttering & transition flag)
            self.__macro_robot_uttering_transition(state_inst) 
            #or if the robot just starts to do following
            or self.__macro_robot_just_stopped_averting(state_inst)
            ):
            #Sample a time interval after which we will start aversion
            self.__next_aversion_interval = sample_rectified_exponential(
                    self.__config_data['InstPolicy']['GazeBehavSpec']['aversion_mean_interval'],
                    self.__config_data['InstPolicy']['GazeBehavSpec']['aversion_min_interval'],
                    self.__config_data['InstPolicy']['GazeBehavSpec']['aversion_max_interval']
                    )

            #Sample the duration of performing aversion
            self.__aversion_dur = numpy.random.uniform(
                        self.__config_data['InstPolicy']['GazeBehavSpec']['aversion_dur_range'][0],
                        self.__config_data['InstPolicy']['GazeBehavSpec']['aversion_dur_range'][1])
            self.__logger.info("Next aversion in %f sec, dur %f." % (self.__next_aversion_interval, self.__aversion_dur) )


        #Check against time stamps
        dur_gaze_state = time.time() - state_inst['robot_gaze']['stamp']
        if( state_inst['robot_gaze']['val'] == self.__config_data['InstState']['StateCode']['r_following'] ):
            if( self.__next_aversion_interval != self.__config_data['InstPolicy']['Misc']['no_stamp_val']
                and
                dur_gaze_state >= self.__next_aversion_interval):
                gaze_action = self.__config_data['BehavExec']['General']['head_gaze_avert'] #Following timeup, start aversion
                # self.__logger.debug("Start aversion")
                self.__next_aversion_interval = self.__config_data['InstPolicy']['Misc']['no_stamp_val']
            else:
                ##No need to repeatedly publish following command
                #gaze_action = self.__config_data['BehavExec']['General']['head_gaze_follow']
                pass
        else:
            if( self.__aversion_dur != self.__config_data['InstPolicy']['Misc']['no_stamp_val']
                and
                dur_gaze_state >= self.__aversion_dur):
                gaze_action = self.__config_data['BehavExec']['General']['head_gaze_follow'] #Aversion timeup, back to following
                # self.__logger.debug("Start following")
                self.__aversion_dur = self.__config_data['InstPolicy']['Misc']['no_stamp_val']
            else:
                ##No need to repeatedly publish aversion command
                #gaze_action = self.__config_data['BehavExec']['General']['head_gaze_avert']
                pass 

        return gaze_action

    def __macro_robot_speaking(self, state_inst):
        return state_inst['robot_speaking']['val'] == self.__config_data['InstState']['StateCode']['r_speaking'] 

    def __macro_human_turn_over(self, state_inst):
        return ( state_inst['turn_ownership']['transition'] 
                and state_inst['turn_ownership']['from'] == self.__config_data['InstState']['StateCode']['turn_h'])

    def __bcPolicy(self, state_inst):
        bc_action = {'nodding': None, 'hum': None}

        if(state_inst['turn_ownership']['transition']):
            #Reset the timing upon turn transition
            self.__clearAllTriggers()

        if( not self.__macro_robot_speaking(state_inst) ):
            if( state_inst['turn_ownership']['val'] == self.__config_data['InstState']['StateCode']['turn_h'] ):
                #In human's turn, (don't further specify for now)
                bc_action = self.__humanTurnBC(state_inst)
            else:
                #In indefinite / robot's turn, (don't further specify for now)
                bc_action = self.__robotTurnBC(state_inst)
                
        return bc_action 

    def __humanTurnBC(self, state_inst):
        #Refer to the timestamp of the robot not-humming state & robot not-nodding state
        #to decide whether the robot should output some bc at this instant
        bc_action = {}

        #Nodding policy routine
        nodding_trigger = self.__standardBCTrigger(
                                state_inst,
                                'robot_nodding',
                                self.__config_data['InstState']['StateCode']['r_n_nodding'],
                                self.__config_data['InstPolicy']['NoddingSpec']['mean_interval'],
                                self.__config_data['InstPolicy']['NoddingSpec']['min_interval'],
                                self.__config_data['InstPolicy']['NoddingSpec']['max_interval'])
        if(nodding_trigger):
            bc_action['nodding'] = self.__config_data['BehavExec']['General']['nod_cmd']
        else:
            bc_action['nodding'] = None

        #Hum policy routine
        hum_trigger = self.__standardBCTrigger(
                                state_inst,
                                'robot_humming',
                                self.__config_data['InstState']['StateCode']['r_n_humming'],
                                self.__config_data['InstPolicy']['HUMSpec']['human_turn']['mean_interval'],
                                self.__config_data['InstPolicy']['HUMSpec']['human_turn']['min_interval'],
                                self.__config_data['InstPolicy']['HUMSpec']['human_turn']['max_interval'])
        if(hum_trigger):
                bc_action['hum'] = {
                    'cmd': self.__config_data['BehavExec']['General']['hum_behav_exec_cmd'],
                    'content': self.__config_data['InstPolicy']['HUMSpec']['predefined']['debug_human_turn_hum']
                    }
        else:
            bc_action['hum'] = None

        return bc_action
            
    def __robotTurnBC(self, state_inst):
        #Refer to the timestamp of the robot not-speaking state & robot not-nodding state
        #to decide whether the robot should output some bc at this instant
        bc_action = {}

        #No nodding in robot turn
        bc_action['nodding'] = None

        #Hum policy routine
        hum_trigger = self.__standardBCTrigger(
                                state_inst,
                                'robot_humming',
                                self.__config_data['InstState']['StateCode']['r_n_humming'],
                                self.__config_data['InstPolicy']['HUMSpec']['robot_turn']['mean_interval'],
                                self.__config_data['InstPolicy']['HUMSpec']['robot_turn']['min_interval'],
                                self.__config_data['InstPolicy']['HUMSpec']['robot_turn']['max_interval'])

        if(self.__macro_human_turn_over(state_inst)):
            #Just transisted from human turn to robot turn
            #force a special bc immediately, including nodding and utterance immediately
            bc_action['hum'] = {
                'cmd': self.__config_data['BehavExec']['General']['hum_behav_exec_cmd'],
                'content': self.__config_data['InstPolicy']['HUMSpec']['predefined']['debug_special_col']
                }
        else:
            if(hum_trigger):
                content = None
                if(state_inst['turn_ownership']['val'] == self.__config_data['InstState']['StateCode']['turn_r']):
                    content = self.__config_data['InstPolicy']['HUMSpec']['predefined']['debug_robot_turn_hum']
                elif(state_inst['turn_ownership']['val'] == self.__config_data['InstState']['StateCode']['turn_no']):
                    content = self.__config_data['InstPolicy']['HUMSpec']['predefined']['debug_not_owned_turn_hum']
                else:
                    self.__logger.error("Unexpected state!!")
                
                bc_action['hum'] = {
                    'cmd': self.__config_data['BehavExec']['General']['hum_behav_exec_cmd'],
                    'content': content
                    }
            else:
                bc_action['hum'] = None

        return bc_action

    def __clearAllTriggers(self):
        self.__bc_ref_stamp = {
                            'robot_nodding':  time.time(), 
                            'robot_humming': time.time()
                            }
        self.__bc_interval = {
                            'robot_nodding': self.__config_data['InstPolicy']['Misc']['no_stamp_val'], 
                            'robot_humming': self.__config_data['InstPolicy']['Misc']['no_stamp_val']
                            }

    def __standardBCTrigger(self,
                    state_inst,
                    monitored_state,
                    not_acting_state_val,
                    mean_interval,
                    min_interval,
                    max_interval):
        perform_BC = False

        #If we are currently in the not-acting state
        not_acting = (state_inst[monitored_state]['val'] == not_acting_state_val)
        
        #Update ref stamp if we just transited into this not-acting state
        if(state_inst[monitored_state]['transition']): 
            self.__bc_ref_stamp[monitored_state] = state_inst[monitored_state]['stamp']

        #Whether we need to sample a new interval
        sample_trigger = not_acting and (self.__bc_interval[monitored_state] == self.__config_data['InstPolicy']['Misc']['no_stamp_val'])

        if( sample_trigger ):
            #Sample for the interval till next BC
            self.__bc_interval[monitored_state] = sample_rectified_exponential(mean_interval,min_interval,max_interval)
            self.__logger.info("Next %s in %f sec." % (monitored_state ,self.__bc_interval[monitored_state]))


        #Compare against the sampled interval
        if( not_acting 
            and 
            self.__bc_ref_stamp[monitored_state] != self.__config_data['InstPolicy']['Misc']['no_stamp_val']):
            not_acting_dur = time.time() - self.__bc_ref_stamp[monitored_state]
            if( self.__bc_interval[monitored_state] != self.__config_data['InstPolicy']['Misc']['no_stamp_val']
                and
                not_acting_dur >= self.__bc_interval[monitored_state]):

                perform_BC = True

                #Clear interval, update ref stamp
                self.__bc_interval[monitored_state] = self.__config_data['InstPolicy']['Misc']['no_stamp_val']
                self.__bc_ref_stamp[monitored_state] = time.time()


        return perform_BC