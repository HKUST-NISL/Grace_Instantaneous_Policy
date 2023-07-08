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


    '''
    Question:
        Format of template actions (bc)?
        Format of the output action definitions?
    '''


    def applyPolicy(self, state_inst, state_prog):
        gaze_action = gazePolicy(state_inst, state_prog)
        bc_action = bcPolicy(state_inst, state_prog)
        return Merge(gaze_action, bc_action)

    def gazePolicy(self, state_inst, state_prog):
        #No aversion when the robot is speaking
        if( state_inst['robot_speaking']['val'] ):
            #Do following
            pass
        else:
            #Alternate between following and aversion 
            #by an exponential distribution
            pass
        pass




    def bcPolicy(self, state_inst, state_prog):
        #No bc when the robot is speaking (either a stretch of speech or a bc is playing)
        if( state_inst['robot_speaking']['val'] ):
            #No bc
            pass
        else:
            if( state_inst['turn_ownership']['val'] == 'human_turn' ):
                #In human's turn, bc by acknowledgement and nodding 
                #by an exponential distribution
                pass
            else:
                #In robot's turn or indefinite turn,
                #bc by (different) acknowledgement
                pass
                






