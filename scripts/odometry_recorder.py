#!/usr/bin/env python
import rospy, rospkg
import math
import time
import sys, os
import signal
import subprocess
import threading
import yaml, json
# File operations
import datetime
# import dircache # <-- Python2.x only, repalce with os.listdir
import shutil
# Args
import argparse
#-------------------------#
try:
    import queue as Queue # Python 3.x
except:
    import Queue # Python 2.x
#-------------------------#
from std_msgs.msg import (
    Empty,
    Bool,
    String,
)



def _record_cmd_callback(data):
    """
    The callback function for operation command.
    """
    global _rosbag_caller
    if data.data:
        _rosbag_caller.start(_warning=True)
    else:
        _rosbag_caller.stop(_warning=True)

def _backup_trigger_callback(data):
    """
    The callback function for operation command.
    """
    global _rosbag_caller
    _rosbag_caller.backup(reason=data.data)




def main(sys_args):



    # Process arguments
    parser = argparse.ArgumentParser(description="Record ROS messages to rosbag files with enhanced controllability.\nThere are mainly two usage:\n- Manual-mode: simple start/stop record control\n- Auto-mode: Continuous recording with files backup via triggers.")
    #---------------------------#
    # Explicitly chose to auto-mode or manual-mode (exculsive)
    group = parser.add_mutually_exclusive_group()
    group.add_argument("-A", "--auto", action="store_true", help="continuous recording with triggered backup")
    group.add_argument("-M", "--manual", action="store_true", help="manually control the start/stop of recorder")
    # UI setting
    parser.add_argument("--NO_KEY_IN", action="store_true", help="disable the stdin (keyboard) user-input")
    # Full setting, the following setting will overwrite the above settings.
    parser.add_argument("-d", "--PARAM_DIR", help="specify the directory of the setting-file and topics-file")
    parser.add_argument("-s", "--SETTING_F", help="specify the filename of the setting-file")
    parser.add_argument("-t", "--TOPICS_F", help="specify the filename of the topics-file")
    #---------------------------#
    # _args = parser.parse_args()
    _args, _unknown = parser.parse_known_args()



    #
    rospy.init_node('odometer', anonymous=True)
    #
    _node_name = rospy.get_name()[1:] # Removing the '/'
    print("_node_name = %s" % _node_name)
    #
    rospack = rospkg.RosPack()
    _pack_path = rospack.get_path('msg_recorder')
    print("_pack_path = %s" % _pack_path)
    # Loading parameters
    #---------------------------------------------#
    rospack = rospkg.RosPack()
    pack_path = rospack.get_path('msg_recorder')
    f_path = _pack_path + "/params/"

    # Manual mode
    f_name_params = "rosbag_setting.yaml"

    # Read param file
    #------------------------#
    _f = open( (f_path+f_name_params),'r')
    params_raw = _f.read()
    _f.close()
    param_dict = yaml.load(params_raw)
    #------------------------#




    # Print the params
    # print("param_dict = %s" % str(param_dict))
    print("\nsettings (in json format):\n%s" % json.dumps(param_dict, indent=4))


    # test, the param_dict after combination
    # print("param_dict = %s" % str(param_dict))
    # print("param_dict (in json format):\n%s" % json.dumps(param_dict, indent=4))
    #---------------------------------------------#

    # Init ROS communication interface
    #--------------------------------------#
    # Subscriber
    rospy.Subscriber("/REC/record", Bool, _record_cmd_callback)
    rospy.Subscriber("/REC/req_backup", String, _backup_trigger_callback)
    # Publisher
    _recorder_running_pub = rospy.Publisher("/REC/is_recording", Bool, queue_size=10, latch=True) #
    _recorder_running_pub.publish(False)
    _trigger_event_report_pub = rospy.Publisher("/REC/trigger_report", String, queue_size=20, latch=True) #
    #--------------------------------------#




    # Determine if we are using keyboard input
    _is_key_in = _args.NO_KEY_IN


    # Loop for user command via stdin
    while not rospy.is_shutdown():
        #
        time.sleep(0.5)
    print("End of main loop.")




if __name__ == '__main__':

    try:
        main(sys.argv)
    except rospy.ROSInterruptException:
        pass
    print("End of odometer.")
