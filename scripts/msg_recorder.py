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
import dircache
import shutil
# Args
import argparse

from std_msgs.msg import (
    Empty,
    Bool,
)

# _rosbag_caller
_rosbag_caller = None

# Publisher
_recorder_running_pub = None

# For clearing the last line on terminal screen
#---------------------------------------------------#
CURSOR_UP_ONE = '\x1b[1A'
ERASE_LINE = '\x1b[2K'
def delete_last_lines(n=1):
    for _ in range(n):
        sys.stdout.write(CURSOR_UP_ONE)
        sys.stdout.write(ERASE_LINE)
#---------------------------------------------------#


class ROSBAG_CALLER:
    """
    This is the function for handling the rosbag subprocess.
    """
    # Public methods
    def __init__(self, param_dict, node_name="msg_recorder"):
        """
        The param_dict contains the following elements:
        - output_dir_tmp (default: "./rosbag_tmp"): The directry of the output bag files, temporary stored
        - output_dir_kept (default: "./rosbag_backup"): The directry of the output bag files, palced for backuped files
        - bag_name_prefix (default: "record"): The prefix for the generated bag files
        - is_splitting (default: False): Determine if the output bag should be split (once reach the size limit or duration limit)
        - split_size (default: None/1024 MB(when record_duration is None) ): The size to split the file
        - max_split_num (default: None): Maximum number of split files. The oldest file will be delete When the one is generated.
        - record_duration (default: None): 30 (30 sec.), 1m (1 min.), 1h (1 hr.), the duration for recording before stopping/splitting
        - is_compressed (default: false): raw data or compressed data to store (the file all ends with .bag, to check if it's compressed, use rosbag info)
        - compression_method (default: lz4): lz4/bz2; bz2: slow, high compression; lz4: fast, low compression
        - buffsize (default: None <- 256MB): Input buffer size for all messages before bagging, empty -> None -> 256MB
        - chunksize (default: None <- 768KB): Advanced, memory to chunks of SIZE KB before writing to disk. Tune if needed.
        - is_recording_all_topics (default: True): To record all topics or not
        - topics (default: []): a list of topics names (with or without "/" are allowabled)
        #
        - time_pre_triger (default: 60.0 sec.): Keep all records since time_pre_triger
        - time_post_triger (default: 5.0 sec.): Keep all records before time_post_triger
        """
        # Variables
        self._thread_rosbag = None
        self._ps = None
        self.rosbag_node_name_suffix = "rosbag_subprocess"
        self.rosbag_node_name = node_name + "_"+ self.rosbag_node_name_suffix
        print("rosbag_node_name = %s" % self.rosbag_node_name)
        #
        self._last_trigger_timestamp = 0.0

        # Parameters for rosbag record with default values
        #--------------------------------------------------------------------------#
        self.output_dir_tmp = param_dict.get('output_dir_tmp', "./rosbag_tmp")
        self.output_dir_kept = param_dict.get('output_dir_kept', "./rosbag_backup")
        self.bag_name_prefix = param_dict.get('bag_name_prefix', "record")
        self.is_splitting = param_dict.get('is_splitting', False)
        self.split_size = param_dict.get('split_size', None)
        self.max_split_num = param_dict.get('max_split_num', None)
        self.record_duration = param_dict.get('record_duration', None)
        self.is_compressed = param_dict.get('is_compressed', False)
        self.compression_method = param_dict.get('compression_method', "lz4")
        self.buffsize = param_dict.get('buffsize', None)
        self.chunksize = param_dict.get('chunksize', None)
        self.is_recording_all_topics = param_dict.get('is_recording_all_topics', True)
        self.topic_list = param_dict.get('topics', [])
        #
        self.time_pre_triger = param_dict.get('time_pre_triger', 60.0)
        self.time_post_triger = param_dict.get('time_post_triger', 5.0)

        # The self.output_dir_tmp and self.output_dir_kept cannot be the same.
        if self.output_dir_tmp == self.output_dir_kept:
            if self.output_dir_kept[-1] != "/":
                self.output_dir_kept += "/"
            self.output_dir_kept += "rosbag_backup"

        # Add '/' at the end
        if self.output_dir_tmp[-1] != "/":
            self.output_dir_tmp += "/"
        if self.output_dir_kept[-1] != "/":
            self.output_dir_kept += "/"

        # If both split_size and record_duration are not specified, set the split size
        if self.is_splitting and self.record_duration is None:
            if (self.split_size is None) or (int(self.split_size) <= 0):
                self.split_size = 1024
                print("self.split_size is forced to <%d MB> since both split_size and record_duration are not specified." % int(self.split_size))
        #--------------------------------------------------------------------------#


        # test
        print("self.record_duration = %s" % str(self.record_duration))


        # Preprocessing for parameters
        self.output_dir_tmp = os.path.expandvars( os.path.expanduser(self.output_dir_tmp) )
        print("self.output_dir_tmp = %s" % self.output_dir_tmp)
        self.output_dir_kept = os.path.expandvars( os.path.expanduser(self.output_dir_kept) )
        print("self.output_dir_tmp = %s" % self.output_dir_kept)


        # Creating directories
        try:
            _out = subprocess.check_output(["mkdir", "-p", self.output_dir_tmp], stderr=subprocess.STDOUT)
            print("The directory <%s> has been created." % self.output_dir_tmp)
        except:
            print("The directry <%s> already exists." % self.output_dir_tmp)
            pass

        try:
            _out = subprocess.check_output(["mkdir", "-p", self.output_dir_kept], stderr=subprocess.STDOUT)
            print("The directory <%s> has been created." % self.output_dir_kept)
        except:
            print("The directry <%s> already exists." % self.output_dir_kept)
            pass


    def start(self, _warning=False):
        """
        To start recording.
        """
        if not self._is_thread_rosbag_valid():
            self._thread_rosbag = threading.Thread(target=self._rosbag_watcher)
            self._thread_rosbag.start()
            return True
        else:
            if _warning:
                print("rosbag is already running, no action")
            return False

    def stop(self, _warning=False):
        """
        To stop recording

        Note: If the stop() is called right after start(), then the rosnode kill might fail (the node is not intialized yet.)
        """
        if self._is_thread_rosbag_valid() and ( (not self._ps is None) and self._ps.poll() is None): # subprocess is running
            return self._terminate_rosbag(_warning=_warning)
        else:
            if _warning:
                print("rosbag is not running, no action.")
            return False

    def backup(self):
        """
        Backup all the files interset with time zone.

        Use a deamon thread to complete the work even if the program is killed.
        """
        _t = threading.Thread(target=self._keep_files_before_and_after)
        _t.daemon = True
        _t.start()

    # Private methods
    def _is_thread_rosbag_valid(self):
        """
        Check if the thread is running.
        """
        try:
            return ( (not self._thread_rosbag is None) and self._thread_rosbag.is_alive() )
        except:
            return False

    # Subprocess controll
    #-------------------------------------#
    def _open_rosbag(self):
        """
        The wraper for starting rosbag
        """
        # The program to be run
        #----------------------------#
        """
        # Note; changing working directry here has no effect on the other subprocess calls.
        subprocess.call("cd " + self.output_dir_tmp, shell=True)
        subprocess.call("pwd", shell=True)
        """
        # New subprocess
        #----------------------------#
        # The command
        cmd_list = ["rosbag", "record", ("__name:=%s" % self.rosbag_node_name)]
        # File name prefix
        cmd_list += ["-o", self.bag_name_prefix]
        # Splitting
        if self.is_splitting:
            cmd_list += ["--split"]
            if not self.split_size is None:
                cmd_list += ["--size=%d" % int(self.split_size)]
            if not self.max_split_num is None:
                cmd_list += ["--max-splits=%d" % int(self.max_split_num)]
        # Duration
        if not self.record_duration is None:
            cmd_list += ["--duration=%s" % self.record_duration]
        # Compression
        if self.is_compressed:
            cmd_list += ["--%s" % self.compression_method]
        # self.buffsize
        if not self.buffsize is None:
            cmd_list += ["--buffsize=%d" % int(self.buffsize)]
        # Memory chunksize
        if not self.chunksize is None:
            cmd_list += ["--chunksize=%d" % int(self.chunksize)]
        # topics
        if self.is_recording_all_topics:
            cmd_list += ["-a"]
        else:
            cmd_list += self.topic_list
        #
        print("")
        print("Executing command: %s" % cmd_list)
        print("Working directry: %s" % self.output_dir_tmp)
        print("")
        # self._ps = subprocess.Popen(["rosbag", "record", "__name:=rosbag_subprocess", "-a"]) # rosbag
        # self._ps = subprocess.Popen(["rosbag", "record", ("__name:=%s" % self.rosbag_node_name), "-a"]) # rosbag
        # self.Ps = subprocess.Popen("rosbag record __name:=rosbag_subprocess -a", shell=True) # rosbag
        self._ps = subprocess.Popen(cmd_list, cwd=self.output_dir_tmp)
        #----------------------------#
        return True

    def _terminate_rosbag(self, _warning=False):
        """
        The wraper for killing rosbag
        """
        # First try
        try:
            # self._ps.terminate() # TODO: replace this to a signal to the thread
            # self._ps.send_signal(signal.SIGTERM) # <-- This method cannot cleanly kill the rosbag.
            # subprocess.Popen(["rosnode", "kill", "/rosbag_subprocess"]) # <-- this method can close the rosbag cleanly.
            # subprocess.Popen(["rosnode", "kill", ("/%s" % self.rosbag_node_name) ]) # <-- this method can close the rosbag cleanly.
            subprocess.Popen("rosnode kill /%s" % self.rosbag_node_name, shell=True) # <-- this method can close the rosbag cleanly.
            # subprocess.call(["rosnode", "kill", ("/%s" % self.rosbag_node_name) ]) # <-- this method can close the rosbag cleanly.
            # subprocess.call("rosnode kill /%s" % self.rosbag_node_name, shell=True) # <-- this method can close the rosbag cleanly.
            return True
        except:
            if _warning:
                print("The process cannot be killed by rosnode kill.")
        # Second try
        try:
            self._ps.terminate() #
            if _warning:
                print("The rosbag is killed through SIGTERM, the bag might still be active.")
            return True
        except:
            print("Something wrong while killing the rosbag subprocess")
        #
        return False
    #-------------------------------------#

    def _rosbag_watcher(self):
        """
        This function run as a thread to look after the rosbag process.
        """
        global _recorder_running_pub
        # The private method to start the process
        self._open_rosbag()
        print("=== Subprocess started.===")
        _recorder_running_pub.publish(True)
        #
        time_start = time.time()
        while self._ps.poll() is None:
            duration = time.time() - time_start
            print("---Subprocess is running, duration = %f" % duration)
            # delete_last_lines()
            time.sleep(1.0)
        result = self._ps.poll()
        print("result = %s" % str(result))
        print("=== Subprocess finished.===")
        _recorder_running_pub.publish(False)

        # Clear the handle, indicating that no process is running
        # self._ps = None
        return



    # Backing up files
    #----------------------------------------------#
    def _get_latest_inactive_bag(self, timestamp=None):
        """
        This is a helper funtion for finding the latest (inactive) bag file.
        """
        file_list = dircache.listdir(self.output_dir_tmp)
        file_list.sort() # Sort in ascending order
        #
        if timestamp is None:
            target_date = datetime.datetime.now()
        else:
            target_date = datetime.datetime.fromtimestamp(timestamp)
        target_date_formate = target_date.strftime("%Y-%m-%d-%H-%M-%S")
        # print('target_date = %s' % str(target_date))
        # print('target_date_formate = %s' % target_date_formate)
        target_name_prefix_date = self.bag_name_prefix + '_' + target_date_formate
        print('target_name_prefix_date = %s' % target_name_prefix_date)
        # Seraching
        closest_file_name = None
        is_last = True
        # Assume the file_list is sorted in ascending order
        for i in range(len(file_list)):
            # if file_list[-1-i].rfind('.active') >= 0:
            if file_list[-1-i][-4:] != '.bag':
                # active file or other file type
                continue
            if file_list[-1-i][:len(self.bag_name_prefix)] != self.bag_name_prefix:
                # Not our bag
                continue
            if file_list[-1-i] < target_name_prefix_date:
                closest_file_name = file_list[-1-i]
                break
            else:
                is_last = False
        """
        # Assume the file_list is not sorted
        for i in range(len(file_list)):
            # if file_list[-1-i].rfind('.active') >= 0:
            if file_list[i][-4:] != '.bag':
                # active file or other file type
                continue
            if file_list[i][:len(self.bag_name_prefix)] != self.bag_name_prefix:
                # Not our bag
                continue
            if file_list[i] < target_name_prefix_date:
                if (closest_file_name is None) or (file_list[i] > closest_file_name):
                    # Note: None is actually smaller than anything
                    closest_file_name = file_list[i]
            else:
                is_last = False
            #
        """
        # Note: it's possible to return a None when there is no file in the directory or no inactive file before the given time
        # e.g. We are just recording the first bag (which is acive)
        return (closest_file_name, is_last)

    def _get_list_of_inactive_bag_in_timezone(self, timestamp_start, timestamp_end=None):
        """
        This is a helper funtion for finding the latest (inactive) bag file.
        """
        file_list = dircache.listdir(self.output_dir_tmp)
        file_list.sort() # Sort in ascending order
        #
        target_date_start = datetime.datetime.fromtimestamp(timestamp_start)
        if timestamp_end is None:
            target_date_end = datetime.datetime.now()
        else:
            target_date_end = datetime.datetime.fromtimestamp(timestamp_end)
        target_date_start_formate = target_date_start.strftime("%Y-%m-%d-%H-%M-%S")
        target_date_end_formate = target_date_end.strftime("%Y-%m-%d-%H-%M-%S")
        # print('target_date_start_formate = %s' % target_date_start_formate)
        # print('target_date_end_formate = %s' % target_date_end_formate)
        target_name_prefix_date_start = self.bag_name_prefix + '_' + target_date_start_formate
        target_name_prefix_date_end = self.bag_name_prefix + '_' + target_date_end_formate
        print('target_name_prefix_date_start = %s' % target_name_prefix_date_start)
        print('target_name_prefix_date_end = %s' % target_name_prefix_date_end)
        # Seraching
        file_in_zone_list = []
        # Assume the file_list is sorted in ascending order
        for i in range(len(file_list)):
            # if file_list[-1-i].rfind('.active') >= 0:
            if file_list[-1-i][-4:] != '.bag':
                # active file or other file type
                continue
            # Note that if self.bag_name_prefix is '', then the following is bypassed
            if file_list[-1-i][:len(self.bag_name_prefix)] != self.bag_name_prefix:
                # Not our bag
                continue
            if file_list[-1-i] < target_name_prefix_date_end:
                file_in_zone_list.append(file_list[-1-i])
                if file_list[-1-i] < target_name_prefix_date_start:
                    break
        # Note: it's possible to return an empty list when there is no file in the directory or no inactive file fall in the given time
        # e.g. We are just recording the first bag (which is acive)
        return file_in_zone_list


    def _keep_files_before_and_after(self):
        """
        To keep 2 files: before and after

        The file might look like the following when this function called:
        a.bag
        b.bag.active

        Solution (rough concept):
        1. Backup (copy) the a.bag immediately <-- This is already done in a deamon thread according to the way of this function call. (in case that the main program being closed)
        2. Start another thread (deamon, in case that the main program being closed) for listening that if the b.bag.active has become the b.bag

        Since the original rosbag using date as file name, we don't bother to track the file midification time.
        """
        # Get the current time
        _trigger_timestamp = time.time()
        self._last_trigger_timestamp = _trigger_timestamp
        #
        _pre_trigger_timestamp = _trigger_timestamp - self.time_pre_triger
        _post_trigger_timestamp = _trigger_timestamp + self.time_post_triger

        # Find all the "a.bag" files
        file_in_pre_zone_list = self._get_list_of_inactive_bag_in_timezone( _pre_trigger_timestamp, _trigger_timestamp)
        print("file_in_pre_zone_list = %s" % file_in_pre_zone_list)
        # Bacuk up "a.bag", note tha empty list is allowed
        for _F in file_in_pre_zone_list:
            shutil.copy2( (self.output_dir_tmp + _F), self.output_dir_kept)

        # Start a deamon thread for watching the "b.bag"
        _t = threading.Thread(target=self._keep_files_after, args=(_trigger_timestamp, _post_trigger_timestamp, file_in_pre_zone_list))
        _t.daemon = True
        _t.start()
        print("===Pre-triggered-file backup thread finished.===")

    def _keep_files_after(self, _trigger_timestamp, _post_trigger_timestamp, file_in_pre_zone_list):
        """
        This is a worker for listening the post-triggered bags.
        """
        # Wait ntil reached the _post_trigger_timestamp
        """
        # Note: this is not good for prone to lost the latest post-file.
        time.sleep(_post_trigger_timestamp - _trigger_timestamp)
        """
        # Start listening, first stage
        time_start = time.time()
        while self._is_thread_rosbag_valid():
            duration = time.time() - time_start
            print("---===Post-triggered file backup thread is running, duration = %f" % duration)
            #
            (closest_file_name, is_last) = self._get_latest_inactive_bag(_post_trigger_timestamp)
            if not closest_file_name in file_in_pre_zone_list:
                shutil.copy2( (self.output_dir_tmp + closest_file_name), self.output_dir_kept)
                file_in_pre_zone_list.append(closest_file_name)
            if not is_last:
                break
            time.sleep(1.0)
        #
        # Find all the rest "b.bag" files
        # Note: most of them had been backuped
        file_in_post_zone_list = self._get_list_of_inactive_bag_in_timezone( _trigger_timestamp, _post_trigger_timestamp)
        print("file_in_post_zone_list = %s" % file_in_post_zone_list)
        # Bacuk up "a.bag", note tha empty list is allowed
        for _F in file_in_post_zone_list:
            if not _F in file_in_pre_zone_list:
                shutil.copy2( (self.output_dir_tmp + _F), self.output_dir_kept)
        print("===Post-triggered file backup thread finished.===")
    #----------------------------------------------#



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
    _rosbag_caller.backup()







def main(sys_args):
    global _rosbag_caller
    global _recorder_running_pub

    # Process arguments
    parser = argparse.ArgumentParser(description="Record ROS messages to rosbag files with enhanced controllability.\nThere are mainly two usage:\n- Manual-mode: simple start/stop record control\n- Auto-mode: Continuous recording with files backup via triggers.")
    #---------------------------#
    # Explicitly chose to auto-mode or manual-mode (exculsive)
    group = parser.add_mutually_exclusive_group()
    group.add_argument("-A", "--auto", action="store_true", help="continuous recording with triggered backup")
    group.add_argument("-M", "--manual", action="store_true", help="manually control the start/stop of recorder")
    # Full setting, the following setting will overwrite the above settings.
    parser.add_argument("-d", "--PARAM_DIR", help="specify the directory of the setting-file and topics-file")
    parser.add_argument("-s", "--SETTING_F", help="specify the filename of the setting-file")
    parser.add_argument("-t", "--TOPICS_F", help="specify the filename of the topics-file")
    #---------------------------#
    _args = parser.parse_args()



    #
    rospy.init_node('msg_recorder', anonymous=True)
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
    f_name_topics = "record_topics.txt"


    # Overwriting default values
    #-----------------------------------#
    # Auto/manual
    if _args.auto:
        f_name_params = "rosbag_setting_auto.yaml"
        f_name_topics = "record_topics_auto.txt"
    elif _args.manual:
        # Default is manual-mode, nothing to do
        pass

    # Customize
    if not _args.PARAM_DIR is None:
        f_path = _args.PARAM_DIR
        if f_path[-1] != '/':
            f_path += '/'
    if not _args.SETTING_F is None:
        f_name_params = _args.SETTING_F
    if not _args.TOPICS_F is None:
        f_name_topics = _args.TOPICS_F
    #-----------------------------------#

    # Read param file
    #------------------------#
    _f = open( (f_path+f_name_params),'r')
    params_raw = _f.read()
    _f.close()
    param_dict = yaml.load(params_raw)
    #------------------------#

    # Read topic_list file
    #------------------------#
    topic_list = []
    _f = open( (f_path+f_name_topics),'r')
    for _s in _f:
        # Remove the space and '\n'
        _s1 = _s.rstrip().lstrip()
        if len(_s1) > 0: # Append non-empty string (after stripping)
            topic_list.append(_s1)
    _f.close()
    print("topic_list = %s" % str(topic_list))
    #------------------------#

    # Add the 'topics' to param_dict
    param_dict['topics'] = topic_list

    # test
    print("param_dict = %s" % str(param_dict))
    # print("param_dict (in json format):\n%s" % json.dumps(param_dict, indent=4))

    #---------------------------------------------#

    # Init ROS communication interface
    #--------------------------------------#
    # Subscriber
    rospy.Subscriber("/REC/rercord", Bool, _record_cmd_callback)
    rospy.Subscriber("/REC/req_backup", Bool, _backup_trigger_callback)
    # Publisher
    _recorder_running_pub = rospy.Publisher("/REC/is_recording", Bool, queue_size=10, latch=True) #
    _recorder_running_pub.publish(False)
    #--------------------------------------#





    # The manager for rosbag record
    #---------------------------------------------#
    _rosbag_caller = ROSBAG_CALLER(param_dict, _node_name)
    # Start at beginning
    if param_dict['start_at_begining']:
        _rosbag_caller.start(_warning=True)
    #---------------------------------------------#





    # Loop for user command via stdin
    while not rospy.is_shutdown():
        # A blocking std_in function
        str_in = raw_input("\n----------------------\nType a command and press ENTER:\n----------------------\ns:start \nt:terminate \nk:keep file \nq:quit \n----------------------\n>>> ")
        #
        if str_in == 's':
            _rosbag_caller.start(_warning=True)
        elif str_in == 't':
            _rosbag_caller.stop(_warning=True)
        elif str_in == 'k':
            _rosbag_caller.backup()
        elif str_in == 'q':
            _rosbag_caller.stop(_warning=False)
            break
        else:
            pass
        #
        time.sleep(0.5)
    print("End of main loop.")


if __name__ == '__main__':



    try:
        main(sys.argv)
    except rospy.ROSInterruptException:
        pass
    print("End of recorder.")