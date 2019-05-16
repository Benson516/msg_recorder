#!/usr/bin/env python
import rospy
import time
import sys, os
import signal
import subprocess
import threading
import yaml, json


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
    def __init__(self, param_dict):
        """
        The param_dict contains the following elements:
        - output_dir (default: "./"): The directry of the output bag files
        - bag_name_prefix (default: ""): The prefix for the generated bag files
        - is_splitting (default: False): Determine if the output bag should be split (once reach the size limit or duration limit)
        - split_size (default: 1024 MB): The size to split the file
        - record_duration (default: None): 30 (30 sec.), 1m (1 min.), 1h (1 hr.), the duration for recording before stopping/splitting
        - is_compressed (default: false): raw data or compressed data to store (the file all ends with .bag, to check if it's compressed, use rosbag info)
        - compression_method (default: lz4): lz4/bz2; bz2: slow, high compression; lz4: fast, low compression
        - is_recording_all_topics (default: True): To record all topics or not
        - topics (default: []): a list of topics names (with or without "/" are allowabled)
        """
        self._thread_rosbag = None
        self._ps = None
        self.rosbag_node_name = "rosbag_subprocess_1"

        # Parameters for rosbag record with default values
        self.output_dir = param_dict.get('output_dir', "./")
        self.bag_name_prefix = param_dict.get('bag_name_prefix', "")
        self.is_splitting = param_dict.get('is_splitting', False)
        self.split_size = param_dict.get('split_size', "1024")
        self.record_duration = param_dict.get('record_duration', None)
        self.is_compressed = param_dict.get('is_compressed', False)
        self.compression_method = param_dict.get('compression_method', "lz4")
        self.is_recording_all_topics = param_dict.get('is_recording_all_topics', True)
        self.topic_list = param_dict.get('topics', [])

        # test
        print("self.record_duration = %s" % str(self.record_duration))


        # Preprocessing for parameters
        self.output_dir = os.path.expandvars( os.path.expanduser(self.output_dir) )
        print("self.output_dir = %s" % self.output_dir)


    def start(self, _warning=False):
        """
        To start recording.
        """
        if not self._is_thread_valid():
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
        """
        if self._is_thread_valid(): # subprocess is running
            return self._terminate_rosbag(_warning=_warning)
        else:
            if _warning:
                print("rosbag is not running, no action.")
            return False


    # Private methods
    def _is_thread_valid(self):
        """
        Check if the thread is running.
        """
        return ( (not self._thread_rosbag is None) and self._thread_rosbag.is_alive() )

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
        subprocess.call("cd " + self.output_dir, shell=True)
        subprocess.call("pwd", shell=True)
        """
        try:
            _out = subprocess.check_output(["mkdir", self.output_dir], stderr=subprocess.STDOUT)
            print("The directory <%s> has been created." % self.output_dir)
        except:
            print("The directry <%s> already exists." % self.output_dir)
            pass

        # The command
        cmd_list = ["rosbag", "record", ("__name:=%s" % self.rosbag_node_name)]
        # File name prefix
        cmd_list += ["-o", self.bag_name_prefix]
        # Splitting
        if self.is_splitting:
            cmd_list += ["--split", "--size=%s" % self.split_size]
        # Duration
        if not self.record_duration is None:
            cmd_list += ["--duration=%s" % self.record_duration]
        # Compression
        if self.is_compressed:
            cmd_list += ["--%s" % self.compression_method]
        # topics
        if self.is_recording_all_topics:
            cmd_list += ["-a"]
        else:
            cmd_list += self.topic_list
        #
        print("")
        print("Executing command: %s" % cmd_list)
        print("Working directry: %s" % self.output_dir)
        print("")
        # self._ps = subprocess.Popen(["rosbag", "record", "__name:=rosbag_subprocess", "-a"]) # rosbag
        # self._ps = subprocess.Popen(["rosbag", "record", ("__name:=%s" % self.rosbag_node_name), "-a"]) # rosbag
        # self.Ps = subprocess.Popen("rosbag record __name:=rosbag_subprocess -a", shell=True) # rosbag
        self._ps = subprocess.Popen(cmd_list, cwd=self.output_dir)
        #----------------------------#
        return True

    def _terminate_rosbag(self, _warning=False):
        """
        The wraper for killing rosbag
        """
        try:
            # self._ps.terminate() # TODO: replace this to a signal to the thread
            # self._ps.send_signal(signal.SIGTERM) # <-- This method cannot cleanly kill the rosbag.
            # subprocess.Popen(["rosnode", "kill", "/rosbag_subprocess"]) # <-- this method can close the rosbag cleanly.
            subprocess.Popen(["rosnode", "kill", ("/%s" % self.rosbag_node_name) ]) # <-- this method can close the rosbag cleanly.
            return True
        except:
            if _warning:
                print("The process does not exit.")
            return False
    #-------------------------------------#

    def _rosbag_watcher(self):
        """
        This function run as a thread to look after the rosbag process.
        """
        # The private method to start the process
        self._open_rosbag()
        print("=== Subprocess started.===")
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
        # self._ps = None
        return




def main(args):
    rospy.init_node('msg_recorder', anonymous=True)

    # Loading parameters
    #---------------------------------------------#
    f_path = "./"
    f_name_params = "rosbag_setting.yaml"
    f_name_topics = "record_topics.txt"

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

    _rosbag_caller = ROSBAG_CALLER(param_dict)
    while not rospy.is_shutdown():
        # A blocking std_in function
        str_in = raw_input("Type a command and press ENTER (s:start/t:terminate/q:quit): \n")
        #
        if str_in == 's':
            _rosbag_caller.start(_warning=True)
        elif str_in == 't':
            _rosbag_caller.stop(_warning=True)
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
