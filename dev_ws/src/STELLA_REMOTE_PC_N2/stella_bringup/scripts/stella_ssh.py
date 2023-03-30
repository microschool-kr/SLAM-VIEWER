import paramiko
import time
import os
import rospy
import socket
import rosnode
import rosgraph
import subprocess
import sys
from custom_gui.srv import Check, CheckResponse
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist

class ConnectRobot():
    def __init__(self):
        rospy.init_node('stella_ssh')
        self.service = rospy.Service("connect_robot", Check, self.handle_check)
        self.client = rospy.ServiceProxy("connect_lost", Empty)
        self.robot_ip = os.getenv("ROS_CLIENT")
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=100)
        self.is_opened = False
        self.is_connected = False
        self.proc = None


    def openSSH(self):
        if not self.is_up():
            print("network")
            return False, "network"
                
        self.ssh = paramiko.SSHClient()
        self.ssh.set_missing_host_key_policy(paramiko.MissingHostKeyPolicy())

        try:
            self.ssh.connect(self.robot_ip, username="odroid", password="odroid", timeout=3)
        except socket.timeout as e:
            print(e)
            return False, "timed_out"

        except Exception as e:
            print(type(e))
            print(e)
            return False, str(e)
            
        try:
            self.channel = self.ssh.invoke_shell()
            outdata, errdata = self.waitStreamsCheck()
            print(outdata)
            self.channel.send(f"sudo date -s @{time.time()}\n")
            outdata, errdata = self.waitStreamsCheck()
            print(outdata)
            self.channel.send(f"odroid\n")
            outdata, errdata = self.waitStreamsCheck()
            print(outdata)
            self.channel.send(f"export ROS_MASTER_URI={os.getenv('ROS_MASTER_URI')}\n")
            outdata, errdata = self.waitStreamsCheck()
            print(outdata)
            self.channel.send(f"export ROS_HOSTNAME={self.robot_ip}\n")
            outdata, errdata = self.waitStreamsCheck()
            print(outdata)
            self.is_opened = True
            return True, "opened"

        except TimeoutError as e:
            print(e)
            return False, "timed_out"
            
        except Exception as e:
            print(type(e))
            print(e)
            return False, str(e)

    def is_up(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(3)
        result = sock.connect_ex((self.robot_ip, 22))
        return result == 0

    def checkChannel(self):
        if self.is_opened:
            if not self.is_up():
                try:
                    response = self.client()
                except rospy.ServiceException as e:
                    print(f"Service call failed : {e}")
                else:
                    self.is_opened = False


    def waitStreams(self):
        time.sleep(0.5)
        outdata = errdata = ""
        while self.channel.recv_ready():
            outdata += self.channel.recv(1000).decode('utf-8')
        while self.channel.recv_stderr_ready():
            errdata += self.channel.recv_stderr(1000).decode('utf-8')
        return outdata, errdata

    def waitStreamsCheck(self, word="odroid:"):
        outdata = errdata = ""
        while True:
            while self.channel.recv_ready():
                outdata += self.channel.recv(1000).decode('utf-8')
                if word in outdata:
                    return outdata, errdata
            while self.channel.recv_stderr_ready():
                errdata += self.channel.recv_stderr(1000).decode('utf-8')
                
    def waitStreamsLidar(self):
        outdata = errdata = ""
        # start = time.time()
        # while time.time() - start < 25:
        while True:
            while self.channel.recv_ready():
                outdata += self.channel.recv(1000).decode('utf-8')
                if "Now YDLIDAR is scanning" in outdata:
                    return "connected", outdata, errdata
                elif "unable to contact" in outdata:
                    return "retry", outdata, errdata
            while self.channel.recv_stderr_ready():
                errdata += self.channel.recv_stderr(1000).decode('utf-8')
                if "Now YDLIDAR is scanning" in errdata:
                    return "connected", outdata, errdata
                elif "unable to contact" in errdata:
                    return "retry", outdata, errdata
        # return "scan_time_out", outdata, errdata

    def connect(self):
        try:
            self.channel.send("roslaunch stella_bringup stella_robot.launch\n")
            msg, outdata, errdata = self.waitStreamsLidar()
            print(outdata)
            
            if msg == "connected":
                self.is_connected = True
                return True, msg
            else:
                return False, msg

        except TimeoutError as e:
            print(e)
            return False, "timed_out"
        
        except Exception as e:
            print(type(e))
            print(e)
            return False, str(e)

    def disconnect(self):
        try:
            self.pub.publish()
            self.channel.send(chr(3))
            outdata, errdata = self.waitStreamsCheck()
            print(outdata)
            self.is_connected = False
            return True, "disconnected"

        except TimeoutError as e:
            print(e)
            return False, "timed_out"
        
        except Exception as e:
            print(type(e))
            print(e)
            return False, str(e)


    def handle_check(self, req):
        if req.data == "open":
            ret, msg = self.openSSH()
            return CheckResponse(ret, msg)

        elif req.data == "control":
            self.proc = subprocess.Popen(["roslaunch", "custom_gui", "custom_slam.launch"], shell=False)
            ret, msg = self.connect()
            if not ret:
                return CheckResponse(ret, msg)
            return CheckResponse(True, "control")
        
        elif req.data == "mapping":
            self.proc = subprocess.Popen(["roslaunch", "custom_gui", "custom_slam.launch"], shell=False)
            ret, msg = self.connect()
            if not ret:
                return CheckResponse(ret, msg)
            return CheckResponse(True, "mapping")

        elif req.data == "navigation":
            self.proc = subprocess.Popen(["roslaunch", "custom_gui", "custom_navigation.launch", "map_file:=c:/dev_ws/map/mymap.yaml"], shell=False)
            ret, msg = self.connect()
            if not ret:
                return CheckResponse(ret, msg)
            return CheckResponse(True, "navigation")
            
        elif req.data == "disconnect":
            if self.proc:
                subprocess.run(["taskkill", "/f", "/pid", str(self.proc.pid), "/t"])
                self.proc = None
            if self.is_connected:
                ret, msg = self.disconnect()
                if not ret:
                    return CheckResponse(ret, msg)
            rosnode.cleanup_master_whitelist(rosgraph.Master('/rosnode'), ["/rosout", "/custom_gui", "/stella_ssh"])
            return CheckResponse(ret, msg)

        elif req.data == "off":
            if self.proc:
                subprocess.run(["taskkill", "/f", "/pid", str(self.proc.pid), "/t"])
                self.proc = None
            if self.is_connected:
                ret, msg = self.disconnect()
                if not ret:
                    return CheckResponse(ret, msg)
            rosnode.cleanup_master_whitelist(rosgraph.Master('/rosnode'), [])
            subprocess.run(["taskkill", "/f", "/t", "/im", "cmd.exe"])
            subprocess.run(["taskkill", "/f", "/t", "/im", "python.exe"])
            sys.exit()

        return CheckResponse(False, "unknown")

    def __del__(self):
        self.ssh.close()
    

if __name__ == "__main__":
    connect_robot = ConnectRobot()
    loop_rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        connect_robot.checkChannel()
        loop_rate.sleep()
    # rospy.spin()