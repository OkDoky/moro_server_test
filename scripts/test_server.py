#!/usr/bin/python

import rospy
import random
from threading import Thread
import traceback
from cv_bridge import CvBridge
import cv2

## import message files
from geometry_msgs.msg import Twist, Vector3, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import BatteryState, LaserScan, Image
from cai_msgs.msg import *

## import service files
from std_srvs.srv import SetBool
from cai_msgs.srv import *

class ConnectionHandler:
    def __init__(self):
        self.pubs, self.clients = {}, {}
        self.subs, self.srvs = [], []
        
        ## init values
        self.start_time = rospy.Time.now()
        self.rpose = RobotState()
        self.feedvel = Twist()
        self.bms = BatteryState()
        self.heading_angle = Vector3()
        self.scan = LaserScan()
        self.image = self.init_image()
        self.map = OccupancyGrid()
        self.feedback = JobFeedback()
        self.plan = Path()
        self.init_default_value()
        self.result = JobResultRequest()
        self.result.job_id = "test"
        
        ## init subscribers
        self.subs.append(rospy.Subscriber("API/cmd_vel", Twist, self.api_cmdvel_callback))
        self.subs.append(rospy.Subscriber("API/haed_ang", Vector3, self.api_haed_ang_callback))
        
        ## init service server
        self.srvs.append(rospy.Service("CoreNode/mode_change", StringRequest, self.mode_change_callback))
        self.srvs.append(rospy.Service("CoreNode/save_map_as", MapSave, self.save_map_callback))
        self.srvs.append(rospy.Service("JobScheduler/goal", JobGoal, self.goal_handler_callback))
        self.srvs.append(rospy.Service("JobScheduler/cancel", JobCancel, self.cancel_handler_callback))
        self.srvs.append(rospy.Service("API/follow_me_mode", SetBool, self.followme_handler_callback))
        
        ## init publishers
        self.pubs['robot_state'] = rospy.Publisher("robot_state", RobotState, queue_size=1)
        self.pubs['feedback_vel'] = rospy.Publisher("feedback_vel", Twist, queue_size=1)
        self.pubs['bms'] = rospy.Publisher("bms", BatteryState, queue_size=1)
        self.pubs['cur_head_angle'] = rospy.Publisher("current_head_angle", Vector3, queue_size=1)
        # self.pubs[''] = rospy.Publisher("errors", , queue_size=1)
        self.pubs['scan'] = rospy.Publisher("scan_filtered", LaserScan, queue_size=1)
        self.pubs['image'] = rospy.Publisher("camera/color/image_raw", Image, queue_size=1)
        # self.pubs[''] = rospy.Publisher("sonar", , queue_size=1)
        self.pubs['map'] = rospy.Publisher("map", OccupancyGrid, queue_size=1)
        self.pubs['feedback'] = rospy.Publisher("JobScheduler/feedback", JobFeedback, queue_size=1)
        self.pubs['plan'] = rospy.Publisher("move_base/GlobalPlanner/plan", Path, queue_size=1)

        ## init clients
        self.clients['result'] = rospy.ServiceProxy("JobScheduler/result",JobResult)

        ## init pub/client thread
        pub_delay = 1.0
        client_delay = 5.0
        self.pub_thread = Thread(target=self.publisher_thread, args=(pub_delay,))
        self.pub_thread.daemon = True
        self.pub_thread.start()
        
        self.cli_thread = Thread(target=self.client_thread, args=(client_delay,))
        self.cli_thread.daemon = True
        self.cli_thread.start()
        rospy.spin()
    
    ## init image
    def init_image(self):
        _bridge = CvBridge()
        img = cv2.imread('/root/catkin_ws/src/moro_server_test/map/map.pgm', cv2.IMREAD_COLOR) # Read the image file
        ros_img = _bridge.cv2_to_imgmsg(img, "bgr8")
        return ros_img
    
    ## init default values
    def init_default_value(self):
        self.rpose.rid = "r1"
        self.rpose.mode_state = 0
        self.rpose.bridge_state = 0
        
        self.bms.header.frame_id = "base_footprint"
        self.bms.present = False
        self.bms.serial_number = "11238885hhdfl"
        self.bms.location = "Ansan"
        
        self.scan.header.frame_id = "base_scan"
        self.scan.angle_min = -3.14159011841
        self.scan.angle_max = 3.14159011841
        self.scan.angle_increment = 0.00174581271131 ## 3600 ranges
        self.scan.time_increment = 0.0
        self.scan.range_min = 0.119
        self.scan.range_max = 10.0
        self.scan.intensities = [0.0 for _ in range(3600)]
        
        self.image.header.frame_id = "cam_1"
        
        self.feedback.header.frame_id = "base_footprint"
        self.feedback.job_id = "test"
        self.feedback.action_id = "test_1"
        self.feedback.action_type = 1
        self.feedback.current_action_index = 0
        self.feedback.pause_state = False
        self.feedback.action_state = 1
        self.feedback.error_code = 0
        self.feedback.elapsed_time = (rospy.Time.now() - self.start_time).to_sec()
        
        self.plan.header.frame_id = "odom"
        
        x = 5.0
        y = 10.0
        for i in range(100):
            point = PoseStamped()
            point.pose.position.x = x + i*0.1
            point.pose.position.y = y + i*0.1
            point.pose.orientation.w = 1.0
            self.plan.poses.append(point)
    
    ## init thread callbacks    
    def publisher_thread(self, delay):
        while not rospy.is_shutdown():
            try:
                self.random_init_pubvel()
                self.publish_topics()
            except Exception as e:
                rospy.logwarn("[TestServer] publisher thread, %s"%traceback.format_exc())
            finally:
                rospy.sleep(delay)
        
    def client_thread(self, delay):
        while not rospy.is_shutdown():
            try:
                self.random_init_clientvel()
                self.call_clients()
            except Exception as e:
                rospy.logwarn("[TestServer] client thread, %s"%traceback.format_exc())
            finally:
                rospy.sleep(delay)
                
    ## init random value maker & pub/call functions
    def random_init_pubvel(self):
        _time = rospy.Time.now()
        self.rpose.pose.x = random.uniform(-5.0, 5.0)
        self.rpose.pose.y = random.uniform(10.0, 100.0)
        self.rpose.pose.theta += 0.0001
        
        self.feedvel.linear.x = random.uniform(-0.05, 0.05)
        self.feedvel.angular.z = 0.0001
        
        self.bms.voltage = random.uniform(20.0, 28.0)
        self.bms.current = random.uniform(2.3, 5.0)
        self.bms.charge = random.uniform(23, 26)
        self.bms.capacity = random.randint(10,89)
        self.bms.percentage = random.randint(20,100)
        self.bms.cell_voltage = [random.uniform(20, 28),random.uniform(20, 28)]
        self.bms.header.stamp = _time
        
        self.heading_angle.z = random.uniform(0,90)
        
        self.scan.header.stamp = _time
        self.scan.ranges = [random.uniform(0.1, 10.0) for _ in range(3600)]
        
        self.feedback.header.stamp = _time
        self.feedback.elapsed_time = (_time - self.start_time).to_sec()
        
        self.plan.header.stamp = _time
        
    def random_init_clientvel(self):
        self.result.elapsed_time = (rospy.Time.now()-self.start_time).to_sec()
    
    def publish_topics(self):
        self.pubs['robot_state'].publish(self.rpose)
        self.pubs['feedback_vel'].publish(self.feedvel)
        self.pubs['bms'].publish(self.bms)
        self.pubs['cur_head_angle'].publish(self.heading_angle)
        self.pubs['scan'].publish(self.scan)
        self.pubs['image'].publish(self.image)
        # self.pubs['map'].publish(self.map)
        self.pubs['feedback'].publish(self.feedback)
        self.pubs['plan'].publish(self.plan)

    
    def call_clients(self):
        try:
            rospy.wait_for_service("JobScheduler/result", timeout=0.5)
            self.clients['result'].call(self.result)
        except rospy.ROSException as e:
            rospy.logwarn("[TestServer] client thread, if JobResult server is not ready, its ok, but if already working, than check what is wrong...")
            
    
    ## init subscriber callbacks
    def api_cmdvel_callback(self, msg):
        rospy.logwarn("[Topic][API/cmd_vel] got cmd vel, x : %f, \ntheta : %f"%(msg.linear.x, msg.angular.z))
    
    def api_haed_ang_callback(self, msg):
        rospy.logwarn("[Topic][API/head_ang] got heading angle, x : %f, \ntheta : %f"%(msg.x, msg.z))
    
    ## init service callback
    def mode_change_callback(self, req):
        rospy.logwarn("[Service][CoreNode/mode_chage] req mode : %s"%req.req)
        success = True
        code = 0
        return success, code
    
    def save_map_callback(self, req):
        rospy.logwarn("[Service][CoreNode/save_map_as] map name : %s, mode : %d, demension : %d"%(req.map_name, req.mode, req.demension))
        success = True
        code = 0
        return success, code
    
    def goal_handler_callback(self, req):
        _job = ""
        _job += "job id : %s\n"%req.job_id
        _job += "num_repeat : %d\n"%req.num_repeat
        for i in range(len(req.action_list)):
            _job += "[%d] action_type : %d"%(i+1,req.action_list[i].action_type)
            for j in range(len(req.action_list[i].action_args)):
                _job += "[%d] arg name : %s, "%(j+1,req.action_list[i].action_args[j].arg_name)
                _job += "arg type : %s, "%req.action_list[i].action_args[j].type
                _job += "arg value : %s\n"%req.action_list[i].action_args[j].value
            for k in range(len(req.action_list[i].action_params)):
                _job += "[%d] param name : %s, "%(k+1,req.action_list[i].action_params[k].param_name)
                _job += "param type : %s, "%req.action_list[i].action_params[k].type
                _job += "param value : %s\n"%req.action_list[i].action_params[k].value
                
        rospy.logwarn("[Service][JobScheduler/goal] job info : %s"%_job)
        success = True
        error_code = 0
        return success, error_code
    
    def cancel_handler_callback(self, req):
        rospy.logwarn("[Service][JobScheduler/cancel] job id : %s, action id : %s"%(req.job_id, req.action_id))
        success = True
        error_code = 0
        return success, error_code
    
    def followme_handler_callback(self, req):
        rospy.logwarn("[Service][API/follow_me_mode] req data : %s"%req.data)
        success = True
        message = "success"
        return success, message
    
if __name__ == "__main__":
    rospy.init_node("TestServer")
    ConnectionHandler()
    
    while not rospy.is_shutdown():
        rospy.spinOnce()
        rospy.sleep(1)