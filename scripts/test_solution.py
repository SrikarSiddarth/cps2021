#!/usr/bin/env python
"""
    JD March 21, 2021
    Harish
    2020-05-14
"""

import rospy
from gazebo_msgs.srv import DeleteModel
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped, Vector3, Quaternion
from mavros_msgs.msg import State, PositionTarget, AttitudeTarget
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse

from mavros_msgs.srv import SetMode, CommandBool
from std_msgs.msg import String, Header, Bool
from numpy import sqrt, pi, cos, arctan2
g = 9.80665

class OffboardControl:
    """ Controller for PX4-UAV offboard mode """

    def __init__(self):
        rospy.init_node('OffboardControl', anonymous=True)
        self.curr_pose = PoseStamped()
        self.is_ready_to_fly = False
        self.mode = "BELLY-FLOP"
        self.arm = False
        self.att = AttitudeTarget()
        self.attach = False
        
#        self.separation = 1.07
        self.orientation = [0]*3      # orientation of the drone
        self.t = rospy.Time.now()
        self.probe_pos = [0]*3
		self.target_pos = [10,0]
        self.targetAngle = 0
        self.k = [0]*3
        self.range = sqrt(self.target_pos[0]**2+self.target_pos[1]**2)
        self.attitudeRate = 0
        self.maxYattRate = 16
        self.minYattRate = 3
        self.separation = sqrt((sqrt(2)-1)*g*self.range)/self.maxYattRate
#        if self.range>20:
#            self.separation = 0.025*self.range
#        else:
#            self.separation = 0.5
#	fixing the angle of throw to pi/8	
        self.height = self.range - self.separation*cos(pi/8)
        print(self.height, self.separation)
        self.attachFlag = 1
        self.poseFlag = 1
        self.pubFlag = 1

        # define ros subscribers and publishers
        self.armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
        self.detach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)

        
        self.pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, callback=self.pose_callback)
        self.state_sub = rospy.Subscriber('/mavros/state', State, callback=self.state_callback)
        self.decision_sub = rospy.Subscriber('/data', String, callback=self.set_mode)
        self.model_poses_sub = rospy.Subscriber('gazebo/model_states/',ModelStates,callback=self.model_states_cb)

        self.parachute_pub = rospy.Publisher('parachute_plugin/sample_probe',Bool,queue_size=10)
        self.att_setpoint_pub = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)
        self.attach_pub = rospy.Publisher('/attach', String, queue_size=10)
        
#        self.attach_models('sample_probe_0','base_link','sample_probe_1','base_link')

        self.controller()
#        self.test()

    def set_mode(self, msg):
        self.mode = str(msg.data)

    def model_states_cb(self, msg):
        self.probe_pos[0] = msg.pose[1].position.x
        self.probe_pos[1] = msg.pose[1].position.y
        self.probe_pos[2] = msg.pose[1].position.z
#	self.target_pos[0] = msg.pose[1].position.x
#	self.target_pos[1] = msg.pose[1].position.y
#	self.range = sqrt(self.target_pos[0]**2+self.target_pos[0]**2)

    def pose_callback(self, msg):
        self.curr_pose = msg
        self.orientation = euler_from_quaternion((msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w))
#	print(self.orientation)

    def state_callback(self, msg):
        if msg.mode == 'OFFBOARD' and self.arm == True:
            self.is_ready_to_fly = True
        else:
            self.take_off()

    def detach_chute(self):
        
        try:
            chuteService = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
            isModeChanged = chuteService(model_name='parachute_small')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. OFFBOARD Mode could not be set. Check that GPS is enabled" % e)


    def set_offboard_mode(self):
        rospy.wait_for_service('/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
            isModeChanged = flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. OFFBOARD Mode could not be set. Check that GPS is enabled" % e)

    def set_arm(self):
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            self.armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
            self.armService(True)
            self.arm = True
        except rospy.ServiceException as e:
            print("Service arm call failed: %s" % e)

    def take_off(self):
        self.set_offboard_mode()
        self.set_arm()


    def attach_models(self, model1, link1, model2, link2):
        req = AttachRequest()
        req.model_name_1 = model1
        req.link_name_1 = link1
        req.model_name_2 = model2
        req.link_name_2 = link2
        self.attach_srv.call(req)

    def detach_models(self, model1, link1, model2, link2):
        req = AttachRequest()
        req.model_name_1 = model1
        req.link_name_1 = link1
        req.model_name_2 = model2
        req.link_name_2 = link2
        self.detach_srv.call(req)


    def retro(self):
        print(self.orientation,'o')
        print(self.curr_pose.pose.position.x,self.curr_pose.pose.position.y,self.curr_pose.pose.position.z)
        self.detach_models('if750a','base_link','sample_probe','base_link')
        print('detached the models!!!')
        self.mode = "LAND"
        print('changed the mode to LAND')
            

    def land(self):
        rate = rospy.Rate(15)  # Hz
        try:
            flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
            isModeChanged = flightModeService(custom_mode='AUTO.LAND')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. OFFBOARD Mode could not be set. Check that GPS is enabled" % e)
        while self.mode == "LAND" and not rospy.is_shutdown():
            if self.pubFlag:
                if self.probe_pos[2]<self.height*0.4 and self.pubFlag:
                    self.parachute_pub.publish(1)
    	            print('Altitude too low!! Calling for parachute')
                    self.pubFlag = 0
                elif sqrt(self.probe_pos[0]**2+self.probe_pos[1]**2) > 0.95*self.range:
                    self.parachute_pub.publish(1)
                    print('Entered the landing range!! Calling the parachute')
                    self.pubFlag = 0

            if self.probe_pos[2]<0.2 and self.poseFlag:
                print('probe landed!!!')
                print(self.probe_pos, sqrt(self.probe_pos[0]**2+self.probe_pos[1]**2))
                self.poseFlag = 0
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    def belly_flop(self):
        rate = rospy.Rate(15)  # Hz
        self.set_offboard_mode()
        self.att.body_rate = Vector3()
        self.att.header = Header()
        self.att.header.frame_id = "base_footprint"
        self.attach = True
        self.chute_attached = True

        while self.mode == "BELLY-FLOP" and not rospy.is_shutdown():
#            print(self.orientation)
            self.att.header.stamp = rospy.Time.now()
            self.att.thrust = 0.7
            self.att.type_mask = 128
            
            if self.curr_pose.pose.position.z > self.separation and self.attachFlag:
                self.attach_models('if750a','base_link','sample_probe','base_link')
                print('model attached!!')
                self.attachFlag = 0

            if self.curr_pose.pose.position.z > self.height:
#                self.att.body_rate.y = 32/(1+1.7**(self.attitudeRate + 1.295))
#                self.att.body_rate.y = (0.68*self.range - 2.9)*(2.7**(-self.attitudeRate))
                self.att.body_rate.y = 0.65*sqrt((sqrt(2)-1)*g*self.range)/self.separation * (1.5**(-self.attitudeRate))
                print(self.attitudeRate, self.att.body_rate.y)
                self.attitudeRate += 0.1

            # pi/4 = 0.7853981634
            if self.orientation[1]>pi/2 - pi/4:
                self.att.thrust = 2
                self.att_setpoint_pub.publish(self.att)
                rate.sleep()

                self.mode="RETRO"
                print(' set the mode to retro')

            self.att_setpoint_pub.publish(self.att)
            

            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass



    def controller(self):
        """ A state machine developed to have UAV states """
        while not rospy.is_shutdown():
            if self.mode =="BELLY-FLOP":
                print("belly flop!")
                self.belly_flop()
            elif self.mode == "RETRO":
                self.retro()
            elif self.mode == "LAND":
                self.land()

    def test(self):
        r = rospy.Rate(15)
        while not rospy.is_shutdown():
            if time.time() - self.t>10:
                self.parachute_pub.publish(1)
                break


if __name__ == "__main__":
    
    OffboardControl()


